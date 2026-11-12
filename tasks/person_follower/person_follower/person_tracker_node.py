"""Stage 2: track one person across frames, not just detect people.

Detection alone re-decides "who is the target" every frame, so with two
people in view the choice can flip between them. Here ByteTrack assigns a
persistent id to each person and the node locks onto exactly one.

Target policy:
  - On startup, lock the largest person in frame (nearest the camera is
    the best guess at "the operator who just started this").
  - Hold that id as long as it keeps appearing.
  - If it disappears, keep publishing nothing but stay locked for
    reacquire_secs, so walking behind something does not hand the robot
    to a bystander.
  - After that window expires, fall back to the largest person again.

Publishes the same /target_person contract as Stage 1, so anything
downstream is unaffected by the switch.

Run with:  ros2 run person_follower person_tracker
Press Q or ESC to quit, and T to force re-lock onto the largest person.
"""

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from ultralytics import YOLO

IMAGE_TOPIC = "/conduit/camera/front/image_raw/compressed"
INFO_TOPIC = "/conduit/camera/front/camera_info"
TARGET_TOPIC = "/target_person"
WINDOW = "Stage 2 - person tracking"

PERSON_CLASS = 0


class PersonTracker(Node):
    def __init__(self):
        super().__init__("person_tracker")

        self.declare_parameter("image_topic", IMAGE_TOPIC)
        self.declare_parameter("info_topic", INFO_TOPIC)
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.4)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("show", True)
        # How long to keep a lock alive with the target unseen. Long enough
        # to survive an occlusion, short enough that a genuinely departed
        # target does not block re-locking forever.
        self.declare_parameter("reacquire_secs", 3.0)
        # Exponential smoothing on the published bearing and box height.
        # The raw box centre jitters by several degrees as the detector
        # redraws it each frame -- especially when the person turns
        # sideways and the box narrows -- and the car chases that jitter.
        # 0 = no smoothing, 0.9 = very heavy. 0.6 keeps it responsive.
        self.declare_parameter("smoothing", 0.6)
        # Only re-lock onto a new person if they are at least this close to
        # where the old target was last seen, as a fraction of frame width.
        # Stops the car silently adopting a stranger across the room.
        self.declare_parameter("relock_max_shift", 0.35)
        # Frames a freshly-acquired target must be seen for before anything
        # is published for it. A single detection at the frame edge is
        # often noise, and acting on it makes the car lurch into a hard
        # turn the moment someone reappears.
        self.declare_parameter("confirm_frames", 5)

        self.conf = self.get_parameter("conf").value
        self.imgsz = self.get_parameter("imgsz").value
        self.show = self.get_parameter("show").value
        self.reacquire_secs = self.get_parameter("reacquire_secs").value
        self.smoothing = self.get_parameter("smoothing").value
        self.relock_max_shift = self.get_parameter("relock_max_shift").value
        self.confirm_frames = self.get_parameter("confirm_frames").value

        device = self.get_parameter("device").value
        if not device:
            import torch
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.device = device

        model_name = self.get_parameter("model").value
        self.get_logger().info(f"Loading {model_name} on {self.device} ...")
        self.model = YOLO(model_name)
        self.get_logger().info("Model ready")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("info_topic").value
        self.create_subscription(CompressedImage, image_topic, self.on_frame, qos)
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos)
        self.target_pub = self.create_publisher(Point, TARGET_TOPIC, 10)

        self.fx = None
        self.cx_cam = None
        self.target_id = None
        self.last_seen = None      # rclpy Time the target was last matched
        self.last_cx = None        # where the target was, for sane re-locks
        self.smooth_bearing = None
        self.smooth_box_h = None
        self.confirmed = 0         # consecutive frames the target has held
        self.frames = 0
        self.last_report = self.get_clock().now()
        self._fps = 0.0
        self._mark = 0

        self.get_logger().info(f"Listening on {image_topic}")

    def on_info(self, msg):
        if self.fx is None and msg.k[0] > 0:
            self.fx = msg.k[0]
            self.cx_cam = msg.k[2]
            self.get_logger().info(
                f"Camera intrinsics: fx={self.fx:.1f} cx={self.cx_cam:.1f}"
            )

    def on_frame(self, msg):
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            self.get_logger().warn("Dropped a frame that failed to decode")
            return

        self.frames += 1
        h, w = frame.shape[:2]

        # persist=True is what carries tracker state between calls; without
        # it every frame starts a fresh tracker and ids never stabilise.
        result = self.model.track(
            frame,
            classes=[PERSON_CLASS],
            conf=self.conf,
            imgsz=self.imgsz,
            device=self.device,
            persist=True,
            tracker="bytetrack.yaml",
            verbose=False,
        )[0]

        people = self._extract(result)
        target = self._select(people, w)

        # Let a freshly acquired target settle before steering on it. The
        # smoothing filter is also warming up over these frames, so the
        # first value the controller sees is already filtered rather than
        # a raw edge-of-frame detection.
        if target is not None:
            if self.confirmed >= self.confirm_frames:
                self._publish(target, w)
            else:
                self._warm_up(target, w)

        if self.show:
            self._draw(frame, people, target, w, h)

    def _extract(self, result):
        people = []
        if result.boxes is None:
            return people
        for box in result.boxes:
            # A box can exist before the tracker assigns it an id; those are
            # drawn but are not lock candidates.
            track_id = int(box.id[0]) if box.id is not None else None
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            people.append({
                "id": track_id,
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "cx": (x1 + x2) / 2,
                "cy": (y1 + y2) / 2,
                "w": x2 - x1,
                "h": y2 - y1,
                "area": (x2 - x1) * (y2 - y1),
                "conf": float(box.conf[0]),
            })
        return people

    def _select(self, people, width):
        """Return the locked target this frame, or None if it is not visible."""
        now = self.get_clock().now()
        tracked = [p for p in people if p["id"] is not None]

        if self.target_id is not None:
            match = next((p for p in tracked if p["id"] == self.target_id), None)
            if match is not None:
                self.last_seen = now
                self.last_cx = match["cx"]
                self.confirmed += 1
                return match

            gone_for = (now - self.last_seen).nanoseconds / 1e9
            if gone_for < self.reacquire_secs:
                return None   # still locked, just occluded

            self.get_logger().info(
                f"Lost target id={self.target_id} after {gone_for:.1f}s; re-locking"
            )
            self.target_id = None
            # Bearing history belongs to the person who just left; keeping
            # it would smear their last position into the new target.
            self.smooth_bearing = None
            self.smooth_box_h = None

        if not tracked:
            return None

        # Prefer someone near where the target was standing. ByteTrack
        # renumbers a person who blinks out and returns, so the nearest
        # box is usually the same human under a new id -- whereas the
        # largest box in frame may be a bystander who just walked closer.
        candidates = tracked
        if self.last_cx is not None:
            near = [p for p in tracked
                    if abs(p["cx"] - self.last_cx) < self.relock_max_shift * width]
            if near:
                candidates = near
            chosen = min(candidates, key=lambda p: abs(p["cx"] - self.last_cx))
        else:
            chosen = max(candidates, key=lambda p: p["area"])

        self.target_id = chosen["id"]
        self.last_seen = now
        self.last_cx = chosen["cx"]
        self.confirmed = 1
        self.get_logger().info(f"Locked onto id={self.target_id}")
        return chosen

    def _bearing_of(self, person, width):
        cx_ref = self.cx_cam if self.cx_cam is not None else width / 2
        error_px = person["cx"] - cx_ref
        if self.fx is not None:
            return -math.atan2(error_px, self.fx)
        return -error_px / (width / 2)

    def _warm_up(self, person, width):
        """Advance the smoothing filter without publishing.

        Runs while a new target is still being confirmed, so by the time
        the controller sees a bearing the filter has already converged --
        instead of the first published value being a raw, unsmoothed
        detection that makes the car lurch.
        """
        self._blend(self._bearing_of(person, width), person["h"])

    def _blend(self, bearing, box_h):
        a = self.smoothing
        if self.smooth_bearing is None:
            self.smooth_bearing = bearing
            self.smooth_box_h = box_h
        else:
            self.smooth_bearing = a * self.smooth_bearing + (1 - a) * bearing
            self.smooth_box_h = a * self.smooth_box_h + (1 - a) * box_h

    def _publish(self, person, width):
        bearing = self._bearing_of(person, width)

        # Smooth both signals. The detector redraws the box every frame and
        # its edges move even when the person does not, which the car would
        # otherwise chase as real motion.
        self._blend(bearing, person["h"])

        msg = Point()
        msg.x = float(self.smooth_bearing)
        # Stage 4 has no depth sensor, so distance is carried as the box
        # height in pixels, negated. The sign is the marker: a negative y
        # means "pixels, invert me", a positive y means real metres from
        # Stage 5. Downstream can then handle either without a flag.
        msg.y = -float(self.smooth_box_h)
        msg.z = float(person["conf"])
        self.target_pub.publish(msg)

    def _draw(self, frame, people, target, w, h):
        cv2.line(frame, (w // 2, 0), (w // 2, h), (120, 120, 120), 1)

        for p in people:
            locked = target is not None and p["id"] == target["id"]
            colour = (0, 255, 0) if locked else (0, 160, 255)
            x1, y1 = int(p["x1"]), int(p["y1"])
            x2, y2 = int(p["x2"]), int(p["y2"])

            cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 3 if locked else 1)
            label = f"id={p['id']}" if p["id"] is not None else "id=?"
            if locked:
                label += "  TARGET"
            cv2.putText(
                frame, f"{label} {p['conf']:.2f}", (x1, max(y1 - 8, 14)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2,
            )
            if locked:
                cv2.circle(frame, (int(p["cx"]), int(p["cy"])), 5, colour, -1)
                cv2.putText(
                    frame, f"err={p['cx'] - w / 2:+.0f}px",
                    (x1, min(y2 + 20, h - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1,
                )

        if target is not None and self.confirmed < self.confirm_frames:
            cv2.putText(
                frame,
                f"CONFIRMING {self.confirmed}/{self.confirm_frames}", (10, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2,
            )
        elif target is None and self.target_id is not None:
            cv2.putText(
                frame, f"TARGET {self.target_id} OCCLUDED", (10, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
            )

        cv2.putText(
            frame,
            f"{self._tick():.1f} FPS  people={len(people)}  target={self.target_id}",
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
        )

        cv2.imshow(WINDOW, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            raise SystemExit
        if key == ord("t"):
            self.get_logger().info("Manual re-lock requested")
            self.target_id = None

    def _tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_report).nanoseconds / 1e9
        if elapsed >= 1.0:
            self._fps = (self.frames - self._mark) / elapsed
            self.last_report = now
            self._mark = self.frames
        return self._fps


def main():
    rclpy.init()
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
