"""Stage 1: detect people in the Conduit stream and draw boxes.

Detection only -- no tracking, no identity, no motion commands. If two
people are in frame this node will happily report both, and which one is
"first" can change frame to frame. Fixing that is Stage 2.

Alongside the debug window it publishes the strongest detection to
/target_person, so the downstream stages have the interface they expect
from the start. Bearing is a real angle when camera_info has arrived;
distance stays NaN until Stage 5 supplies depth.

Run with:  ros2 run person_follower person_detector
Press Q or ESC in the window to quit.
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
WINDOW = "Stage 1 - person detection"

PERSON_CLASS = 0  # COCO class id for "person"


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")

        # Topics are overridable: the driver's native names differ from the
        # namespaced ones recorded in the rosbag, so which one is live
        # depends on how Conduit was launched.
        self.declare_parameter("image_topic", IMAGE_TOPIC)
        self.declare_parameter("info_topic", INFO_TOPIC)
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.5)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")  # "" -> auto (cuda if present)
        self.declare_parameter("show", True)

        self.conf = self.get_parameter("conf").value
        self.imgsz = self.get_parameter("imgsz").value
        self.show = self.get_parameter("show").value

        device = self.get_parameter("device").value
        if not device:
            import torch
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.device = device

        model_name = self.get_parameter("model").value
        self.get_logger().info(f"Loading {model_name} on {self.device} ...")
        self.model = YOLO(model_name)
        # One warm-up pass: the first inference pays CUDA context and cuDNN
        # autotune costs that would otherwise show up as a multi-second
        # stall on the first real frame.
        self.model.predict(
            np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8),
            device=self.device, verbose=False,
        )
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

        self.fx = None      # focal length, filled in from camera_info
        self.cx_cam = None
        self.frames = 0
        self.last_report = self.get_clock().now()
        self._fps = 0.0
        self._mark = 0

        self.get_logger().info(f"Listening on {image_topic}")

    def on_info(self, msg):
        # Intrinsics are static; one message is enough.
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

        result = self.model.predict(
            frame,
            classes=[PERSON_CLASS],
            conf=self.conf,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )[0]

        people = self._extract(result)
        if people:
            self._publish(max(people, key=lambda p: p["conf"]), w)

        if self.show:
            self._draw(frame, people, w, h)

    def _extract(self, result):
        people = []
        for box in result.boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            people.append({
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "cx": (x1 + x2) / 2,
                "cy": (y1 + y2) / 2,
                "w": x2 - x1,
                "h": y2 - y1,
                "conf": float(box.conf[0]),
            })
        return people

    def _publish(self, person, width):
        """Publish as (bearing, distance, confidence).

        Bearing is radians, positive when the person is to the LEFT --
        matching the sign convention of angular.z, so Stage 3 can use it
        without flipping anything.
        """
        cx_ref = self.cx_cam if self.cx_cam is not None else width / 2
        error_px = person["cx"] - cx_ref

        if self.fx is not None:
            bearing = -math.atan2(error_px, self.fx)
        else:
            # No intrinsics yet: fall back to a normalised error in [-1, 1].
            bearing = -error_px / (width / 2)

        msg = Point()
        msg.x = float(bearing)
        msg.y = float("nan")   # distance: Stage 5
        msg.z = float(person["conf"])
        self.target_pub.publish(msg)

    def _draw(self, frame, people, w, h):
        cv2.line(frame, (w // 2, 0), (w // 2, h), (120, 120, 120), 1)

        best = max(people, key=lambda p: p["conf"]) if people else None
        for p in people:
            primary = p is best
            colour = (0, 255, 0) if primary else (0, 160, 255)
            x1, y1 = int(p["x1"]), int(p["y1"])
            x2, y2 = int(p["x2"]), int(p["y2"])

            cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2 if primary else 1)
            cv2.circle(frame, (int(p["cx"]), int(p["cy"])), 4, colour, -1)
            cv2.putText(
                frame, f"person {p['conf']:.2f}", (x1, max(y1 - 8, 14)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2,
            )
            if primary:
                # The three measurements Stage 1 is meant to establish.
                cv2.putText(
                    frame,
                    f"cx={p['cx']:.0f} err={p['cx'] - w / 2:+.0f} "
                    f"box={p['w']:.0f}x{p['h']:.0f}",
                    (x1, min(y2 + 20, h - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1,
                )

        cv2.putText(
            frame, f"{self._tick():.1f} FPS  people={len(people)}  {self.device}",
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
        )

        cv2.imshow(WINDOW, frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            raise SystemExit

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
    node = PersonDetector()
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
