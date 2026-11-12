"""Display the raw Conduit camera stream. No detection -- this exists to
prove the video path works before anything is layered on top.

Conduit publishes CompressedImage (JPEG) only, so we decode with
cv2.imdecode rather than cv_bridge: it is fewer moving parts and avoids a
copy on every frame.

Run with:  ros2 run person_follower camera_viewer
Press Q or ESC in the window to quit.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

TOPIC = "/conduit/camera/front/image_raw/compressed"
WINDOW = "Conduit camera"


class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")

        self.declare_parameter("topic", TOPIC)
        # Conduit publishes 480x640 portrait frames even with the phone held
        # in landscape, so the image arrives on its side. Rotating here is
        # display-only: it does not widen the field of view.
        # One of 0, 90, 180, 270 (degrees, counter-clockwise).
        self.declare_parameter("rotate", 0)

        topic = self.get_parameter("topic").value
        degrees = self.get_parameter("rotate").value
        self.rotate = self._rotation_code(degrees)
        if degrees and self.rotate is None:
            self.get_logger().warn(
                f"rotate:={degrees} is not one of 0/90/180/270; not rotating"
            )

        # Camera frames are a live stream: BEST_EFFORT with depth 1 means we
        # always render the newest frame instead of working through a backlog
        # if display briefly falls behind the publisher.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CompressedImage, topic, self.on_frame, qos)

        self.frames = 0
        self.last_report = self.get_clock().now()

        self.get_logger().info(f"Listening on {topic}")
        self.get_logger().info("Waiting for frames... (Q or ESC to quit)")

    @staticmethod
    def _rotation_code(degrees):
        """Map a rotation in degrees to the cv2 constant, or None for 0."""
        return {
            0: None,
            90: cv2.ROTATE_90_COUNTERCLOCKWISE,
            180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_CLOCKWISE,
        }.get(int(degrees), None)

    def on_frame(self, msg):
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            self.get_logger().warn("Dropped a frame that failed to decode")
            return

        if self.rotate is not None:
            frame = cv2.rotate(frame, self.rotate)

        if self.frames == 0:
            h, w = frame.shape[:2]
            self.get_logger().info(f"First frame received: {w}x{h}")
        self.frames += 1

        fps = self._tick()
        h, w = frame.shape[:2]
        cv2.putText(
            frame, f"{w}x{h}  {fps:.1f} FPS  n={self.frames}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
        )

        cv2.imshow(WINDOW, frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            raise SystemExit

    def _tick(self):
        """Rolling FPS, recomputed once a second."""
        now = self.get_clock().now()
        elapsed = (now - self.last_report).nanoseconds / 1e9
        if elapsed >= 1.0:
            self._fps = self.frames_since_report() / elapsed
            self.last_report = now
            self._mark = self.frames
        return getattr(self, "_fps", 0.0)

    def frames_since_report(self):
        return self.frames - getattr(self, "_mark", 0)


def main():
    rclpy.init()
    node = CameraViewer()
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
