#!/usr/bin/env python3
"""Show the Conduit camera stream from ROS 2.

Exit codes (used by main.py):
    0  stream worked; viewer closed with Q, Esc, or the window's X
    1  no frames received, or the stream stopped
"""

import sys
import time

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import CompressedImage
except ImportError:
    # ROS 2 is not installed or not sourced in this terminal.
    sys.exit(1)


TOPIC = '/conduit/camera/front/image_raw/compressed'
WINDOW = 'Conduit camera | Q to quit'

# Give up if no frame arrives for this many seconds.
FRAME_TIMEOUT_S = 5.0


class CameraViewer(Node):

    def __init__(self):
        super().__init__('check_conduit_camera')

        # Camera publishers commonly use sensor-data (best-effort) QoS
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            CompressedImage, TOPIC, self.image_callback, qos
        )

        self.frame_count = 0
        self.last_frame_time = time.monotonic()

        self.get_logger().info(f'Waiting for camera frames on {TOPIC}...')

    def image_callback(self, msg):
        image_data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warning('Could not decode image')
            return

        self.frame_count += 1
        self.last_frame_time = time.monotonic()

        cv2.imshow(WINDOW, frame)

        if self.frame_count == 1:
            self.get_logger().info(
                f'Camera stream received: {frame.shape[1]}x{frame.shape[0]}'
            )


def run():
    rclpy.init()
    node = CameraViewer()
    exit_code = 0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            # No frames at all, or the stream stopped.
            if time.monotonic() - node.last_frame_time > FRAME_TIMEOUT_S:
                exit_code = 1
                break

            if node.frame_count > 0:
                key = cv2.waitKey(1) & 0xFF

                if key in (ord('q'), 27):
                    break

                # The window's X button was clicked.
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(run())
