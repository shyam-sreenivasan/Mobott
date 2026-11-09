
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        # Camera topic
        self.topic = '/conduit/camera/front/image_raw/compressed'

        # Camera publishers commonly use sensor-data (best-effort) QoS
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribe to compressed camera images
        self.subscription = self.create_subscription(
            CompressedImage,
            self.topic,
            self.image_callback,
            qos
        )

        self.frame_count = 0

        self.get_logger().info('Waiting for camera frames...')

    def image_callback(self, msg):

        # Convert compressed image bytes into a NumPy array
        image_data = np.frombuffer(msg.data, dtype=np.uint8)

        # Decode JPEG image
        frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warning('Could not decode image')
            return

        self.frame_count += 1

        # Display image
        cv2.imshow('iPhone Camera - ROS 2', frame)
        cv2.waitKey(1)

        if self.frame_count == 1:
            self.get_logger().info(
                f'Camera stream received: {frame.shape[1]}x{frame.shape[0]}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = CameraViewer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

