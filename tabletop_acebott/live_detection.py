import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class AprilTagViewer(Node):

    def __init__(self):
        super().__init__('apriltag_viewer')

        self.topic = '/conduit/camera/front/image_raw/compressed'

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            self.topic,
            self.image_callback,
            qos
        )

        # Use the SAME dictionary as robot_localize.py
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            self.parameters
        )

        self.get_logger().info('ArUco detector started')

    def image_callback(self, msg):

        image_data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

        if frame is None:
            return

        # Match the detection method in robot_localize.py
        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is not None:

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):

                points = corners[i][0]

                center_x = int(np.mean(points[:, 0]))
                center_y = int(np.mean(points[:, 1]))

                cv2.circle(
                    frame,
                    (center_x, center_y),
                    5,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    frame,
                    f'ID: {marker_id}',
                    (center_x + 10, center_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

        else:
            cv2.putText(
                frame,
                'MARKER NOT DETECTED',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        cv2.imshow('Live Robot Marker Detection', frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            cv2.imwrite('live_frame.jpg', frame)
            self.get_logger().info('Saved live_frame.jpg')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    node = AprilTagViewer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()