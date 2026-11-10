
#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from collections import deque

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)

from sensor_msgs.msg import CompressedImage

# Reuse the functions from your existing localization project.
import robot_localize as localization


class LiveRobotLocalization(Node):

    def __init__(self, args):

        super().__init__('robot_localize_live')

        self.args = args
        self.topic = args.topic

        # Load calibration on the first received frame.
        self.H = None
        self.metadata = None

        # Reuse the existing DICT_4X4_50 detector.
        self.detector = localization.create_detector("4X4_50")

        # Store recent robot positions for the trajectory map.
        self.trail = deque(maxlen=1000)

        self.frame_count = 0
        self.last_pose = None

        # ROS 2 camera QoS.
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

        self.get_logger().info(
            f'Live localization started. Listening to {self.topic}'
        )

        self.get_logger().info(
            f'Target marker ID: {args.marker_id}'
        )

    def load_live_calibration(self, frame):

        path = Path(self.args.tag_homography)

        if not path.exists():
            raise FileNotFoundError(
                f'Calibration not found: {path}'
            )

        metadata_path = path.with_suffix('.json')

        if not metadata_path.exists():
            raise FileNotFoundError(
                f'Calibration metadata not found: {metadata_path}'
            )

        H = np.load(str(path), allow_pickle=False)
        metadata = json.loads(metadata_path.read_text())

        if H.shape != (3, 3):
            raise RuntimeError('Invalid homography dimensions')

        if metadata.get('coordinate_frame_version') != (
            localization.FRAME_VERSION
        ):
            raise RuntimeError(
                'Calibration uses an outdated coordinate system.'
            )

        # Verify the calibration plane height.
        plane_height = metadata.get('plane_height_cm')

        if plane_height is None or abs(
            float(plane_height) - self.args.tag_plane_height_cm
        ) > 0.5:

            raise RuntimeError(
                f'Expected tag-plane calibration at '
                f'{self.args.tag_plane_height_cm:.1f} cm. '
                f'Calibration metadata says {plane_height!r}.'
            )

        # Compare calibration resolution with live camera resolution.
        calibration_width = metadata['image_width_px']
        calibration_height = metadata['image_height_px']

        live_height, live_width = frame.shape[:2]

        calibration_ratio = calibration_width / calibration_height
        live_ratio = live_width / live_height

        if abs(calibration_ratio - live_ratio) > 0.01:

            raise RuntimeError(
                'Live camera aspect ratio differs from calibration. '
                f'Calibration: {calibration_width}x{calibration_height}, '
                f'Live: {live_width}x{live_height}. '
                'Use matching camera orientation and aspect ratio.'
            )

        # Adjust the homography for a resolution change.
        #
        # Original calibration coordinates:
        # x_original = x_live * calibration_width / live_width
        # y_original = y_live * calibration_height / live_height

        scale = np.array([
            [calibration_width / live_width, 0, 0],
            [0, calibration_height / live_height, 0],
            [0, 0, 1]
        ], dtype=np.float64)

        H_live = H @ scale

        self.H = H_live
        self.metadata = metadata

        self.get_logger().info(
            f'Calibration loaded: {path}'
        )

        self.get_logger().info(
            f'Calibration resolution: '
            f'{calibration_width}x{calibration_height}'
        )

        self.get_logger().info(
            f'Live resolution: {live_width}x{live_height}'
        )

        self.get_logger().info(
            f'Tag plane height: {plane_height} cm'
        )

    def image_callback(self, msg):

        # Decode compressed ROS image.
        image_data = np.frombuffer(
            msg.data,
            dtype=np.uint8
        )

        frame = cv2.imdecode(
            image_data,
            cv2.IMREAD_COLOR
        )

        if frame is None:
            return

        frame = cv2.rotate(
        frame,
        cv2.ROTATE_90_COUNTERCLOCKWISE
    )

        # Load calibration once.
        if self.H is None:

            try:
                self.load_live_calibration(frame)

            except Exception as error:
                self.get_logger().error(
                    f'Calibration error: {error}'
                )
                rclpy.shutdown()
                return

        self.frame_count += 1

        # --------------------------------------------------
        # 1. DETECT MARKER
        # --------------------------------------------------

        corners = localization.detect_marker(
            frame,
            self.detector,
            self.args.marker_id
        )

        overlay = frame.copy()
        robot_map_pose = None

        if corners is not None:

            # --------------------------------------------------
            # 2. ESTIMATE TAG POSITION AND HEADING
            # --------------------------------------------------

            tag_pose = localization.estimate_pose(
                corners,
                self.H,
                self.metadata,
                self.args.heading_offset_deg
            )

            tag_x, tag_y, heading = tag_pose

            # --------------------------------------------------
            # 3. ESTIMATE ROBOT GEOMETRIC CENTER
            # --------------------------------------------------

            robot_x, robot_y = (
                localization.robot_center_from_tag(
                    tag_pose,
                    self.args.tag_forward_offset_cm
                )
            )

            self.last_pose = (
                robot_x,
                robot_y,
                heading
            )

            self.trail.append((robot_x, robot_y))

            robot_map_pose = (
                robot_x,
                robot_y,
                heading
            )

            # --------------------------------------------------
            # 4. DRAW DETECTION AND POSE
            # --------------------------------------------------

            localization.annotate_marker(
                overlay,
                corners,
                tag_pose,
                self.args.marker_id,
                (robot_x, robot_y)
            )

            # Display detailed coordinates.
            info = (
                f'Robot: X={robot_x:.1f} cm '
                f'Y={robot_y:.1f} cm '
                f'Theta={heading:.1f} deg'
            )

            cv2.putText(
                overlay,
                info,
                (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

            # Avoid printing at every camera frame.
            if self.frame_count % 30 == 0:

                self.get_logger().info(
                    f'Tag: ({tag_x:.2f}, {tag_y:.2f}) cm | '
                    f'Robot: ({robot_x:.2f}, {robot_y:.2f}) cm | '
                    f'Heading: {heading:.2f} deg'
                )

        else:

            cv2.putText(
                overlay,
                'MARKER NOT DETECTED',
                (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        # --------------------------------------------------
        # 5. DISPLAY LIVE CAMERA
        # --------------------------------------------------

        cv2.imshow(
            'Live Robot Localization',
            overlay
        )

        # --------------------------------------------------
        # 6. DISPLAY TABLETOP MAP
        # --------------------------------------------------

        if not self.args.no_map:

            map_image = localization.draw_map(
                self.metadata,
                self.trail,
                robot_map_pose
            )

            cv2.imshow(
                'Live Tabletop Map',
                map_image
            )

        # Press Q to stop.
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):

        cv2.destroyAllWindows()
        super().destroy_node()


def main():

    parser = argparse.ArgumentParser(
        description='Real-time robot localization using ROS 2 and OpenCV.'
    )

    parser.add_argument(
        '--tag-homography',
        type=Path,
        required=True,
        help='Path to elevated tag-plane homography (.npy)'
    )

    parser.add_argument(
        '--topic',
        default='/conduit/camera/front/image_raw/compressed'
    )

    parser.add_argument(
        '--marker-id',
        type=int,
        default=0
    )

    parser.add_argument(
        '--tag-plane-height-cm',
        type=float,
        default=11.0
    )

    parser.add_argument(
        '--tag-forward-offset-cm',
        type=float,
        default=4.5
    )

    parser.add_argument(
        '--heading-offset-deg',
        type=float,
        default=0.0
    )

    parser.add_argument(
        '--no-map',
        action='store_true',
        help='Disable the tabletop map window'
    )

    args = parser.parse_args()

    rclpy.init()

    node = LiveRobotLocalization(args)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()

