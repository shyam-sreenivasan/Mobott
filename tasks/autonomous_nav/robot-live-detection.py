#!/usr/bin/env python3
"""Live robot localization from the Conduit camera feed."""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


TOPIC = '/conduit/camera/front/image_raw/compressed'
WINDOW = 'Live localization | Q to quit'
BEV_WINDOW = "Bird's-eye view | Q to quit"

PROJECT_DIR = Path(__file__).resolve().parent
CALIBRATION_DIR = PROJECT_DIR / 'calibration'
BEV_PIXELS_PER_CM = 10             # bird's-eye display only

FRAME_TIMEOUT_S = 5.0      # give up if no frame arrives this long

ROBOT_MARKER_ID = 0        # DICT_4X4_50 marker on the robot
ARROW_LENGTH = 1.5         # heading arrow length, in marker heights from the center


def load_calibration(calibration):
    """ Loads the .npy and .json files based on the arg parsed file  """
    path = Path(calibration)
    if path.suffix != '.npy':
        path = path.with_name(path.name + '.npy')
    if not path.exists() and not path.is_absolute():
        path = CALIBRATION_DIR / path.name

    json_path = path.with_suffix('.json')

    for file in (path, json_path):
        if not file.exists():
            print(f'Calibration file not found: {file}')
            return None

    H = np.load(path)
    metadata = json.loads(json_path.read_text())
    return path.stem, H, metadata


class LiveLocalization(Node):

    def __init__(self, calibration_name, H, metadata):
        super().__init__('robot_localize_live')

        # Calibration: camera pixels -> table cm
        self.calibration_name = calibration_name
        self.H = H
        self.metadata = metadata
        self.calibration_ok = None      # checked against the first frame

        # Display scaling for the bird's-eye view
        scale = BEV_PIXELS_PER_CM
        x_len = metadata['x_length_cm']
        y_len = metadata['y_length_cm']
        cm_to_image = np.array([
            [scale, 0.0,    0.0],
            [0.0,   -scale, y_len * scale],
            [0.0,   0.0,    1.0],
        ])
        self.bev_matrix = cm_to_image @ H
        self.cm_to_image = cm_to_image          # cm -> bird's-eye pixels (for drawing)
        self.H_inv = np.linalg.inv(H)           # table cm -> camera pixels (for drawing)
        self.bev_size = (int(round(x_len * scale)), int(round(y_len * scale)))
        self.x_len = x_len
        self.y_len = y_len

        # ArUco detector for the robot marker
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        
        # detects marker it different type based on opencv version
        
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            self.detect_markers = detector.detectMarkers
        else:
            self.detect_markers = lambda image: cv2.aruco.detectMarkers(
                image, dictionary, parameters=parameters
            )

        self.robot_marker = None        # latest detection, or None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                                  # only the newest frame
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(CompressedImage, TOPIC, self.on_image, qos)

        self.frame = None
        self.frame_count = 0
        self.last_frame_time = time.monotonic()

        self.get_logger().info(f'Listening to {TOPIC}')

    def detect_robot_marker(self, frame):
        """Find marker ID 0 and return it only if it is inside the workspace"""
        
        corners, ids, _ = self.detect_markers(frame)
        if ids is None:
            return None

        for quad, marker_id in zip(corners, ids.flatten()):
            if int(marker_id) != ROBOT_MARKER_ID:
                continue

            corners_px = np.asarray(quad, dtype=np.float32).reshape(4, 2)
            corners_cm = cv2.perspectiveTransform(
                corners_px.reshape(-1, 1, 2), self.H
            ).reshape(4, 2)
            cx, cy = corners_cm.mean(axis=0)

            # Heading: the direction the marker's top side faces, on the table.
            # 0 = toward D (+x), 90 = toward Y (+y), 180 = -x, 270 = -y
            top_mid = (corners_cm[0] + corners_cm[1]) / 2.0     # ArUco corners 0,1 = top edge
            dx, dy = top_mid[0] - cx, top_mid[1] - cy
            heading = float(np.degrees(np.arctan2(dy, dx)) % 360.0)
            if heading >= 360.0:        # tiny negative angles can round up to 360
                heading = 0.0

            inside = (0.0 <= cx <= self.x_len) and (0.0 <= cy <= self.y_len)	# checks if the marker is within the boundary
            if not inside:
                return None

            return {
                'corners_px': corners_px,	# 4x2 marker corners in the camera image
                'corners_cm': corners_cm,	# 4x2 marker corners on the table (cm)
                'center_cm': (float(cx), float(cy)),		# (x, y) marker center on the table (cm)
                'heading_deg': heading,                 # 0-359 degrees, see above
                'top_dir_cm': (float(dx), float(dy)),   # center -> top edge, on the table (cm)
            }

        return None

    def on_image(self, msg):
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )

        if frame is None:
            self.get_logger().warning('Could not decode image')
            return

        self.frame = frame
        self.frame_count += 1
        self.last_frame_time = time.monotonic()

        # The calibration only fits the camera resolution it was made with.
        if self.calibration_ok is None:
            expected = (self.metadata['image_width_px'],
                        self.metadata['image_height_px'])
            actual = (frame.shape[1], frame.shape[0])
            self.calibration_ok = (expected == actual)
            if not self.calibration_ok:
                print(f'Camera resolution {actual[0]}x{actual[1]} does not match '
                      f'{self.calibration_name} ({expected[0]}x{expected[1]}). '
                      'Recalibrate the workspace.')
                return
            self.get_logger().info(f'Using calibration {self.calibration_name}')

        if not self.calibration_ok:
            return

        bev = cv2.warpPerspective(frame, self.bev_matrix, self.bev_size)

        marker = self.detect_robot_marker(frame)
        if (marker is None) != (self.robot_marker is None):   # entered or left
            if marker is None:
                print(f'ID {ROBOT_MARKER_ID} left the workspace')
            else:
                print(f'ID {ROBOT_MARKER_ID} detected in the workspace')
        self.robot_marker = marker

        display = frame.copy()      # draw on a copy; self.frame stays clean
        if marker is not None:
            self.draw_marker_box(display, bev, marker)

        status = self.marker_status_text(marker)
        self.draw_status(display, status)
        self.draw_status(bev, status)

        cv2.imshow(WINDOW, display)
        cv2.imshow(BEV_WINDOW, bev)

    def marker_status_text(self, marker):
        """Text with the ID 0 center position in the workspace (cm)."""
        if marker is None:
            return f'ID {ROBOT_MARKER_ID} Marker = Not detected, x = -, y = -, angle = -'

        x, y = marker['center_cm']
        angle = marker['heading_deg']
        return (f'ID {ROBOT_MARKER_ID} Marker = Detected, x = {x:.2f} cm, '
                f'y = {y:.2f} cm, angle = {round(angle) % 360} deg')

    def draw_status(self, image, text):
        """Draw the text in a dark bar across the top of the image."""
        cv2.rectangle(image, (0, 0), (image.shape[1], 32), (0, 0, 0), -1)
        cv2.putText(image, text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 1, cv2.LINE_AA)

    def draw_marker_box(self, display, bev, marker):
        """Draw a bounding box around the detected ID 0 marker in both views."""
        color = (0, 255, 0)     # green

        # Camera view: the detected corners in pixels
        box_px = marker['corners_px'].astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(display, [box_px], True, color, 2)

        # Bird's-eye view: the corners in cm, converted to bird's-eye pixels
        box_bev = cv2.perspectiveTransform(
            marker['corners_cm'].reshape(-1, 1, 2), self.cm_to_image
        ).astype(np.int32)
        cv2.polylines(bev, [box_bev], True, color, 2)

        self.draw_heading_arrow(display, bev, marker)

    def draw_heading_arrow(self, display, bev, marker):
        """Draw an arrow from the ID 0 center in the direction it faces."""
        color = (0, 0, 255)     # red

        cx, cy = marker['center_cm']
        dx, dy = marker['top_dir_cm']               # half a marker height long
        scale = 2.0 * ARROW_LENGTH
        points_cm = np.array([[[cx, cy]],
                              [[cx + dx * scale, cy + dy * scale]]], dtype=np.float64)

        # Camera view: table cm -> camera pixels
        start, tip = cv2.perspectiveTransform(points_cm, self.H_inv).reshape(2, 2)
        cv2.arrowedLine(display, tuple(np.int32(start)), tuple(np.int32(tip)),
                        color, 2, tipLength=0.3)

        # Bird's-eye view: table cm -> bird's-eye pixels
        start, tip = cv2.perspectiveTransform(points_cm, self.cm_to_image).reshape(2, 2)
        cv2.arrowedLine(bev, tuple(np.int32(start)), tuple(np.int32(tip)),
                        color, 2, tipLength=0.3)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Live localization using a workspace calibration.'
    )
    parser.add_argument(
        '--calibration', required=True,
        help='Calibration to use, e.g. workspace_3 or calibration/workspace_3.npy'
    )
    return parser.parse_args()


def main():
    args = parse_args()

    calibration = load_calibration(args.calibration)
    if calibration is None:
        return 1

    rclpy.init()
    node = LiveLocalization(*calibration)
    exit_code = 0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            if node.calibration_ok is False:
                exit_code = 1
                break

            if time.monotonic() - node.last_frame_time > FRAME_TIMEOUT_S:
                print('Problem with the camera stream. '
                      'Check if the topics are publishing')
                exit_code = 1
                break

            if node.frame_count > 0:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                if (cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1 or
                        cv2.getWindowProperty(BEV_WINDOW, cv2.WND_PROP_VISIBLE) < 1):
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
    sys.exit(main())
