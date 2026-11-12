#!/usr/bin/env python3
"""Calibrate the tabletop from the live Conduit camera feed."""

import json
import re
import sys
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


TOPIC = '/conduit/camera/front/image_raw/compressed'

PROJECT_DIR = Path(__file__).resolve().parent.parent   # this file is in setup/
CONFIG_PATH = PROJECT_DIR / 'setup' / 'config.json'
CALIBRATION_DIR = PROJECT_DIR / 'calibration'
CALIBRATION_IMAGE_DIR = CALIBRATION_DIR / 'images'
BEV_PIXELS_PER_CM = 10            # bird's-eye reference image only
CALIBRATION_NAME = 'workspace'     # saves workspace_1, workspace_2, ...

CAPTURE_SECONDS = 2.0      # how long to collect frames
FRAME_TIMEOUT_S = 5.0      # give up if no frame arrives this long

# Workspace corner markers (DICT_4X4_50): marker ID -> corner label
CORNER_MARKERS = {1: 'X', 2: 'Y', 3: 'U', 4: 'D'}


def sharpness(frame):
    """Higher value = sharper image (variance of the Laplacian)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var()


class SharpFrameGrabber(Node):

    def __init__(self):
        super().__init__('calibrate_frame_grabber')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(CompressedImage, TOPIC, self.on_image, qos)

        self.best_frame = None
        self.best_score = -1.0
        self.frame_count = 0
        self.first_frame_time = None
        self.last_frame_time = time.monotonic()

    def on_image(self, msg):
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )

        if frame is None:
            return

        now = time.monotonic()
        if self.first_frame_time is None:
            self.first_frame_time = now
        self.last_frame_time = now
        self.frame_count += 1

        score = sharpness(frame)
        if score > self.best_score:
            self.best_score = score
            self.best_frame = frame


def capture_sharp_frame():
    """Collect live frames for CAPTURE_SECONDS and return the sharpest one."""

    rclpy.init()
    node = SharpFrameGrabber()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()

            if now - node.last_frame_time > FRAME_TIMEOUT_S:
                raise RuntimeError(
                    'Problem with the camera stream. '
                    'Check if the topics are publishing'
                )

            if (node.first_frame_time is not None
                    and now - node.first_frame_time >= CAPTURE_SECONDS):
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return node.best_frame


def detect_corner_markers(frame):
    """Find the DICT_4X4_50 corner markers and label them X, Y, U, D.

    Returns {label: {'id': ..., 'corners': 4x2 array, 'center': (x, y)}}
    for every corner marker found. Missing markers are left out.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    if hasattr(cv2.aruco, 'ArucoDetector'):   # OpenCV 4.7+
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(frame)
    else:                                      # older OpenCV
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, dictionary, parameters=parameters
        )

    found = {}
    if ids is None:
        return found

    for quad, marker_id in zip(corners, ids.flatten()):
        label = CORNER_MARKERS.get(int(marker_id))
        if label is None:
            continue

        quad = np.asarray(quad, dtype=np.float32).reshape(4, 2)
        center = quad.mean(axis=0)

        found[label] = {
            'id': int(marker_id),
            'corners': quad,
            'center': (float(center[0]), float(center[1])),
        }

    return found


def compute_homography(markers, x_length_cm, y_length_cm,
                       marker_width_cm, marker_height_cm):
    """Homography from image pixels to table cm.

    The workspace corners are the outer corners of the markers:
        X: left by width/2, down by height/2  -> (0, 0)
        Y: left by width/2, up by height/2    -> (0, y_length)
        U: right by width/2, up by height/2   -> (x_length, y_length)
        D: right by width/2, down by height/2 -> (x_length, 0)

    So each marker's center sits half a marker inside its workspace corner.
    The detected centers are matched to those inner positions, which puts
    the outer corners exactly at the workspace corners.
    """
    order = ['X', 'Y', 'U', 'D']
    hw = marker_width_cm / 2.0
    hh = marker_height_cm / 2.0

    src = np.array([markers[label]['center'] for label in order], dtype=np.float32)
    dst = np.array([
        [hw,               hh],                # X center
        [hw,               y_length_cm - hh],  # Y center
        [x_length_cm - hw, y_length_cm - hh],  # U center
        [x_length_cm - hw, hh],                # D center
    ], dtype=np.float32)

    return cv2.getPerspectiveTransform(src, dst)


def save_bev_image(H, frame, config, name):
    """Save a bird's-eye view of the workspace for reference (not used by code).

    H maps pixels -> cm, so the image is scaled x10 (10 pixels per cm) and
    flipped vertically so that Y (y up) appears at the top of the picture.
    """
    scale = BEV_PIXELS_PER_CM
    width_px = int(round(config['x_length_cm'] * scale))
    height_px = int(round(config['y_length_cm'] * scale))

    cm_to_image = np.array([
        [scale, 0.0,    0.0],
        [0.0,   -scale, config['y_length_cm'] * scale],
        [0.0,   0.0,    1.0],
    ])

    bev = cv2.warpPerspective(frame, cm_to_image @ H, (width_px, height_px))

    CALIBRATION_IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    bev_path = CALIBRATION_IMAGE_DIR / f'{name}_bev.png'
    cv2.imwrite(str(bev_path), bev)
    print(f'Saved: {bev_path}')


def next_calibration_name():
    """Return workspace_{i} with i one higher than any existing calibration."""
    pattern = re.compile(rf'^{CALIBRATION_NAME}_(\d+)\.(npy|json)$')
    numbers = [
        int(match.group(1))
        for path in CALIBRATION_DIR.iterdir()
        if (match := pattern.match(path.name))
    ]
    return f'{CALIBRATION_NAME}_{max(numbers, default=0) + 1}'


def save_calibration(H, frame, markers, config):
    """Save the homography (.npy) and its details (.json) in calibration/."""
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)

    name = next_calibration_name()
    npy_path = CALIBRATION_DIR / f'{name}.npy'
    json_path = CALIBRATION_DIR / f'{name}.json'

    np.save(npy_path, H)

    metadata = {
        'mapping': 'camera pixels -> table cm',
        'frame': 'X=(0,0), Y=(0,y_length), U=(x_length,y_length), D=(x_length,0)',
        'x_length_cm': config['x_length_cm'],
        'y_length_cm': config['y_length_cm'],
        'marker_width_cm': config['marker_width_cm'],
        'marker_height_cm': config['marker_height_cm'],
        'image_width_px': int(frame.shape[1]),
        'image_height_px': int(frame.shape[0]),
        'camera_topic': TOPIC,
        'marker_centers_px': {
            label: list(markers[label]['center']) for label in ['X', 'Y', 'U', 'D']
        },
        'created': datetime.now().isoformat(timespec='seconds'),
    }

    json_path.write_text(json.dumps(metadata, indent=2))

    print(f'Saved: {npy_path}')
    print(f'Saved: {json_path}')

    save_bev_image(H, frame, config, name)


def load_config():
    """Read the calibration inputs from config.json in the project folder."""
    required = ['x_length_cm', 'y_length_cm', 'marker_width_cm', 'marker_height_cm']

    if not CONFIG_PATH.exists():
        sys.exit(f'Config file not found: {CONFIG_PATH}')

    config = json.loads(CONFIG_PATH.read_text())

    missing = [key for key in required if key not in config]
    if missing:
        sys.exit(f'Missing in {CONFIG_PATH.name}: {", ".join(missing)}')

    return {key: float(config[key]) for key in required}


if __name__ == '__main__':
    config = load_config()

    frame = capture_sharp_frame()
    print(f'Captured sharp frame: {frame.shape[1]}x{frame.shape[0]}')

    markers = detect_corner_markers(frame)

    missing = [label for label in CORNER_MARKERS.values() if label not in markers]
    for label in missing:
        print(f'The marker {label} is not detected so unable to calibrate')
    if missing:
        sys.exit(1)

    H = compute_homography(
        markers,
        config['x_length_cm'], config['y_length_cm'],
        config['marker_width_cm'], config['marker_height_cm']
    )
    print('Homography (pixels -> cm):')
    print(H)

    save_calibration(H, frame, markers, config)
