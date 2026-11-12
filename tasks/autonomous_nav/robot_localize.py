
#!/usr/bin/env python3
"""Tabletop localization and ArUco tracking.

Coordinate convention:

    M (0,74) -------- Y (90,74)
       |                  |
       |                  |
    X (0,0) --------- F (90,0)

+X: X -> F
+Y: X -> M
Heading 0 degrees: X -> F
Positive heading: counterclockwise

Load a named homography created by calibrate.py. For a robot-mounted,
upward-facing marker, use a calibration made at the marker plane height
and pass it with --tag-homography.

Step 1:
Calibrate the ground for empty ground
python calibrate.py empty.png --name empty

Step 2: Take the image of the marker in the elevated plane
Calibrate the elevated ArUco marker plane
python calibrate.py robot.png --name robot_marker --plane-height-cm 11

Step 3:
Estimate the robot pose
python robot_localize.py pose robot.png --tag-homography calibration/empty.npy

Step 4: Track the robot in a video
python robot_localize.py track robot_movement.mp4 --tag-homography calibration/empty.npy --display --show-map --save-video

"""

import argparse
import csv
import json
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np


WIDTH_CM = 90.0
HEIGHT_CM = 74.0
PIXELS_PER_CM = 10.0

PROJECT_DIR = Path(__file__).resolve().parent
DATA_DIR = PROJECT_DIR / "data"
VIDEO_DIR = DATA_DIR / "videos"
IMAGE_DIR = DATA_DIR / "images"
CALIBRATION_DIR = PROJECT_DIR / "calibration"
RESULTS_DIR = PROJECT_DIR / "results"

DEFAULT_H = CALIBRATION_DIR / "tabletop.npy"
FRAME_VERSION = "X_BOTTOM_LEFT_X_RIGHT_Y_UP_V1"


def resolve_input_path(value, default_folder=None):
    """Resolve a supplied filename relative to a project data folder."""
    path = Path(value)

    if path.is_absolute() or path.exists() or default_folder is None:
        return path

    return default_folder / path


def read_image(path):
    image = cv2.imread(str(path))

    if image is None:
        raise RuntimeError(f"Cannot read image: {path}")

    return image


def save_image(path, image):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"Could not save: {path}")

    print(f"Saved: {path}")


def add_grid(image, width_cm, height_cm):
    h, w = image.shape[:2]

    for x in range(0, int(width_cm) + 1, 10):
        px = round(x / width_cm * (w - 1))
        cv2.line(image, (px, 0), (px, h - 1),
                 (100, 180, 100), 1)

    for y in range(0, int(height_cm) + 1, 10):
        py = round((1 - y / height_cm) * (h - 1))
        cv2.line(image, (0, py), (w - 1, py),
                 (100, 180, 100), 1)

    return image


# ------------------------------------------------------------
# 2. MANUAL POINT SELECTION
# ------------------------------------------------------------

def select_points(image, labels):
    scale = min(
        1.0,
        1200 / image.shape[1],
        850 / image.shape[0]
    )

    preview = cv2.resize(
        image,
        None,
        fx=scale,
        fy=scale
    )

    selected = []
    window = "Select points | Enter=confirm U=undo R=reset Q=quit"

    def mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(selected) < len(labels):
                selected.append((x / scale, y / scale))

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, mouse)

    while True:
        display = preview.copy()

        for i, point in enumerate(selected):
            px = int(point[0] * scale)
            py = int(point[1] * scale)

            cv2.circle(
                display, (px, py), 6,
                (0, 0, 255), -1
            )

            cv2.putText(
                display, labels[i],
                (px + 10, max(20, py - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (255, 255, 255), 2
            )

        cv2.imshow(window, display)
        key = cv2.waitKey(20) & 0xFF

        if key in (ord("q"), 27):
            cv2.destroyAllWindows()
            return None

        if key == ord("u") and selected:
            selected.pop()

        if key == ord("r"):
            selected.clear()

        if key in (13, 10, 32):
            if len(selected) == len(labels):
                cv2.destroyAllWindows()
                return np.array(selected, dtype=np.float32)


# ------------------------------------------------------------
# 3. CALIBRATION LOADING AND COORDINATE TRANSFORMS
# ------------------------------------------------------------

def load_calibration(path, image):
    path = Path(path)

    if not path.exists():
        raise RuntimeError(
            f"Missing homography: {path}. Run calibrate.py first."
        )

    meta_path = path.with_suffix(".json")

    if not meta_path.exists():
        raise RuntimeError(
            f"Missing calibration JSON: {meta_path}. Recalibrate."
        )

    H = np.load(str(path), allow_pickle=False)
    metadata = json.loads(meta_path.read_text())

    if H.shape != (3, 3):
        raise RuntimeError("Invalid homography dimensions")

    if metadata.get("coordinate_frame_version") != FRAME_VERSION:
        raise RuntimeError("Outdated coordinate system. Recalibrate.")

    expected = (
        metadata["image_width_px"],
        metadata["image_height_px"]
    )

    actual = (image.shape[1], image.shape[0])

    if actual != expected:
        raise RuntimeError(
            f"Image resolution {actual} differs from "
            f"calibration {expected}. Use matching video settings."
        )

    return H, metadata


def transform_points(points, H, metadata):
    points = np.asarray(
        points, dtype=np.float32
    ).reshape(-1, 1, 2)

    transformed = cv2.perspectiveTransform(
        points, H
    ).reshape(-1, 2)

    x = (
        transformed[:, 0]
        / (metadata["bev_width_px"] - 1)
        * metadata["width_cm"]
    )

    y = (
        1.0
        - transformed[:, 1]
        / (metadata["bev_height_px"] - 1)
    ) * metadata["height_cm"]

    return np.column_stack((x, y))


def create_bev(image, H, metadata):
    bev = cv2.warpPerspective(
        image,
        H,
        (
            metadata["bev_width_px"],
            metadata["bev_height_px"]
        )
    )

    return add_grid(
        bev,
        metadata["width_cm"],
        metadata["height_cm"]
    )


# ------------------------------------------------------------
# 4. TEST KNOWN POSITIONS
# ------------------------------------------------------------

def test_position(args):
    image = read_image(resolve_input_path(args.image, IMAGE_DIR))
    H, meta = load_calibration(args.homography, image)

    print("Click the measured ground-plane reference point.")

    points = select_points(
        image, ["Reference point"]
    )

    if points is None:
        return

    x, y = transform_points(
        points, H, meta
    )[0]

    print(f"Estimated: x={x:.2f}, y={y:.2f} cm")

    result = {
        "estimated_x_cm": float(x),
        "estimated_y_cm": float(y)
    }

    if args.ground_truth:
        gx, gy = args.ground_truth
        error = np.hypot(x - gx, y - gy)

        print(f"Ground truth: ({gx:.2f}, {gy:.2f}) cm")
        print(f"Position error: {error:.2f} cm")

        result["gt_x_cm"] = gx
        result["gt_y_cm"] = gy
        result["error_cm"] = float(error)

    overlay = image.copy()
    p = tuple(np.round(points[0]).astype(int))

    cv2.circle(overlay, p, 6, (0, 0, 255), -1)
    cv2.putText(
        overlay, f"({x:.1f}, {y:.1f}) cm",
        (p[0] + 10, max(25, p[1] - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (255, 255, 255), 2
    )

    save_image(args.output, overlay)

    Path(args.output).with_suffix(".json").write_text(
        json.dumps(result, indent=2)
    )

    if args.bev_preview:
        bev = create_bev(image, H, meta)
        save_image(
            Path(args.output).with_name("test_bev.png"),
            bev
        )


# ------------------------------------------------------------
# 5. ARUCO DETECTION AND POSE
# ------------------------------------------------------------

def create_detector(dictionary_name):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "Install opencv-contrib-python for ArUco"
        )

    name = (
        dictionary_name
        if dictionary_name.startswith("DICT_")
        else "DICT_" + dictionary_name
    )

    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, name)
    )

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(
            dictionary,
            cv2.aruco.DetectorParameters()
        )
        return detector.detectMarkers

    return lambda image: cv2.aruco.detectMarkers(
        image, dictionary
    )


def detect_marker(image, detector, marker_id):
    corners, ids, _ = detector(image)

    if ids is None:
        return None

    for quad, detected_id in zip(
        corners, ids.flatten()
    ):
        if int(detected_id) == marker_id:
            return np.asarray(
                quad, dtype=np.float32
            ).reshape(4, 2)

    return None


def estimate_pose(corners, H, metadata, heading_offset=0):
    world_corners = transform_points(
        corners, H, metadata
    )

    center = world_corners.mean(axis=0)

    # ArUco corners: TL, TR, BR, BL.
    # Direction from center toward marker's top edge.
    front = (
        world_corners[0] + world_corners[1]
    ) / 2

    direction = front - center

    theta = (
        np.degrees(
            np.arctan2(direction[1], direction[0])
        ) + heading_offset
    ) % 360

    return (
        float(center[0]),
        float(center[1]),
        float(theta)
    )


def robot_center_from_tag(tag_pose, forward_offset_cm):
    """Convert tag center to robot geometric center.

    The tag is assumed to lie on the robot centerline and forward
    of the robot center. Heading is counterclockwise from +X.
    """
    tag_x, tag_y, theta_deg = tag_pose
    theta = np.radians(theta_deg)

    robot_x = tag_x - forward_offset_cm * np.cos(theta)
    robot_y = tag_y - forward_offset_cm * np.sin(theta)

    return float(robot_x), float(robot_y)


def get_pose_calibration(args, image):
    """Choose tabletop or elevated tag-plane calibration."""
    path = args.tag_homography or args.homography
    H, metadata = load_calibration(path, image)

    if args.tag_homography:
        height = metadata.get("plane_height_cm")

        if height is None or abs(
            float(height) - args.tag_plane_height_cm
        ) > 0.5:
            raise RuntimeError(
                f"Tag homography must be calibrated at "
                f"{args.tag_plane_height_cm:.1f} cm. "
                f"Calibration metadata says {height!r}."
            )

    return H, metadata


def annotate_marker(image, corners, pose, marker_id, robot_pose=None):
    x, y, theta = pose

    quad = np.round(corners).astype(np.int32)
    center = corners.mean(axis=0)
    front = (corners[0] + corners[1]) / 2

    direction = front - center
    direction /= max(np.linalg.norm(direction), 1e-6)

    tip = center + direction * 45

    cv2.polylines(
        image, [quad], True,
        (0, 255, 0), 2
    )

    cv2.circle(
        image,
        tuple(np.round(center).astype(int)),
        5, (0, 255, 255), -1
    )

    cv2.arrowedLine(
        image,
        tuple(np.round(center).astype(int)),
        tuple(np.round(tip).astype(int)),
        (0, 100, 255), 3,
        tipLength=0.25
    )

    label = (
        f"TAG ID {marker_id} | "
        f"x={x:.1f} y={y:.1f} "
        f"heading={theta:.1f}"
    )

    if robot_pose is not None:
        rx, ry = robot_pose
        label += f" | ROBOT x={rx:.1f} y={ry:.1f}"

    cv2.putText(
        image, label, (10, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7, (0, 0, 0), 4
    )

    cv2.putText(
        image, label, (10, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7, (255, 255, 255), 2
    )


def detect_pose_image(args):
    image = read_image(resolve_input_path(args.image, IMAGE_DIR))
    H, meta = get_pose_calibration(args, image)

    detector = create_detector(args.dictionary)
    corners = detect_marker(
        image, detector, args.marker_id
    )

    if corners is None:
        print("Marker not detected")
        return

    tag_pose = estimate_pose(
        corners, H, meta, args.heading_offset_deg
    )

    tag_x, tag_y, theta = tag_pose

    robot_x, robot_y = robot_center_from_tag(
        tag_pose, args.tag_forward_offset_cm
    )

    print(
        f"Tag:    x={tag_x:.2f} cm, y={tag_y:.2f} cm, "
        f"heading={theta:.2f} deg"
    )

    print(
        f"Robot:  x={robot_x:.2f} cm, y={robot_y:.2f} cm"
    )

    overlay = image.copy()

    annotate_marker(
        overlay, corners, tag_pose, args.marker_id,
        (robot_x, robot_y)
    )

    save_image(args.output, overlay)

    result = {
        "marker_id": args.marker_id,
        "tag_x_cm": tag_x,
        "tag_y_cm": tag_y,
        "robot_x_cm": robot_x,
        "robot_y_cm": robot_y,
        "heading_deg": theta,
        "tag_plane_height_cm": (
            args.tag_plane_height_cm if args.tag_homography else None
        ),
        "tag_forward_offset_cm": args.tag_forward_offset_cm
    }

    Path(args.output).with_suffix(".json").write_text(
        json.dumps(result, indent=2)
    )

    if args.bev_preview:
        bev = create_bev(image, H, meta)

        bev_corners = cv2.perspectiveTransform(
            corners.reshape(-1, 1, 2), H
        ).reshape(4, 2)

        annotate_marker(
            bev, bev_corners, tag_pose, args.marker_id,
            (robot_x, robot_y)
        )

        # Draw the estimated robot center in the BEV coordinate frame.
        px = int(round(
            robot_x / meta["width_cm"] * (meta["bev_width_px"] - 1)
        ))

        py = int(round(
            (1 - robot_y / meta["height_cm"])
            * (meta["bev_height_px"] - 1)
        ))

        cv2.circle(bev, (px, py), 8, (255, 0, 255), 2)

        cv2.putText(
            bev, "robot center", (px + 9, py - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2
        )

        save_image(
            Path(args.output).with_name("marker_bev.png"),
            bev
        )


# ------------------------------------------------------------
# 6. VIDEO TRACKING
# ------------------------------------------------------------

def draw_map(metadata, trail, pose):
    canvas = np.full(
        (480, 580, 3), 245, dtype=np.uint8
    )

    scale = min(
        480 / metadata["width_cm"],
        380 / metadata["height_cm"]
    )

    def pixel(x, y):
        return (
            round(50 + x * scale),
            round(420 - y * scale)
        )

    cv2.rectangle(
        canvas,
        pixel(0, 0),
        pixel(metadata["width_cm"], metadata["height_cm"]),
        (50, 50, 50), 2
    )

    for x in range(10, int(metadata["width_cm"]), 10):
        cv2.line(
            canvas, pixel(x, 0),
            pixel(x, metadata["height_cm"]),
            (210, 210, 210), 1
        )

    for y in range(10, int(metadata["height_cm"]), 10):
        cv2.line(
            canvas, pixel(0, y),
            pixel(metadata["width_cm"], y),
            (210, 210, 210), 1
        )

    if len(trail) > 1:
        pts = np.array([
            pixel(x, y) for x, y in trail
        ], dtype=np.int32)

        cv2.polylines(
            canvas, [pts], False,
            (230, 130, 50), 2
        )

    if pose is not None:
        x, y, theta = pose

        center = pixel(x, y)
        radians = np.radians(theta)

        tip = pixel(
            x + 7 * np.cos(radians),
            y + 7 * np.sin(radians)
        )

        cv2.circle(
            canvas, center, 5,
            (0, 180, 0), -1
        )

        cv2.arrowedLine(
            canvas, center, tip,
            (0, 80, 255), 3,
            tipLength=0.25
        )

    return canvas


def track_video(args):
    source = (
        int(args.source)
        if args.source.isdecimal()
        else str(resolve_input_path(args.source, VIDEO_DIR))
    )

    cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open: {source}")

    detector = create_detector(args.dictionary)

    fps = cap.get(cv2.CAP_PROP_FPS)

    if fps <= 0 or fps > 240:
        fps = 30.0

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    csv_path = output_dir / "trajectory.csv"

    writer = None
    trail = deque(maxlen=10000)
    frame_index = 0
    last_map = None
    start_time = time.monotonic()

    is_file = isinstance(source, str) and Path(source).is_file()

    try:
        with csv_path.open("w", newline="") as file:
            columns = [
                "frame", "time_s", "detected",
                "tag_x_cm", "tag_y_cm",
                "robot_x_cm", "robot_y_cm", "heading_deg"
            ]

            csv_writer = csv.DictWriter(
                file, fieldnames=columns
            )

            csv_writer.writeheader()

            while True:
                success, frame = cap.read()

                if not success:
                    break

                if frame_index == 0:
                    H, meta = get_pose_calibration(args, frame)

                    if args.save_video:
                        h, w = frame.shape[:2]

                        writer = cv2.VideoWriter(
                            str(output_dir / "annotated.mp4"),
                            cv2.VideoWriter_fourcc(*"mp4v"),
                            fps,
                            (w, h)
                        )

                        if not writer.isOpened():
                            raise RuntimeError(
                                "Cannot create output video"
                            )

                if is_file:
                    time_s = frame_index / fps
                else:
                    time_s = time.monotonic() - start_time

                corners = detect_marker(
                    frame, detector, args.marker_id
                )

                overlay = frame.copy()

                row = {
                    "frame": frame_index,
                    "time_s": round(time_s, 4),
                    "detected": 0,
                    "tag_x_cm": "",
                    "tag_y_cm": "",
                    "robot_x_cm": "",
                    "robot_y_cm": "",
                    "heading_deg": ""
                }

                pose = None
                robot_map_pose = None

                if corners is not None:
                    pose = estimate_pose(
                        corners, H, meta,
                        args.heading_offset_deg
                    )

                    tag_x, tag_y, theta = pose

                    robot_x, robot_y = robot_center_from_tag(
                        pose, args.tag_forward_offset_cm
                    )

                    row.update({
                        "detected": 1,
                        "tag_x_cm": round(tag_x, 3),
                        "tag_y_cm": round(tag_y, 3),
                        "robot_x_cm": round(robot_x, 3),
                        "robot_y_cm": round(robot_y, 3),
                        "heading_deg": round(theta, 3)
                    })

                    trail.append((robot_x, robot_y))
                    robot_map_pose = (robot_x, robot_y, theta)

                    annotate_marker(
                        overlay, corners, pose,
                        args.marker_id, (robot_x, robot_y)
                    )

                else:
                    cv2.putText(
                        overlay, "MARKER NOT DETECTED",
                        (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2
                    )

                csv_writer.writerow(row)

                last_map = draw_map(
                    meta, trail, robot_map_pose
                )

                if writer is not None:
                    writer.write(overlay)

                if args.display:
                    cv2.imshow("ArUco tracking", overlay)

                    if args.show_map:
                        cv2.imshow(
                            "Tabletop map", last_map
                        )

                    key = cv2.waitKey(1) & 0xFF

                    if key in (ord("q"), 27):
                        break

                frame_index += 1

    finally:
        cap.release()

        if writer is not None:
            writer.release()

        if args.display:
            cv2.destroyAllWindows()

    if last_map is not None:
        save_image(
            output_dir / "trajectory_map.png",
            last_map
        )

    print(f"Trajectory CSV: {csv_path}")


# ------------------------------------------------------------
# COMMAND LINE INTERFACE
# ------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Tabletop perception pipeline"
    )

    commands = parser.add_subparsers(
        dest="command", required=True
    )

    # Test
    p = commands.add_parser("test")
    p.add_argument("image")
    p.add_argument("--homography", default=DEFAULT_H)

    p.add_argument(
        "--ground-truth", nargs=2,
        type=float, metavar=("X", "Y")
    )

    p.add_argument(
        "--output",
        default=str(RESULTS_DIR / "test_overlay.png")
    )

    p.add_argument("--bev-preview", action="store_true")
    p.set_defaults(func=test_position)

    # Pose
    p = commands.add_parser("pose")
    p.add_argument("image")

    p.add_argument(
        "--homography", default=DEFAULT_H,
        help="Existing tabletop homography (legacy apparent position)"
    )

    p.add_argument(
        "--tag-homography", type=Path, default=None,
        help="Homography calibrated at the elevated ArUco marker plane"
    )

    p.add_argument("--tag-plane-height-cm", type=float, default=11.0)

    p.add_argument(
        "--tag-forward-offset-cm", type=float, default=4.5,
        help="Distance from robot center to tag center along robot forward axis"
    )

    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--dictionary", default="4X4_50")
    p.add_argument("--heading-offset-deg", type=float, default=0)

    p.add_argument(
        "--output",
        default=str(RESULTS_DIR / "marker_pose_overlay.png")
    )

    p.add_argument("--bev-preview", action="store_true")
    p.set_defaults(func=detect_pose_image)

    # Track
    p = commands.add_parser("track")
    p.add_argument("source")

    p.add_argument(
        "--homography", default=DEFAULT_H,
        help="Existing tabletop homography (legacy apparent position)"
    )

    p.add_argument(
        "--tag-homography", type=Path, default=None,
        help="Homography calibrated at the elevated ArUco marker plane"
    )

    p.add_argument("--tag-plane-height-cm", type=float, default=11.0)

    p.add_argument(
        "--tag-forward-offset-cm", type=float, default=4.5,
        help="Distance from robot center to tag center along robot forward axis"
    )

    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--dictionary", default="4X4_50")
    p.add_argument("--heading-offset-deg", type=float, default=0)

    p.add_argument(
        "--output-dir",
        default=str(RESULTS_DIR / "tracking")
    )

    p.add_argument("--display", action="store_true")
    p.add_argument("--show-map", action="store_true")
    p.add_argument("--save-video", action="store_true")
    p.set_defaults(func=track_video)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

