
#!/usr/bin/env python3
"""Tabletop calibration, localization and ArUco tracking.

Coordinate convention:

    M (0,74) -------- Y (90,74)
       |                  |
       |                  |
    X (0,0) --------- F (90,0)

+X: X -> F
+Y: X -> M
Heading 0 degrees: X -> F
Positive heading: counterclockwise

The homography maps points on the tabletop plane.
Elevated ArUco markers have apparent, not height-corrected,
tabletop positions.
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
DEFAULT_H = Path("workspace_bev.npy")
FRAME_VERSION = "X_BOTTOM_LEFT_X_RIGHT_Y_UP_V1"


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
# 1. EXTRACT SHARPEST FRAME
# ------------------------------------------------------------

def extract_frame(args):
    if args.start < 0 or args.end <= args.start:
        raise ValueError("Require 0 <= --start < --end")

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open: {args.video}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        cap.release()
        raise RuntimeError("Cannot determine video FPS")

    start_frame = int(np.ceil(args.start * fps))
    end_frame = int(np.floor(args.end * fps))

    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    best_score = -1.0
    best_image = None
    best_index = -1

    for index in range(start_frame, end_frame + 1):
        success, frame = cap.read()
        if not success:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        score = cv2.Laplacian(
            gray, cv2.CV_64F
        ).var()

        if score > best_score:
            best_score = score
            best_image = frame.copy()
            best_index = index

    cap.release()

    if best_image is None:
        raise RuntimeError("No frames found in selected interval")

    save_image(args.output, best_image)

    print(f"Time: {best_index / fps:.3f} seconds")
    print(f"Sharpness: {best_score:.2f}")
    print(
        f"Resolution: {best_image.shape[1]} x "
        f"{best_image.shape[0]}"
    )
    print("Check that the selected frame is unobstructed.")


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
# 3. CALIBRATION
# ------------------------------------------------------------

def calibrate(args):
    image = read_image(args.image)

    print("Select taped corners in order: M -> Y -> F -> X")
    print("Use boundary corners, not paper marker centers.")

    corners = select_points(
        image, ["M", "Y", "F", "X"]
    )

    if corners is None:
        print("Calibration cancelled")
        return

    if cv2.contourArea(corners) < 100:
        raise RuntimeError("Invalid corner selection")

    width_px = round(args.width_cm * args.pixels_per_cm)
    height_px = round(args.height_cm * args.pixels_per_cm)

    destination = np.array([
        [0, 0],
        [width_px - 1, 0],
        [width_px - 1, height_px - 1],
        [0, height_px - 1]
    ], dtype=np.float32)

    H = cv2.getPerspectiveTransform(
        corners, destination
    )

    path = Path(args.homography)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.save(str(path), H)

    metadata = {
        "width_cm": args.width_cm,
        "height_cm": args.height_cm,
        "image_width_px": image.shape[1],
        "image_height_px": image.shape[0],
        "bev_width_px": width_px,
        "bev_height_px": height_px,
        "pixels_per_cm": args.pixels_per_cm,
        "corner_order": ["M", "Y", "F", "X"],
        "coordinate_frame_version": FRAME_VERSION,
        "source_corners_px": corners.tolist()
    }

    path.with_suffix(".json").write_text(
        json.dumps(metadata, indent=2)
    )

    bev = cv2.warpPerspective(
        image, H, (width_px, height_px)
    )

    add_grid(bev, args.width_cm, args.height_cm)
    save_image(args.output, bev)

    print(f"Homography: {path}")
    print(f"Metadata: {path.with_suffix('.json')}")
    print("Origin X=(0,0), +X toward F, +Y toward M")


def load_calibration(path, image):
    path = Path(path)

    if not path.exists():
        raise RuntimeError("Missing homography. Run calibrate first.")

    meta_path = path.with_suffix(".json")
    if not meta_path.exists():
        raise RuntimeError("Missing calibration JSON. Recalibrate.")

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
    image = read_image(args.image)
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


def annotate_marker(image, corners, pose, marker_id):
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
        f"ID {marker_id} | "
        f"x={x:.1f} y={y:.1f} "
        f"heading={theta:.1f}"
    )

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
    image = read_image(args.image)
    H, meta = load_calibration(args.homography, image)

    detector = create_detector(args.dictionary)
    corners = detect_marker(
        image, detector, args.marker_id
    )

    if corners is None:
        print("Marker not detected")
        return

    pose = estimate_pose(
        corners, H, meta, args.heading_offset_deg
    )

    x, y, theta = pose

    print(
        f"Pose: x={x:.2f} cm, "
        f"y={y:.2f} cm, theta={theta:.2f} deg"
    )

    overlay = image.copy()
    annotate_marker(
        overlay, corners, pose, args.marker_id
    )
    save_image(args.output, overlay)

    result = {
        "marker_id": args.marker_id,
        "x_cm_apparent": x,
        "y_cm_apparent": y,
        "heading_deg": theta
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
            bev, bev_corners, pose, args.marker_id
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
        else args.source
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
                "x_cm", "y_cm", "heading_deg"
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
                    H, meta = load_calibration(
                        args.homography, frame
                    )

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
                    "x_cm": "",
                    "y_cm": "",
                    "heading_deg": ""
                }

                pose = None

                if corners is not None:
                    pose = estimate_pose(
                        corners, H, meta,
                        args.heading_offset_deg
                    )

                    x, y, theta = pose

                    row.update({
                        "detected": 1,
                        "x_cm": round(x, 3),
                        "y_cm": round(y, 3),
                        "heading_deg": round(theta, 3)
                    })

                    trail.append((x, y))

                    annotate_marker(
                        overlay, corners, pose,
                        args.marker_id
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
                    meta, trail, pose
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

    # Extract frame
    p = commands.add_parser("extract-frame")
    p.add_argument("video")
    p.add_argument("--start", type=float, default=2.0)
    p.add_argument("--end", type=float, default=3.0)
    p.add_argument("--output", default="sharp_frame.png")
    p.set_defaults(func=extract_frame)

    # Calibrate
    p = commands.add_parser("calibrate")
    p.add_argument("image")
    p.add_argument("--width-cm", type=float, default=WIDTH_CM)
    p.add_argument("--height-cm", type=float, default=HEIGHT_CM)
    p.add_argument(
        "--pixels-per-cm",
        type=float, default=PIXELS_PER_CM
    )
    p.add_argument("--homography", default=DEFAULT_H)
    p.add_argument("--output", default="workspace_bev.png")
    p.set_defaults(func=calibrate)

    # Test
    p = commands.add_parser("test")
    p.add_argument("image")
    p.add_argument("--homography", default=DEFAULT_H)
    p.add_argument(
        "--ground-truth", nargs=2,
        type=float, metavar=("X", "Y")
    )
    p.add_argument("--output", default="test_overlay.png")
    p.add_argument("--bev-preview", action="store_true")
    p.set_defaults(func=test_position)

    # Pose
    p = commands.add_parser("pose")
    p.add_argument("image")
    p.add_argument("--homography", default=DEFAULT_H)
    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--dictionary", default="4X4_50")
    p.add_argument("--heading-offset-deg", type=float, default=0)
    p.add_argument(
        "--output", default="marker_pose_overlay.png"
    )
    p.add_argument("--bev-preview", action="store_true")
    p.set_defaults(func=detect_pose_image)

    # Track
    p = commands.add_parser("track")
    p.add_argument("source")
    p.add_argument("--homography", default=DEFAULT_H)
    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--dictionary", default="4X4_50")
    p.add_argument("--heading-offset-deg", type=float, default=0)
    p.add_argument("--output-dir", default="track_results")
    p.add_argument("--display", action="store_true")
    p.add_argument("--show-map", action="store_true")
    p.add_argument("--save-video", action="store_true")
    p.set_defaults(func=track_video)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
