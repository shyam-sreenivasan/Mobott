
#!/usr/bin/env python3
"""Create named tabletop or elevated-plane homography calibrations."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parent
DATA_IMAGE_DIR = PROJECT_DIR / "data" / "images"
CALIBRATION_DIR = PROJECT_DIR / "calibration"
CALIBRATION_IMAGE_DIR = CALIBRATION_DIR / "images"

WIDTH_CM = 90.0
HEIGHT_CM = 74.0
PIXELS_PER_CM = 10.0
FRAME_VERSION = "X_BOTTOM_LEFT_X_RIGHT_Y_UP_V1"


def resolve_image_path(value):
    path = Path(value)
    if path.is_absolute() or path.exists():
        return path
    return DATA_IMAGE_DIR / path


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
    print(f"Saved image: {path}")


def select_points(image, labels):
    scale = min(1.0, 1200 / image.shape[1], 850 / image.shape[0])
    preview = cv2.resize(image, None, fx=scale, fy=scale)

    selected = []
    window = "Select points | Enter=confirm U=undo R=reset Q=quit"

    def mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN and len(selected) < len(labels):
            selected.append((x / scale, y / scale))

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, mouse)

    while True:
        display = preview.copy()

        for i, point in enumerate(selected):
            px, py = int(point[0] * scale), int(point[1] * scale)
            cv2.circle(display, (px, py), 6, (0, 0, 255), -1)
            cv2.putText(
                display, labels[i], (px + 10, max(20, py - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
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

        if key in (13, 10, 32) and len(selected) == len(labels):
            cv2.destroyAllWindows()
            return np.array(selected, dtype=np.float32)


def add_grid(image, width_cm, height_cm):
    h, w = image.shape[:2]

    for x in range(0, int(width_cm) + 1, 10):
        px = round(x / width_cm * (w - 1))
        cv2.line(image, (px, 0), (px, h - 1), (100, 180, 100), 1)

    for y in range(0, int(height_cm) + 1, 10):
        py = round((1 - y / height_cm) * (h - 1))
        cv2.line(image, (0, py), (w - 1, py), (100, 180, 100), 1)

    return image


def calibrate(args):
    image_path = resolve_image_path(args.image)
    image = read_image(image_path)

    if abs(args.plane_height_cm) < 1e-6:
        print("Select tabletop boundary corners in order: M -> Y -> F -> X")
    else:
        print(
            f"Select calibration-frame corners at {args.plane_height_cm:.1f} cm "
            "above the table, in order: M -> Y -> F -> X"
        )

    print("Select the physical workspace corners, not marker centers.")

    corners = select_points(image, ["M", "Y", "F", "X"])

    if corners is None:
        print("Calibration cancelled.")
        return

    if abs(cv2.contourArea(corners)) < 100:
        raise RuntimeError("Invalid corner selection.")

    width_px = round(args.width_cm * args.pixels_per_cm)
    height_px = round(args.height_cm * args.pixels_per_cm)

    destination = np.array([
        [0, 0],
        [width_px - 1, 0],
        [width_px - 1, height_px - 1],
        [0, height_px - 1]
    ], dtype=np.float32)

    H = cv2.getPerspectiveTransform(corners, destination)

    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    CALIBRATION_IMAGE_DIR.mkdir(parents=True, exist_ok=True)

    homography_path = CALIBRATION_DIR / f"{args.name}.npy"
    metadata_path = homography_path.with_suffix(".json")
    workspace_path = CALIBRATION_IMAGE_DIR / f"{args.name}_workspace.png"

    metadata = {
        "calibration_name": args.name,
        "source_image": str(image_path),
        "width_cm": args.width_cm,
        "height_cm": args.height_cm,
        "image_width_px": image.shape[1],
        "image_height_px": image.shape[0],
        "bev_width_px": width_px,
        "bev_height_px": height_px,
        "pixels_per_cm": args.pixels_per_cm,
        "plane_height_cm": args.plane_height_cm,
        "corner_order": ["M", "Y", "F", "X"],
        "coordinate_frame_version": FRAME_VERSION,
        "source_corners_px": corners.tolist()
    }

    np.save(str(homography_path), H)
    metadata_path.write_text(json.dumps(metadata, indent=2))

    bev = cv2.warpPerspective(image, H, (width_px, height_px))
    add_grid(bev, args.width_cm, args.height_cm)
    save_image(workspace_path, bev)

    print("\nCalibration completed.")
    print(f"Name:       {args.name}")
    print(f"Homography: {homography_path}")
    print(f"Metadata:   {metadata_path}")
    print(f"Workspace:  {workspace_path}")
    print("Origin X=(0,0), +X toward F, +Y toward M")


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate the tabletop or an elevated marker plane."
    )

    parser.add_argument(
        "image",
        help="Calibration image filename in data/images/ or a full path."
    )
    parser.add_argument(
        "--name",
        required=True,
        help="Unique name, e.g. video1_tabletop or video1_tag_plane."
    )
    parser.add_argument("--width-cm", type=float, default=WIDTH_CM)
    parser.add_argument("--height-cm", type=float, default=HEIGHT_CM)
    parser.add_argument("--pixels-per-cm", type=float, default=PIXELS_PER_CM)
    parser.add_argument(
        "--plane-height-cm", type=float, default=0.0,
        help="Height of calibrated plane above table (0 for tabletop)."
    )

    args = parser.parse_args()
    calibrate(args)


if __name__ == "__main__":
    main()

