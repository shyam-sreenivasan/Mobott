"""Detect ArUco 4x4 marker ID 0 and show it in the calibrated tabletop BEV.

Examples:
  python aruco_bev_pose.py pose.png --homography workspace_bev.npy
  python aruco_bev_pose.py robot-positions.mp4 --homography workspace_bev.npy --times 3 13 21 33 41 46 53

The homography must come from the SAME fixed camera, resolution, and framing.
Coordinates correspond to the marker plane *as projected using a tabletop homography*;
for an elevated marker, these are NOT height-corrected ground-plane coordinates.
"""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np


def detect(frame, detector, wanted_id):
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return None
    for quad, marker_id in zip(corners, ids.flatten()):
        if int(marker_id) == wanted_id:
            return quad.reshape(4, 2).astype(np.float32)
    return None


def arrow_and_quad(image, quad, label):
    p = np.asarray(quad, dtype=np.float32)
    center = p.mean(axis=0)
    front = (p[0] + p[1]) / 2  # direction toward TOP EDGE of marker
    direction = front - center
    norm = np.linalg.norm(direction)
    if norm > 1e-6:
        tip = center + direction / norm * max(20, norm * 1.2)
    else:
        tip = center
    cv2.polylines(image, [np.round(p).astype(np.int32)], True, (0, 220, 0), 2, cv2.LINE_AA)
    cv2.circle(image, tuple(np.round(center).astype(int)), 5, (0, 255, 255), -1)
    cv2.arrowedLine(image, tuple(np.round(center).astype(int)), tuple(np.round(tip).astype(int)),
                    (0, 90, 255), 3, cv2.LINE_AA, tipLength=0.25)
    at = tuple(np.round(center + np.array([12, -12])).astype(int))
    cv2.putText(image, label, at, cv2.FONT_HERSHEY_SIMPLEX, .55,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, label, at, cv2.FONT_HERSHEY_SIMPLEX, .55,
                (255, 255, 255), 2, cv2.LINE_AA)
    return center, front


def draw_grid(bev, width_cm, height_cm, ppcm):
    hh, ww = bev.shape[:2]
    for cm in range(0, int(width_cm) + 1, 10):
        x = min(ww - 1, round(cm * ppcm))
        cv2.line(bev, (x, 0), (x, hh - 1), (110, 180, 110), 1)
        cv2.putText(bev, str(cm), (max(0, min(x+3, ww-34)), 20),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 80, 0), 1, cv2.LINE_AA)
    for cm in range(0, int(height_cm) + 1, 10):
        y = min(hh - 1, round(cm * ppcm))
        cv2.line(bev, (0, y), (ww - 1, y), (110, 180, 110), 1)
        cv2.putText(bev, str(int(height_cm - cm)), (4, min(y + 17, hh - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 80, 0), 1, cv2.LINE_AA)


def frames_for_input(path, times):
    if path.suffix.lower() in {'.png', '.jpg', '.jpeg', '.bmp', '.webp'}:
        img = cv2.imread(str(path))
        if img is None:
            raise SystemExit(f'Cannot read {path}')
        yield 0.0, img
    else:
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            raise SystemExit(f'Cannot open video: {path}')
        try:
            for sec in times:
                cap.set(cv2.CAP_PROP_POS_MSEC, float(sec)*1000)
                success, frame = cap.read()
                if success:
                    yield float(sec), frame
                else:
                    print(f'Skipping {sec}s: no readable frame')
        finally:
            cap.release()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('input', type=Path, help='Camera image or video')
    p.add_argument('--homography', type=Path, default=Path('workspace_bev.npy'))
    p.add_argument('--width-cm', type=float, default=90.0)
    p.add_argument('--height-cm', type=float, default=74.0)
    p.add_argument('--pixels-per-cm', type=float, default=10.0)
    p.add_argument('--marker-id', type=int, default=0)
    p.add_argument('--times', type=float, nargs='+', default=[3, 13, 21, 33, 37, 41, 46, 53],
                   help='Seconds to sample if input is a video')
    p.add_argument('--out', type=Path, default=Path('pose_results'))
    a = p.parse_args()
    if not a.homography.is_file():
        raise SystemExit(f'Missing {a.homography}. Copy your calibration .npy next to this script or pass --homography PATH')
    H = np.load(a.homography).astype(np.float64)
    metadata_path = a.homography.with_suffix('.json')
    if not metadata_path.is_file():
        raise SystemExit(f'Missing {metadata_path}; recalibrate using tabletop_pipeline.py')
    meta = json.loads(metadata_path.read_text())
    if meta.get('coordinate_frame_version') != 'X_BOTTOM_LEFT_X_RIGHT_Y_UP_V1':
        raise SystemExit('Calibration uses an old coordinate frame; recalibrate with updated tabletop_pipeline.py')
    a.width_cm, a.height_cm = meta['width_cm'], meta['height_cm']
    if H.shape != (3, 3):
        raise SystemExit('Homography must be 3x3.')
    w, h = meta['bev_width_px'], meta['bev_height_px']
    a.out.mkdir(exist_ok=True, parents=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    rows = []
    for k, (t, frame) in enumerate(frames_for_input(a.input, a.times), 1):
        if frame.shape[1] != meta['image_width_px'] or frame.shape[0] != meta['image_height_px']:
            raise SystemExit('Input frame resolution does not match calibration. Recalibrate or use matching video mode.')
        quad = detect(frame, detector, a.marker_id)
        if quad is None:
            print(f'{t:.1f}s: marker ID {a.marker_id} not found')
            continue
        overlay = frame.copy()
        center_px, _ = arrow_and_quad(overlay, quad, f'ID {a.marker_id}')
        bev = cv2.warpPerspective(frame, H, (w, h))
        bev_quad = cv2.perspectiveTransform(quad.reshape(1, 4, 2), H).reshape(4, 2)
        bev_center, bev_front = arrow_and_quad(bev, bev_quad, f'ID {a.marker_id}')
        dx, dy = bev_front - bev_center
        angle = (np.degrees(np.arctan2(-dy, dx)) + 360) % 360  # 0 right, +90 up
        x_cm = bev_center[0] * a.width_cm / (w-1)
        y_cm = (1 - bev_center[1] / (h-1)) * a.height_cm
        draw_grid(bev, a.width_cm, a.height_cm, a.pixels_per_cm)
        label = f'({x_cm:.1f}, {y_cm:.1f}) cm   heading {angle:.1f} deg'
        cv2.rectangle(bev, (0, h-32), (min(w, len(label)*11), h), (255, 255, 255), -1)
        cv2.putText(bev, label, (6, h-10), cv2.FONT_HERSHEY_SIMPLEX, .55, (10, 10, 10), 1, cv2.LINE_AA)
        stem = f'pose_{k:02d}_{t:05.1f}s'
        cv2.imwrite(str(a.out / f'{stem}_camera.png'), overlay)
        cv2.imwrite(str(a.out / f'{stem}_bev.png'), bev)
        rows.append({'time_s': t, 'marker_id': a.marker_id,
                     'image_u_px': round(float(center_px[0]), 2),
                     'image_v_px': round(float(center_px[1]), 2),
                     'workspace_x_cm_apparent': round(float(x_cm), 3),
                     'workspace_y_cm_apparent': round(float(y_cm), 3),
                     'heading_deg_ccw_from_X_to_F': round(float(angle), 2)})
        print(f'{t:5.1f}s -> ({x_cm:6.2f}, {y_cm:6.2f}) cm, heading {angle:6.1f} deg')
    if rows:
        with (a.out/'poses.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        print(f'Saved {len(rows)} annotated pairs + {a.out / "poses.csv"}')
    else:
        print('No markers detected. Confirm marker dictionary, image, and visibility.')


if __name__ == '__main__':
    main()
