#!/usr/bin/env python3
"""
Convert a phone-recorded video into a still image.

Videos are read from: data/videos/
Extracted images are saved to: data/images/

python convert.py robot_movement.mp4 --output calibration_image.png

Extract the sharpest frame between 2 and 5 seconds:
python convert.py robot_movement.mp4 --output calibration_image.png --start 2 --end 5
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


# ------------------------------------------------------------
# FOLDER PATHS
# ------------------------------------------------------------

PROJECT_DIR = Path(__file__).resolve().parent
VIDEO_DIR = PROJECT_DIR / "data" / "videos"
IMAGE_DIR = PROJECT_DIR / "data" / "images"


# ------------------------------------------------------------
# PATH HANDLING
# ------------------------------------------------------------

def resolve_video_path(video):
    """Find the video in data/videos unless a path is provided."""
    path = Path(video)

    if path.is_absolute() or path.exists():
        return path

    return VIDEO_DIR / path


def resolve_output_path(output):
    """Save a filename in data/images unless a path is provided."""
    path = Path(output)

    if path.is_absolute() or path.parent != Path("."):
        return path

    return IMAGE_DIR / path


# ------------------------------------------------------------
# VIDEO TO IMAGE CONVERSION
# ------------------------------------------------------------

def convert_video(args):

    video_path = resolve_video_path(args.video)
    output_path = resolve_output_path(args.output)

    if not video_path.is_file():
        raise FileNotFoundError(f"Video not found: {video_path}")

    if args.start < 0:
        raise ValueError("Start time cannot be negative.")

    if args.end is not None and args.end <= args.start:
        raise ValueError("End time must be greater than start time.")

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")

    try:
        fps = cap.get(cv2.CAP_PROP_FPS)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        if fps <= 0 or total_frames <= 0:
            raise RuntimeError("Cannot determine video FPS or frame count.")

        duration = total_frames / fps

        if args.start >= duration:
            raise ValueError(
                f"Start time is beyond video duration ({duration:.2f} seconds)."
            )

        end_time = duration if args.end is None else min(args.end, duration)

        start_frame = int(np.ceil(args.start * fps))
        end_frame = min(total_frames, int(np.ceil(end_time * fps)))

        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

        best_score = -1.0
        best_frame = None
        best_frame_index = -1

        # Find the sharpest frame in the selected interval.
        for frame_index in range(start_frame, end_frame):

            success, frame = cap.read()

            if not success:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            sharpness = cv2.Laplacian(
                gray, cv2.CV_64F
            ).var()

            if sharpness > best_score:
                best_score = sharpness
                best_frame = frame.copy()
                best_frame_index = frame_index

        if best_frame is None:
            raise RuntimeError("No frames could be extracted from the video.")

        # Create output folder and save the image.
        output_path.parent.mkdir(parents=True, exist_ok=True)

        if not cv2.imwrite(str(output_path), best_frame):
            raise RuntimeError(f"Could not save image: {output_path}")

        print("\nVideo conversion completed.")
        print(f"Input video : {video_path}")
        print(f"Output image: {output_path}")
        print(f"Frame time  : {best_frame_index / fps:.3f} seconds")
        print(f"Sharpness   : {best_score:.2f}")
        print(
            f"Resolution  : {best_frame.shape[1]} x "
            f"{best_frame.shape[0]}"
        )

    finally:
        cap.release()


# ------------------------------------------------------------
# COMMAND LINE INTERFACE
# ------------------------------------------------------------

def main():

    parser = argparse.ArgumentParser(
        description="Extract the sharpest frame from a video."
    )

    parser.add_argument(
        "video",
        help="Video filename inside data/videos/ or a full path."
    )

    parser.add_argument(
        "--output",
        required=True,
        help="Name of the output image."
    )

    parser.add_argument(
        "--start",
        type=float,
        default=0.0,
        help="Starting time in seconds. Default: 0."
    )

    parser.add_argument(
        "--end",
        type=float,
        default=None,
        help="Ending time in seconds. Default: end of video."
    )

    args = parser.parse_args()

    convert_video(args)


if __name__ == "__main__":
    main()
