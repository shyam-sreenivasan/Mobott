# Tabletop Perception Pipeline

A Python/OpenCV pipeline for calibrating a fixed camera, estimating object positions, and tracking ArUco markers in a tabletop workspace.

## Setup

```bash
pip install opencv-contrib-python numpy
```

**Workspace coordinate system:**
- Origin: X = (0, 0)
- X → F: +X axis, 90 cm
- X → M: +Y axis, 74 cm
- Heading: 0° along X → F, increasing counterclockwise

## Usage

### 1. Extract a sharp frame

```bash
python tabletop_pipeline.py extract-frame empty.mp4 \
    --start 2 --end 3 --output empty.png
```

Selects the sharpest frame within the specified time interval.

### 2. Calibrate the camera

```bash
python tabletop_pipeline.py calibrate empty.png
```

Click the four tabletop boundary corners in order: **M → Y → F → X**.

Generates:
- `workspace_bev.npy` — homography transformation matrix
- `workspace_bev.json` — calibration metadata
- `workspace_bev.png` — bird's-eye-view visualization

### 3. Validate object positions

```bash
python tabletop_pipeline.py test objects.png \
    --ground-truth 20 30 --bev-preview
```

Click an object reference point to compare its estimated coordinates against measured ground truth.

### 4. Detect ArUco pose

```bash
python tabletop_pipeline.py pose marker.png --bev-preview
```

Detects ArUco marker ID 0 and estimates its position and heading: `(x_cm, y_cm, theta_deg)`.

### 5. Track ArUco in video

```bash
python tabletop_pipeline.py track marker_motion.mp4 \
    --display --show-map --save-video
```

Tracks marker pose continuously and generates:

```text
track_results/
├── annotated.mp4
├── trajectory.csv
└── trajectory_map.png
```

## Video preprocessing

Rotate video 90° clockwise:

```bash
ffmpeg -i input.mp4 -vf "transpose=1" -metadata:s:v:0 rotate=0 output.mp4
```

Rotate 90° counterclockwise:

```bash
ffmpeg -i input.mp4 -vf "transpose=2" -metadata:s:v:0 rotate=0 output.mp4
```

## Important Notes

- Keep the camera position, orientation, resolution, and field of view unchanged after calibration.
- Use video frames for both calibration and tracking to avoid iPhone photo/video aspect-ratio differences.
- ArUco detection operates on original frames; the homography transforms detected points into tabletop coordinates without requiring a full BEV image.
- Homography assumes points lie on the tabletop plane. Elevated markers introduce position error.
- Tracking currently provides perception only, without robot motor control.

## Next Steps

Integrate a live camera stream through ROS 2, followed by manual robot commands and state-transition logging. Later stages will introduce boundary-safe roaming, obstacle avoidance, and object pushing.
