# Autonomous Navigation: Aruco Marker Localization

A fixed overhead camera watches a tabletop workspace. Four ArUco markers mark
the workspace corners, and a fifth marker on the robot is located live, giving
its **position in centimeters** and its **heading in degrees**.

```text
iPhone camera (Conduit) -> ROS 2 image topic
        |
        +-- calibrate:  find corner markers 1-4 -> homography (pixels -> cm) -> save
        |
        +-- localize:   find marker 0 -> pixels -> cm -> position + heading -> display
```

This is the base for click-to-goal navigation on the table, which is in
progress.

## Requirements

- ROS 2 Humble, sourced in the terminal (`source /opt/ros/humble/setup.bash`)
- The Conduit iPhone app publishing `/conduit/camera/front/image_raw/compressed`
  (setup is in the [main README](../../README.md#run))
- Python packages: `numpy` and OpenCV with the ArUco module (`cv2.aruco`).
  `opencv-contrib-python` 4.7 or newer is recommended.
- Five printed ArUco markers from the **DICT_4X4_50** dictionary: IDs 0–4

## Folder Structure

```text
main.py                     # menu: runs everything below
robot-live-detection.py     # live localization of marker ID 0
setup/
  config.json               # workspace and marker measurements
  calibrate.py              # workspace calibration from the live camera
  check_conduit_camera.py   # checks that the camera stream works
  convert-video-to-img.py   # old: sharpest frame from a recorded video (unused)
calibration/                # saved calibrations (created by calibrate.py)
  workspace_1.npy           # homography: camera pixels -> table cm
  workspace_1.json          # calibration details
  images/workspace_1_bev.png  # bird's-eye reference picture
clickon-navigation.py       # click-to-goal path planning (not yet updated, see below)
data/                       # earlier test images and videos
```

## Workspace Setup

### 1. Place the corner markers

Put markers 1–4 at the four corners of the workspace, flat on the table, with
their edges lined up with the table edges:

```text
   Y (ID 2) ---------------------- U (ID 3)
   (0, y_length)                   (x_length, y_length)
      |                                 |
      |            workspace            |      +y
      |                                 |       ^
      |                                 |       |
   X (ID 1) ---------------------- D (ID 4)     +--> +x
   (0, 0)                          (x_length, 0)
```

The workspace corners are the **outer corners** of the markers: the
bottom-left corner of X is (0, 0), the top-left corner of Y is
(0, `y_length`), and so on.

### 2. Enter the measurements

Edit `setup/config.json`:

```json
{
  "x_length_cm": 90,
  "y_length_cm": 76,
  "marker_width_cm": 10,
  "marker_height_cm": 10
}
```

| Field | Measure |
|---|---|
| `x_length_cm` | Outer corner of X to outer corner of D |
| `y_length_cm` | Outer corner of X to outer corner of Y |
| `marker_width_cm` | Width of a corner marker (all four the same size) |
| `marker_height_cm` | Height of a corner marker |

Measure carefully: every position the system reports is scaled by these numbers.

### 3. Prepare the robot marker

Use marker **ID 0**. The heading is the direction of the marker's printed
**top edge**, so point that edge toward the front of the robot.

Keep a white border around every marker (ArUco needs it), and use stiff,
matte card so the markers stay flat and don't reflect light.

## Usage

Source ROS 2, start Conduit, then:

```bash
cd tasks/autonomous_nav
python3 main.py
```

```text
1. Check camera stream
2. Calibrate workspace
3. Live localization (Manual Control)
0. Exit
```

### 1. Check camera stream

Opens the live camera. If no frames arrive within 5 seconds, it prints
`Problem with the camera stream. Check if the topics are publishing`.
Close the window with **Q**, **Esc**, or the window's close button.

### 2. Calibrate workspace

Run this whenever the camera or the corner markers move.

1. Collects live frames for 2 seconds and keeps the sharpest one.
2. Finds markers 1–4. If any is missing, it prints
   `The marker <name> is not detected so unable to calibrate` and stops.
3. Calculates the homography from the marker positions and `config.json`.
4. Saves the next free number, without overwriting earlier calibrations:

| File | Contents |
|---|---|
| `calibration/workspace_<i>.npy` | 3×3 homography, camera pixels → table cm |
| `calibration/workspace_<i>.json` | Workspace and marker sizes, camera resolution, camera topic, detected marker centers, date |
| `calibration/images/workspace_<i>_bev.png` | Bird's-eye picture of the workspace, for checking by eye |

**Check the bird's-eye picture.** The four markers should sit neatly in its
corners. If one is out of place or the table looks skewed, check the marker
placement and the measurements in `config.json`, then recalibrate.

Run directly: `python3 setup/calibrate.py`

### 3. Live localization

Lists the available calibrations and asks which one to use. Two windows open:
the camera view and a live bird's-eye view of the workspace. When marker ID 0
is inside the workspace:

- a green box is drawn around it,
- a red arrow shows the direction it faces,
- a bar at the top of both windows shows its position and heading:

```text
ID 0 Marker = Detected, x = 33.25 cm, y = 23.77 cm, angle = 134 deg
ID 0 Marker = Not detected, x = -, y = -, angle = -
```

The terminal also prints when the marker enters or leaves the workspace.

Run directly: `python3 robot-live-detection.py --calibration workspace_1`

## Coordinates and Heading

Positions are in cm, measured from the outer corner of marker X, with x toward
D and y toward Y. The position reported is the **center** of marker 0.

The heading runs from 0° to 359°, counterclockwise:

| Marker's top edge faces | Heading |
|---|---|
| toward D (+x) | 0° |
| toward Y (+y) | 90° |
| toward X from D (−x) | 180° |
| toward X from Y (−y) | 270° |

## How It Works

**Calibration.** OpenCV finds the four corner markers and their centers in the
camera image. Each center is half a marker inward from its outer corner, so
X's center is at (`marker_width`/2, `marker_height`/2) cm, and so on. Mapping
the four detected centers to these positions gives a homography: a 3×3 matrix
that converts any camera pixel on the table surface into table centimeters,
undoing the camera's angled perspective.

**Localization.** Marker 0 is detected in the original camera frame, which is
sharper than the bird's-eye view. Its four corners are converted to cm with the
homography, and their average is the marker's position. The heading is the
direction from the marker's center to the middle of its top edge, calculated in
cm so the camera's perspective doesn't distort the angle.

**Bird's-eye view.** The homography gives centimeters, so for display it is
combined with a ×10 scale (10 pixels per cm) and a vertical flip (images count
y downward, the table counts it upward). A 90 × 76 cm workspace becomes a
900 × 760 pixel picture with Y at the top. This is only for viewing; positions
always come from the homography directly.

## Limitations

- **The calibration only applies at table height.** A marker raised above the
  table, such as one mounted on top of the robot, appears shifted away from the
  camera, by several cm depending on its height and the camera angle. Positions
  are exact for a marker lying flat on the table.
- **The camera must not move after calibrating.** Even a small bump shifts
  every position. Recalibrate if it moves.
- **The camera resolution must match.** A calibration only works at the
  resolution it was made with. The live script checks this on the first frame
  and asks you to recalibrate if it differs.
- **Each frame is detected from scratch.** Fast motion blurs the marker, and
  it briefly shows as "Not detected". Good lighting and a larger marker help.
- **The marker must be fully visible**, with its white border, inside the
  workspace.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `Problem with the camera stream. Check if the topics are publishing` | Conduit is not publishing, or ROS 2 is not sourced. Check with `ros2 topic hz /conduit/camera/front/image_raw/compressed`. |
| `ModuleNotFoundError: No module named 'rclpy'` | ROS 2 is not sourced in this terminal. |
| `The marker <name> is not detected so unable to calibrate` | That corner marker is covered, outside the camera view, too small, or reflecting light. |
| `KeyError: 'x_length_cm'` | An old-format calibration (`empty*.npy`) was selected. Use a `workspace_<i>` calibration. |
| `Camera resolution ... does not match` | The camera resolution changed since calibrating. Recalibrate. |
| Positions are off by a constant amount | The marker is raised above the table, or the camera moved. |
| Positions are off more toward one side of the table | `x_length_cm` / `y_length_cm` don't match the real distances. |
| Bird's-eye view is skewed or partly black | The camera moved, or the markers moved since calibrating. |

## Old and In-Progress Files

- **`clickon-navigation.py`** (click-to-goal RRT* planning) still uses the
  previous localization code (`robot_localize.py`, now removed), so it won't run
  until it is updated for the new calibration.
- **`setup/convert-video-to-img.py`** extracts the sharpest frame from a
  recorded video. Calibration now uses the live camera, so it isn't used. Since
  moving into `setup/`, it looks for `setup/data/` instead of `data/`.
- **`calibration/empty*.npy` / `.json`** are calibrations in the old format.
  They don't work with the current scripts.
- **`data/`** holds images and videos from earlier experiments.
