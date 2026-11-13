# MoBoTT - Mobile Robot Platform

A small differential-drive robotics platform for experimenting with **person
following, localization, sensing, control, and assistive load-carrying
concepts**.

The longer-term goal is to explore a mobile robot that can **follow a user,
carry personal loads, and eventually navigate safely at pedestrian speeds**.
The current MoBoTT platform is a low-cost prototype used to test the
underlying robotics stack before moving to a larger load-bearing platform.

## Current Capabilities

- ESP32-based differential-drive control over Wi-Fi or Serial
- Automatic Wi-Fi reconnect on the robot, with no reset or re-flash needed
- Fail-safe stop when commands stop arriving (300 ms watchdog)
- Manual teleoperation from Ubuntu (keyboard or PS5 controller)
- iPhone camera streaming into ROS 2
- ROS 2 person detection, tracking, and following with configurable motion modes
- Target reacquisition and smoothing
- ROS `cmd_vel` interface with bridge to the physical robot
- rosbag recording and replay for repeatable testing
- Tabletop workspace calibration from ArUco corner markers
- Live overhead localization of an ArUco marker (position in cm and heading)
- Modular nodes and scripts for isolated testing of perception, tracking,
  control, and hardware communication

## Assistive Mobility Motivation

Carrying bags, groceries, luggage, or other everyday loads can become
difficult when upper-limb strength, range of motion, or mobility is limited.

The broader question behind this project is:

> Can a small mobile robot provide useful physical assistance by carrying
> loads and staying with the user, while requiring minimal interaction?

Rather than beginning with a full-scale product, this project uses a small
robot to study the core technical pieces:

1. reliably detect and track a person,
2. maintain a comfortable following distance,
3. respond safely to temporary tracking loss,
4. integrate perception and motion control on real hardware,
5. eventually add obstacle avoidance, localization, and pedestrian-speed
   navigation.

## Repository Structure

```text
ESP32/                      # Arduino firmware for the robot
  robot.ino                 # main sketch: command loop + safety stop
  comm.h / comm.cpp         # Wi-Fi (TCP port 1234) or Serial commands
  motion.h / motion.cpp     # v,w -> wheel directions and speed
  motor.h / motor.cpp       # shift register + shared PWM motor output
  secrets.h                 # Wi-Fi credentials (gitignored, see below)
  secrets.h.example         # template for secrets.h
manual-control/
  keyboard-wifi-control.py  # keyboard teleoperation over Wi-Fi
  keyboard-control.py       # keyboard teleoperation over Serial (USB)
  joystick_wifi_control.py  # PS5 (DualSense) teleoperation over Wi-Fi
tasks/
  person_follower/          # ROS 2 package: perception, tracking, following
  autonomous_nav/           # overhead camera: workspace calibration + localization
  mobile_sensor/            # phone sensor bridge for ROS 2 (separate git repo)
requirements.txt            # Python dependencies for manual control
```

## Firmware Setup

### 1. Wi-Fi credentials

```bash
cp ESP32/secrets.h.example ESP32/secrets.h
```

Edit `ESP32/secrets.h` and fill in your network name and password. The file
is gitignored, so your password stays out of the repository.

### 2. Choose communication mode

In [ESP32/comm.h](ESP32/comm.h), the default is Wi-Fi:

```cpp
#define COMM_WIFI  // comment out to use Serial
```

Comment out that line to switch to Serial (USB).

### 3. Flash the ESP32

Open the `ESP32/` folder in the Arduino IDE and upload to the board. The
program is stored in the ESP32's flash memory, so it only needs to be
uploaded again when the code or the Wi-Fi credentials change.

### How the firmware behaves

- **Wi-Fi.** On power-up the robot joins the network from `secrets.h`. If the
  network isn't available, it keeps retrying every 5 seconds in the
  background and connects as soon as Wi-Fi appears. The IP address prints to
  the Serial Monitor (115200 baud) once connected. Wi-Fi power saving is
  turned off to keep command latency low.
- **Protocol.** The robot listens on TCP port 1234 for text lines `v,w`,
  where `v` (forward) and `w` (turn) are between `-1.0` and `1.0`. For
  example, `1.0,0.0` drives forward at full speed.
- **Safety stop.** If no command arrives for 300 ms, the robot stops. Every
  controller in this repo keeps sending commands (zeros when idle) to stay
  inside this window.
- **Motion.** All four motors share one PWM speed signal, so the robot either
  drives straight or pivots in place; it can't curve. See
  [Why `pivot` mode](#why-pivot-mode).

## Teleoperation

Manual driving is the baseline for everything else: it verifies the
communication link and motor response before any autonomy is layered on.

### Install dependencies

```bash
pip install -r requirements.txt
```

### Keyboard control over Wi-Fi

Set `ESP_IP` in `manual-control/keyboard-wifi-control.py` to the robot's
current address (shown in the Serial Monitor at boot), then:

```bash
sudo ./venv/bin/python manual-control/keyboard-wifi-control.py
```

### Keyboard control over Serial

```bash
sudo python manual-control/keyboard-control.py
```

> `sudo` is required on Linux because the `keyboard` library reads raw input events.

### Controls

| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn left |
| `D` | Turn right |

### PS5 controller over Wi-Fi

```bash
python3 manual-control/joystick_wifi_control.py --ip <robot-ip>
```

Left stick drives (forward/back) or pivots (left/right), whichever is pushed
further. L1 / R1 change the top speed, Cross brakes, and Options quits. No
`sudo` is needed.

> **Don't use the right stick.** It sends sensor-servo commands (`S,angle`),
> and the current firmware no longer has a servo. See
> [Known Limitations](#known-limitations).

### Driving with a camera view

Useful for checking the camera framing, or for driving the robot from
another room. Two terminals, and the two halves are independent: the
viewer only subscribes to the camera topic, and the keyboard script talks
straight to the ESP32.

```bash
# terminal 1: live camera, no detection
source /opt/ros/humble/setup.bash && source ~/sensorstream_ws/install/setup.bash
ros2 run person_follower camera_viewer

# terminal 2: manual driving
sudo ./venv/bin/python manual-control/keyboard-wifi-control.py
```

Conduit publishes 480x640 portrait frames even with the phone mounted in
landscape, so the view arrives on its side. Rotate it for display:

```bash
ros2 run person_follower camera_viewer --ros-args -p rotate:=90
```

Accepts `0`, `90`, `180`, `270`. Use `270` if `90` comes out upside down;
which way is up depends on the mount. This is display-only and does not
change the field of view, which stays narrow horizontally.

Requires Conduit to be publishing; see [Person Following](#person-following-ros-2)
for the camera setup.

> The `keyboard` library reads input globally, so W/A/S/D drive the robot
> regardless of which window has focus, including while the camera
> window is focused.

## Person Following (ROS 2)

`tasks/person_follower/` tracks a person in the iPhone camera stream and
drives the robot to follow them. Requires ROS 2 Humble and the Conduit
iPhone bridge publishing `/conduit/camera/front/image_raw/compressed`.

### Nodes and topics

```
/conduit/camera/... -> person_tracker -> /target_person
                                              |            (bearing,
                                              |             distance,
                                              |             confidence)
                                       follow_controller
                                              |
                                       /follow_cmd_vel
                                              |
                                        MoBoTT_bridge --TCP 1234--> robot
```

`/follow_cmd_vel` is kept distinct from `/cmd_vel` so an obstacle-safety
filter can be inserted between them later without rewiring anything else.

### Build

The package lives in this repo but is built from a colcon workspace, via
a symlink:

```bash
ln -s ~/workspace/Robotics-with-MoBoTT-Car/tasks/person_follower \
      ~/sensorstream_ws/src/person_follower

cd ~/sensorstream_ws
colcon build --packages-select person_follower
source install/setup.bash
```

Python dependencies (into the same interpreter ROS uses; a venv would
hide `rclpy`):

```bash
pip install --user ultralytics "numpy<2" "setuptools<80" "packaging>=23" lap
pip uninstall -y opencv-python   # conflicts with the ROS-shipped cv2
```

### Run

Every terminal needs both workspaces sourced:

```bash
source /opt/ros/humble/setup.bash
source ~/sensorstream_ws/install/setup.bash
```

**1. Start the camera stream.** Conduit runs a ROS 2 stack on the iPhone
and joins the laptop's ROS graph directly over DDS; there is nothing to
launch on the laptop for the camera.

In the Conduit app:

- add the laptop as a **unicast peer**: its IP, port **7400**
- set **domain ID `0`** to match the laptop (`ROS_DOMAIN_ID` unset = 0)
- start publishing

Find the laptop's IP with `hostname -I`. Both devices must be on the same
network; a domain ID mismatch makes the two sides invisible to each other
even when everything else is correct.

Confirm the stream before going further:

```bash
ros2 topic list
ros2 topic hz /conduit/camera/front/image_raw/compressed
```

Expect `/conduit/camera/front/image_raw/compressed`,
`/conduit/camera/front/camera_info` and `/conduit/imu`, at roughly 30 Hz.

If `ros2 node list` looks empty while the app says it is publishing, query
DDS directly instead of the cached graph:

```bash
ros2 node list --no-daemon
```

**2. Power the robot and find its IP.** DHCP tends to move it. Read it from
the Arduino Serial Monitor (115200 baud) at boot, or from your router's list
of connected devices.

**3. Launch the follower.** Conduit's topic names are the node defaults,
so no remapping is needed:

```bash
ros2 launch person_follower follow.launch.py esp_ip:=<robot-ip>
```

Step into frame. The log shows a five second countdown, then `Following
now`, and the robot starts following.

Stop with Ctrl-C. The bridge sends `0,0` on exit, and the firmware also
halts on its own after 300 ms of silence.

> **Wheels up for the first run.** There is no obstacle avoidance yet, so
> verify the robot turns toward you and drives forward when you step back
> before putting it on the floor.

### Running nodes individually

Useful for isolating a problem. Each in its own terminal:

```bash
# camera only, no detection: is Conduit working?
ros2 run person_follower camera_viewer

# detection only, no tracking or motion (Stage 1)
ros2 run person_follower person_detector

# tracking with persistent IDs, publishes /target_person (Stage 2)
ros2 run person_follower person_tracker

# turns /target_person into velocity commands, no robot needed
ros2 run person_follower follow_controller

# talks to the robot; nothing else drives it
ros2 run person_follower MoBoTT_bridge --ros-args -p esp_ip:=<robot-ip>
```

The tracker and detector open an OpenCV window: green box is the locked
target, orange boxes are other people. **Q** or **Esc** quits, **T**
forces a re-lock onto the nearest person.

### Testing without the robot

The controller can be verified with no hardware at all:

```bash
ros2 run person_follower follow_controller
ros2 topic echo /follow_cmd_vel
```

Walk around in front of the camera and watch the numbers. Person to your
left gives `angular.z > 0`; too far away gives `linear.x > 0`.

You can also replay a recorded session instead of using the live camera:

```bash
ros2 bag play ~/rosbags/<bag-dir>
```

### Recording a run

```bash
ros2 bag record /conduit/camera/front/image_raw/compressed \
                /conduit/camera/front/camera_info \
                /target_person /follow_cmd_vel /cmd_vel
```

### Troubleshooting

| Symptom | Cause |
|---|---|
| `camera_viewer` hangs on "Waiting for frames" | Conduit is not publishing. `ros2 topic info <topic>` will show `Publisher count: 0`. |
| `ros2 node list` empty but the app says it is publishing | Stale daemon cache; use `ros2 node list --no-daemon`. If still empty, check the domain ID matches on both sides. |
| `Could not reach car ... No route to host` | Robot is off, or DHCP moved it. Check its IP in the Serial Monitor. |
| Robot turns away from you instead of toward | `w_scale` sign; see Sign conventions below. |
| Robot reverses when it should approach | `target_box_h` is below your actual box height. Echo `/target_person` and raise it. |
| Robot drives but never turns, or vice versa | Expected in `pivot` mode: it does one or the other, never both. |
| Tracker keeps re-locking onto new IDs | Detection is dropping out. Try `model:=yolov8s.pt` or a lower `conf`. |

If a node behaves as though your code changes did nothing, check for a
stale process from an earlier run:

```bash
ros2 node list
pkill -f person_tracker
```

### Key parameters

| Argument | Default | Meaning |
|---|---|---|
| `esp_ip` | `10.76.211.120` | Robot's IP address; **usually needs overriding** |
| `target_box_h` | `500.0` | Follow distance, as person box height in px. **Higher = follows closer** |
| `start_delay` | `5.0` | Seconds after first detection before moving. `0` disables |
| `drive_mode` | `pivot` | `pivot`, `pid`, or `blended` (see below) |
| `linear_enabled` | `true` | `false` = rotate in place only, no forward motion |

Turning:

| Argument | Default | Meaning |
|---|---|---|
| `turn_speed` | `0.6` | Turn magnitude at `full_turn_deg` of error |
| `min_turn` | `0.35` | Turn magnitude just outside the deadzone. Must clear the firmware's 0.05 deadband |
| `full_turn_deg` | `40.0` | Bearing error at which the turn reaches `turn_speed` |
| `turn_slew` | `1.2` | Max change in turn command per second; lower is smoother |
| `pid_kp` / `pid_ki` / `pid_kd` | `1.1` / `0.15` / `0.35` | Gains, used only when `drive_mode:=pid` |

Tracking:

| Argument | Default | Meaning |
|---|---|---|
| `smoothing` | `0.6` | EMA on bearing and box height. Higher = smoother but laggier |
| `confirm_frames` | `5` | Frames a re-acquired target must persist before the robot acts on it |
| `reacquire_secs` | `3.0` | How long to stay locked on a target that has vanished |
| `distance_deadzone_px` | `45.0` | Tolerance around `target_box_h` before moving |

Detector settings are node parameters rather than launch arguments:

```bash
ros2 run person_follower person_tracker --ros-args \
  -p model:=yolov8s.pt -p conf:=0.3
```

To calibrate the follow distance: stand where you want the robot to hold
station, run `ros2 topic echo /target_person`, and use the magnitude of
`y` as `target_box_h`.

### Room calibration

The first testable piece, and a prerequisite for the rest. Run it on its
own; it reports a verdict and exits.

```bash
# terminal 1
ros2 run person_follower MoBoTT_bridge --ros-args \
  -p esp_ip:=<robot-ip> -r /cmd_vel:=/follow_cmd_vel

# terminal 2
ros2 run person_follower calibrate_room
```

Three phases:

1. **Spin** ~30s, counting how often the camera view comes back round.
   That measures how fast the robot actually turns on this surface.
2. **Sweep** one accurate circle at the measured rate, sampling the room
   into 36 heading bins.
3. **Return** home by matching the live view against that map.

Phase 1 is the point of the exercise. The robot has no encoders or IMU,
so every angle elsewhere is dead-reckoned from `ms_per_degree`, assumed
to be 7.0 but really dependent on the surface, battery charge and
motor wear. Counting revolutions with the camera measures it instead. In
testing, a robot actually turning at 8.5 ms/deg was measured at 8.50
against an assumed 7.0, and the corrected sweep returned home with a
view match of 0.91 instead of 0.67.

| Argument | Default | Meaning |
|---|---|---|
| `spin_secs` | `30.0` | Phase 1 duration; longer gives more revolutions |
| `measure_rate` | `true` | `false` skips phase 1 and trusts `ms_per_degree` |
| `bins` | `36` | Heading samples per circle |
| `ms_per_degree` | `7.0` | Starting guess, overwritten by phase 1 |
| `turn_speed` | `0.5` | Rotation command used throughout |

A warning that revolution times are inconsistent means peaks were missed,
usually too little texture in view, or the robot slipping.

### Pan tracking (rotate in place)

For a stationary use: the robot sits on a desk during a video call and
turns to keep whoever is in the room framed. Distance is ignored;
`linear.x` is always 0, so it only ever rotates.

```bash
ros2 run person_follower person_tracker        # perception
ros2 run person_follower pan_tracker           # rotate-only controller
ros2 run person_follower MoBoTT_bridge --ros-args \
  -p esp_ip:=<robot-ip> -r /cmd_vel:=/follow_cmd_vel
```

On startup it sweeps one full circle, sampling a view signature every
10°, building a map of the room. It then tracks the person; if they are
lost it sweeps in the direction they were last heading, pauses 10s, and
retries up to three times. After that it uses the room map to work out
which way it is facing, turns back to the starting heading, and waits,
resuming the moment anyone appears.

The room map matters because the robot has no encoders or IMU. Without
it, "turn back to home" is dead-reckoned from turn timing and drifts.
Matching the live camera view against the map is closed-loop, so homing
stays accurate however long the session runs. In a featureless room the
match is ambiguous and the node says so rather than guessing, falling
back to dead reckoning.

| Argument | Default | Meaning |
|---|---|---|
| `calibrate` | `true` | Sweep and map the room at startup |
| `calibration_bins` | `36` | Samples per circle; 72 halves the heading error |
| `search_attempts` | `3` | 360° sweeps before giving up |
| `cooldown_secs` | `10.0` | Pause between sweeps |
| `lost_after` | `1.5` | Seconds before the person counts as lost |
| `visual_home` | `true` | `false` uses dead reckoning only |

### Why `pivot` mode

All four motors share a single PWM line (see `motorMove()` in
[ESP32/motor.cpp](ESP32/motor.cpp)), so the robot cannot run one side slower
than the other. A blended `v,w` just drops the weaker side under the firmware
deadband and the robot spins instead of curving. `pivot` mode therefore turns
in place or drives straight, never both.

### Sign conventions

`/cmd_vel` uses standard ROS signs (+x forward, +z left). The robot's own
protocol differs: on this robot a positive `w` turns right (the keyboard
scripts send `w = +1` for `D`), so `MoBoTT_bridge` applies
`w_scale = -1.0`. Forward (`v`) is **not** inverted. Keep these flips in the bridge only, so every other
node can use normal ROS conventions.

## Autonomous Navigation (Tabletop)

`tasks/autonomous_nav/` uses a fixed overhead camera to localize the robot
on a tabletop workspace:

1. **Calibrate** the workspace from four ArUco corner markers (IDs 1–4),
   producing a camera-pixel → table-cm transformation.
2. **Localize** a robot marker (ID 0) live, showing its position in cm and
   its heading in degrees in both the camera view and a bird's-eye view.

Everything runs from one menu:

```bash
cd tasks/autonomous_nav
python3 main.py
```

Setup, marker placement, coordinate system, and details are in
[tasks/autonomous_nav/README.md](tasks/autonomous_nav/README.md).

## Mobile Sensor Bridge

`tasks/mobile_sensor/` is the
[ros2-mobile-sensor-bridge](https://github.com/VedantC2307/ros2-mobile-sensor-bridge)
project: it streams a phone's camera, IMU, GPS, and speech into ROS 2 from a
web browser. See its own [README](tasks/mobile_sensor/README.md). It is a
separate git repository inside this folder, so it is not included when this
repository is cloned.

## Known Limitations

Honest state of the prototype, since these shape what is worth building next:

- **Distance is estimated from bounding-box height**, not measured. It reacts
  to the subject's *apparent* size, so crouching, turning sideways, or partial
  occlusion all read as "moved away". The controller already accepts real
  metres, so this is a perception-side upgrade.
- **No obstacle avoidance.** The robot will drive into anything between it and
  the person.
- **No on-robot sensors.** The ultrasonic sensor and servo were removed from
  the firmware, so the robot no longer sends telemetry. `MoBoTT_bridge` still
  listens for it and `/car/distance` stays empty, and the joystick's right
  stick sends servo commands the firmware no longer understands.
- **Following is not yet smooth.** Turning is coarse, and reacquisition after a
  loss can produce a visible correction. Current evidence points at detection
  dropouts rather than the control law.
- **Single shared motor PWM** means the robot cannot turn and drive at the same
  time; see [Why `pivot` mode](#why-pivot-mode).
- **Tabletop localization assumes a flat marker.** A marker raised on the
  robot appears shifted away from the camera; see the
  [autonomous_nav README](tasks/autonomous_nav/README.md#limitations).

## Roadmap

| Stage | Status |
|---|---|
| Teleoperation, rosbag recording | Done |
| Person detection | Done |
| Person tracking with persistent identity | Done |
| Steering to keep the person centred | Done |
| Following at a set distance (box-height proxy) | Done |
| Tabletop workspace calibration (ArUco corners) | Done |
| Live overhead localization (position + heading) | Done |
| Click-to-goal path planning on the tabletop | In progress |
| Distance from a real depth sensor | Next |
| Emergency obstacle stop | Planned |
| Local obstacle avoidance | Planned |
| Outdoor pedestrian-speed following | Planned |
| Load-bearing platform | Future |
