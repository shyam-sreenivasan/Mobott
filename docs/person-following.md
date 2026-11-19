# Assistive Mobile Robot Prototype with Mobott

A small differential-drive robotics platform for experimenting with **person
following, sensing, control, and assistive load-carrying concepts**.

The longer-term goal is to explore a mobile robot that can **follow a user,
carry personal loads, and eventually navigate safely at pedestrian speeds**.
The current Mobott platform is a low-cost prototype used to test the
underlying robotics stack before moving to a larger load-bearing platform.

## Current Capabilities

- ESP32-based differential-drive control over Wi-Fi or Serial
- Manual teleoperation from Ubuntu
- iPhone camera streaming into ROS 2
- ROS 2 person detection and tracking
- Person-following controller with configurable motion modes
- Target reacquisition and smoothing
- ROS `cmd_vel` interface with bridge to the physical robot
- rosbag recording and replay for repeatable testing
- Fail-safe stop behavior on communication loss
- Modular nodes for isolated testing of perception, tracking, control, and
  hardware communication

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

## Current Person-Following Pipeline

```text
iPhone camera
      ↓
ROS 2 image stream
      ↓
person detector / tracker
      ↓
target bearing + apparent distance
      ↓
follow controller
      ↓
/cmd_vel
      ↓
Mobott bridge
      ↓
ESP32 differential-drive robot
```

## Project Structure

```
robot/                  # Arduino firmware
  robot.ino             # main sketch
  motor.h / motor.cpp   # low-level motor driver
  motion.h / motion.cpp # differential drive + smoothing
  comm.h / comm.cpp     # communication (WiFi or Serial)
  secrets.h             # WiFi credentials (gitignored — see below)
  secrets.h.example     # template for secrets.h
keyboard-control.py     # teleoperation over Serial
keyboard-wifi-control.py# teleoperation over WiFi
roam.py                 # autonomous roaming with obstacle avoidance
find-robot.py           # scan the LAN for the robot's IP
requirements.txt        # Python dependencies
ros2/
  person_follower/      # ROS 2 package: perception, tracking, following
```

## Firmware Setup

### 1. WiFi credentials

```bash
cp robot/secrets.h.example robot/secrets.h
```

Edit `robot/secrets.h` and fill in your network name and password.

### 2. Choose communication mode

In [robot/comm.h](robot/comm.h), the default is WiFi:

```cpp
#define COMM_WIFI  // comment out to use Serial
```

Comment out that line to switch to Serial.

### 3. Flash the ESP32

Open the `robot/` folder in Arduino IDE and upload to the board. The IP address will print to the Serial Monitor after connecting.

## Teleoperation

Manual driving is the baseline for everything else: it verifies the
communication link and motor response before any autonomy is layered on.

### Install dependencies

```bash
pip install -r requirements.txt
```

### Keyboard control over WiFi

Set `ESP_IP` in `keyboard-wifi-control.py` to the robot's current address
(`./venv/bin/python find-robot.py` will find it), then:

```bash
sudo ./venv/bin/python keyboard-wifi-control.py
```

### Keyboard control over Serial

```bash
sudo python keyboard-control.py
```

> `sudo` is required on Linux because the `keyboard` library reads raw input events.

### Controls

| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn left |
| `D` | Turn right |

### Driving with a camera view

Useful for checking the camera framing, or for driving the robot from
another room. Two terminals, and the two halves are independent — the
viewer only subscribes to the camera topic, and the keyboard script talks
straight to the ESP32.

```bash
# terminal 1 — live camera, no detection
source /opt/ros/humble/setup.bash && source ~/sensorstream_ws/install/setup.bash
ros2 run person_follower camera_viewer

# terminal 2 — manual driving
sudo ./venv/bin/python keyboard-wifi-control.py
```

Conduit publishes 480x640 portrait frames even with the phone mounted in
landscape, so the view arrives on its side. Rotate it for display:

```bash
ros2 run person_follower camera_viewer --ros-args -p rotate:=90
```

Accepts `0`, `90`, `180`, `270`. Use `270` if `90` comes out upside down —
which way is up depends on the mount. This is display-only and does not
change the field of view, which stays narrow horizontally.

Requires Conduit to be publishing; see [Person Following](#person-following-ros-2)
for the camera setup.

> The `keyboard` library reads input globally, so W/A/S/D drive the robot
> regardless of which window has focus — including while the camera
> window is focused.

## Person Following (ROS 2)

`ros2/person_follower/` tracks a person in the iPhone camera stream and
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
                                        Mobott_bridge --TCP 1234--> robot
```

`/follow_cmd_vel` is kept distinct from `/cmd_vel` so an obstacle-safety
filter can be inserted between them later without rewiring anything else.

### Build

The package lives in this repo but is built from a colcon workspace, via
a symlink:

```bash
ln -s ~/workspace/Robotics-with-Mobott-Car/ros2/person_follower \
      ~/sensorstream_ws/src/person_follower

cd ~/sensorstream_ws
colcon build --packages-select person_follower
source install/setup.bash
```

Python dependencies (into the same interpreter ROS uses — a venv would
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
and joins the laptop's ROS graph directly over DDS — there is nothing to
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

**2. Power the robot and find its IP.** DHCP tends to move it:

```bash
./venv/bin/python find-robot.py
```

Or read it from the Arduino Serial Monitor (115200 baud) at boot.

**3. Launch the follower.** Conduit's topic names are the node defaults,
so no remapping is needed:

```bash
ros2 launch person_follower follow.launch.py esp_ip:=<robot-ip>
```

Step into frame. The log shows a five second countdown, then `Following
now`, and the robot starts following.

Stop with Ctrl-C — the bridge sends `0,0` on exit, and the firmware also
halts on its own after 300 ms of silence.

> **Wheels up for the first run.** There is no obstacle avoidance yet, so
> verify the robot turns toward you and drives forward when you step back
> before putting it on the floor.

### Running nodes individually

Useful for isolating a problem. Each in its own terminal:

```bash
# camera only, no detection — is Conduit working?
ros2 run person_follower camera_viewer

# detection only, no tracking or motion (Stage 1)
ros2 run person_follower person_detector

# tracking with persistent IDs, publishes /target_person (Stage 2)
ros2 run person_follower person_tracker

# turns /target_person into velocity commands, no robot needed
ros2 run person_follower follow_controller

# talks to the robot; nothing else drives it
ros2 run person_follower Mobott_bridge --ros-args -p esp_ip:=<robot-ip>
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
| `ros2 node list` empty but the app says it is publishing | Stale daemon cache — use `ros2 node list --no-daemon`. If still empty, check the domain ID matches on both sides. |
| `Could not reach car ... No route to host` | Robot is off, or DHCP moved it. Re-run `find-robot.py`. |
| Robot turns away from you instead of toward | `w_scale` sign — see Sign conventions below. |
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
| `esp_ip` | `10.76.211.120` | Robot's IP address — **usually needs overriding** |
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
ros2 run person_follower Mobott_bridge --ros-args \
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
so every angle elsewhere is dead-reckoned from `ms_per_degree` — assumed
to be 7.0, but it really depends on the surface, battery charge and
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

A warning that revolution times are inconsistent means peaks were missed
— usually too little texture in view, or the robot slipping.

### Pan tracking (rotate in place)

For a stationary use — the robot sits on a desk during a video call and
turns to keep whoever is in the room framed. Distance is ignored;
`linear.x` is always 0, so it only ever rotates.

```bash
ros2 run person_follower person_tracker        # perception
ros2 run person_follower pan_tracker           # rotate-only controller
ros2 run person_follower Mobott_bridge --ros-args \
  -p esp_ip:=<robot-ip> -r /cmd_vel:=/follow_cmd_vel
```

On startup it sweeps one full circle, sampling a view signature every
10°, building a map of the room. It then tracks the person; if they are
lost it sweeps in the direction they were last heading, pauses 10s, and
retries up to three times. After that it uses the room map to work out
which way it is facing, turns back to the starting heading, and waits —
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
`robot/motor.cpp`), so the robot cannot run one side slower than the other.
A blended `v,w` just drops the weaker side under the firmware deadband and
the robot spins instead of curving. `pivot` mode therefore turns in place or
drives straight, never both — the same approach `roam.py` uses.

### Sign conventions

`/cmd_vel` uses standard ROS signs (+x forward, +z left). The robot's own
protocol differs, so `Mobott_bridge` applies `w_scale = -1.0`: `roam.py`'s
`turn_for()` documents "direction is -1 for left, +1 for right". Forward
(`v`) is **not** inverted. Keep these flips in the bridge only, so every
other node can use normal ROS conventions.

## Known Limitations

Honest state of the prototype, since these shape what is worth building next:

- **Distance is estimated from bounding-box height**, not measured. It reacts
  to the subject's *apparent* size, so crouching, turning sideways, or partial
  occlusion all read as "moved away". The controller already accepts real
  metres, so this is a perception-side upgrade.
- **No obstacle avoidance.** The robot will drive into anything between it and
  the person. The ultrasonic sensor already streams distance on `/car/distance`
  and is unused so far.
- **Following is not yet smooth.** Turning is coarse, and reacquisition after a
  loss can produce a visible correction. Current evidence points at detection
  dropouts rather than the control law.
- **Single shared motor PWM** means the robot cannot turn and drive at the same
  time — see [Why `pivot` mode](#why-pivot-mode).

## Roadmap

| Stage | Status |
|---|---|
| Teleoperation, rosbag recording | Done |
| Person detection | Done |
| Person tracking with persistent identity | Done |
| Steering to keep the person centred | Done |
| Following at a set distance (box-height proxy) | Done |
| Distance from a real depth sensor | Next |
| Emergency obstacle stop | Planned |
| Local obstacle avoidance | Planned |
| Outdoor pedestrian-speed following | Planned |
| Load-bearing platform | Future |
