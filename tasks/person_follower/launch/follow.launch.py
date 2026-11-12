"""Stage 3: tracker -> controller -> car.

  person_tracker  --/target_person-->  follow_controller
                                              |
                                      /follow_cmd_vel
                                              |
                                   (relay: Stage 6 replaces this
                                    with the obstacle safety filter)
                                              |
                                          /cmd_vel
                                              |
                                       acebott_bridge --TCP--> car

Run with:
  ros2 launch person_follower follow.launch.py esp_ip:=10.76.211.120
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    esp_ip = LaunchConfiguration("esp_ip")
    kp = LaunchConfiguration("kp")
    max_angular = LaunchConfiguration("max_angular")
    linear_enabled = LaunchConfiguration("linear_enabled")
    max_linear = LaunchConfiguration("max_linear")
    target_box_h = LaunchConfiguration("target_box_h")

    return LaunchDescription([
        DeclareLaunchArgument("esp_ip", default_value="10.76.211.120"),
        DeclareLaunchArgument("kp", default_value="1.2"),
        DeclareLaunchArgument("max_angular", default_value="0.6"),
        # Set false to fall back to Stage 3 rotate-in-place.
        DeclareLaunchArgument("linear_enabled", default_value="true"),
        # Deliberately below the node default of 0.5 for first floor runs.
        DeclareLaunchArgument("max_linear", default_value="0.30"),
        DeclareLaunchArgument("target_box_h", default_value="500.0"),
        # pivot   = turn in place, then drive straight (this car's shared
        #           PWM cannot curve; matches how roam.py drives it)
        # blended = simultaneous turn and drive, for per-side speed control
        # pid     = PID on bearing, smoother but needs tuning
        DeclareLaunchArgument("drive_mode", default_value="pivot"),
        DeclareLaunchArgument("pid_kp", default_value="1.1"),
        DeclareLaunchArgument("pid_ki", default_value="0.15"),
        DeclareLaunchArgument("pid_kd", default_value="0.35"),
        # Seconds between the first detection and the first movement.
        # One-shot: later losses of the target do not re-arm it.
        DeclareLaunchArgument("start_delay", default_value="5.0"),
        # Smoothing on the tracked bearing (0=off, 0.9=heavy).
        DeclareLaunchArgument("smoothing", default_value="0.6"),
        # Turn magnitude: ramps from min_turn at the deadzone edge to
        # turn_speed at full_turn_deg of bearing error.
        DeclareLaunchArgument("turn_speed", default_value="0.6"),
        DeclareLaunchArgument("min_turn", default_value="0.35"),
        DeclareLaunchArgument("full_turn_deg", default_value="40.0"),
        # Max change in turn command per second; eases into corrections.
        DeclareLaunchArgument("turn_slew", default_value="1.2"),
        # How long the tracker holds a lock through an occlusion.
        DeclareLaunchArgument("reacquire_secs", default_value="3.0"),
        # Frames a re-acquired target must persist before the car acts on
        # it, so reappearing at the frame edge does not cause a lurch.
        DeclareLaunchArgument("confirm_frames", default_value="5"),
        # Tolerance around target_box_h, in pixels. Widen it if the car
        # hunts back and forth instead of settling.
        DeclareLaunchArgument("distance_deadzone_px", default_value="45.0"),
        # v: same polarity as roam.py (W sends "1.0,0.0" -> forward).
        # w: negated -- roam.py's turn_for() documents "-1 for left,
        #    +1 for right", the opposite of the ROS +z=left convention.
        DeclareLaunchArgument("v_scale", default_value="1.0"),
        DeclareLaunchArgument("w_scale", default_value="-1.0"),

        Node(
            package="person_follower", executable="person_tracker",
            name="person_tracker", output="screen",
            parameters=[{
                "smoothing": LaunchConfiguration("smoothing"),
                "reacquire_secs": LaunchConfiguration("reacquire_secs"),
                "confirm_frames": LaunchConfiguration("confirm_frames"),
            }],
        ),
        Node(
            package="person_follower", executable="follow_controller",
            name="follow_controller", output="screen",
            parameters=[{
                "kp": kp,
                "max_angular": max_angular,
                "linear_enabled": linear_enabled,
                "max_linear": max_linear,
                "target_box_h": target_box_h,
                "drive_mode": LaunchConfiguration("drive_mode"),
                "start_delay": LaunchConfiguration("start_delay"),
                "turn_speed": LaunchConfiguration("turn_speed"),
                "min_turn": LaunchConfiguration("min_turn"),
                "full_turn_deg": LaunchConfiguration("full_turn_deg"),
                "turn_slew": LaunchConfiguration("turn_slew"),
                "pid_kp": LaunchConfiguration("pid_kp"),
                "pid_ki": LaunchConfiguration("pid_ki"),
                "pid_kd": LaunchConfiguration("pid_kd"),
                "distance_deadzone_px": LaunchConfiguration("distance_deadzone_px"),
            }],
        ),
        Node(
            package="person_follower", executable="acebott_bridge",
            name="acebott_bridge", output="screen",
            parameters=[{
                "esp_ip": esp_ip,
                "v_scale": LaunchConfiguration("v_scale"),
                "w_scale": LaunchConfiguration("w_scale"),
            }],
            # Stage 3 has no safety filter yet, so the controller's output
            # is remapped straight onto the bridge's input. Stage 6 removes
            # this line and inserts obstacle_safety in the gap.
            remappings=[("/cmd_vel", "/follow_cmd_vel")],
        ),
    ])
