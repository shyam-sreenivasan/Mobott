"""Replay a Mobott ROS 2 episode, optionally exporting a synchronized MP4.

Examples:
    python -m analysis.episode_player experiments/.../data/rosbag
    python -m analysis.episode_player experiments/.../data/rosbag --save mobott_demo.mp4
"""

import argparse
import bisect
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, FFMpegWriter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

CAMERA_TOPIC = "/conduit/camera/wide/image_raw/compressed"
LOCALIZATION_TOPIC = "/mobott/localization"
COMMAND_TOPIC = "/mobott/commands"


def read_episode(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    required = (CAMERA_TOPIC, LOCALIZATION_TOPIC, COMMAND_TOPIC)
    for topic in required:
        if topic not in types:
            print(f"Warning: {topic} is absent from this bag")

    events = []
    message_types = {topic: get_message(types[topic]) for topic in required if topic in types}
    while reader.has_next():
        topic, payload, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        msg = deserialize_message(payload, message_types[topic])
        if topic == CAMERA_TOPIC:
            events.append((timestamp_ns, topic, bytes(msg.data)))
        else:
            try:
                data = json.loads(msg.data)
                if isinstance(data, dict):
                    events.append((timestamp_ns, topic, data))
            except (ValueError, TypeError):
                pass

    if not events:
        raise ValueError("No usable Mobott messages were found")
    events.sort(key=lambda e: e[0])
    t0 = events[0][0]
    result = {topic: [] for topic in required}
    for timestamp_ns, topic, value in events:
        result[topic].append(((timestamp_ns - t0) * 1e-9, value))
    result["duration"] = (events[-1][0] - t0) * 1e-9
    return result


def float_or_nan(value):
    try:
        return float(value) if value is not None else np.nan
    except (TypeError, ValueError):
        return np.nan


class EpisodePlayer:
    def __init__(self, data, fps=20, speed=1.0):
        self.data = data
        self.fps = fps
        self.speed = speed
        self.duration = data["duration"]
        self.camera = data[CAMERA_TOPIC]
        self.camera_times = [t for t, _ in self.camera]
        self.localization = data[LOCALIZATION_TOPIC]
        self.commands = data[COMMAND_TOPIC]
        self.paused = False
        self.frame_index = 0
        self.last_camera_index = -1
        self.placeholder = np.zeros((480, 640, 3), dtype=np.uint8)

        self.fig = plt.figure(figsize=(15, 8), constrained_layout=True)
        grid = self.fig.add_gridspec(3, 2, width_ratios=[1.35, 1.0])
        self.ax_camera = self.fig.add_subplot(grid[:, 0])
        self.ax_xy = self.fig.add_subplot(grid[0, 1])
        self.ax_theta = self.fig.add_subplot(grid[1, 1])
        self.ax_cmd = self.fig.add_subplot(grid[2, 1])
        self.image_artist = self.ax_camera.imshow(self.placeholder)
        self.ax_camera.axis("off")
        self.ax_camera.set_title("Camera (90° counterclockwise)")
        self.fig.suptitle("Mobott | Recorded episode replay", fontsize=15)

        loc_t, x, y, theta = [], [], [], []
        for t, record in self.localization:
            loc_t.append(t)
            if str(record.get("status", "OK")).upper() in ("OK", "SUCCESS"):
                x.append(float_or_nan(record.get("x")))
                y.append(float_or_nan(record.get("y")))
                theta.append(float_or_nan(record.get("theta")))
            else:
                x.append(np.nan)
                y.append(np.nan)
                theta.append(np.nan)

        self.loc_t = np.array(loc_t)
        self.x, self.y, self.theta = map(np.array, (x, y, theta))
        self.ax_xy.plot(self.x, self.y, color="tab:blue", alpha=0.35)
        (self.xy_cursor,) = self.ax_xy.plot([], [], "o", color="tab:orange", markersize=8)
        self.ax_xy.set(xlabel="X (recorded units)", ylabel="Y (recorded units)", title="Localization trajectory")
        self.ax_xy.grid(alpha=0.25)
        # Plot original angles; do not assume degrees versus radians.
        self.ax_theta.plot(self.loc_t, self.theta, color="tab:purple", linewidth=1.4)
        self.ax_theta.set(xlabel="Time (s)", ylabel="Heading (recorded units)", title="Robot heading")

        cmd_t = np.array([t for t, _ in self.commands])
        v = np.array([float_or_nan(d.get("linear")) for _, d in self.commands])
        w = np.array([float_or_nan(d.get("angular")) for _, d in self.commands])
        self.ax_cmd.step(cmd_t, v, where="post", label="Linear (command)")
        self.ax_cmd.step(cmd_t, w, where="post", label="Angular (command)")
        self.ax_cmd.set(xlabel="Time (s)", ylabel="Command value", title="Keyboard commands")
        self.ax_cmd.legend(loc="upper right", fontsize=8)
        self.ax_cmd.grid(alpha=0.25)

        self.time_lines = [ax.axvline(0, color="tab:red", alpha=0.8) for ax in (self.ax_theta, self.ax_cmd)]
        self.ax_theta.set_xlim(0, max(1.0, self.duration))
        self.ax_cmd.set_xlim(0, max(1.0, self.duration))
        for ax in (self.ax_xy, self.ax_theta, self.ax_cmd):
            ax.margins(0.08)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.animation = None

    def on_key(self, event):
        if event.key == " ":
            self.paused = not self.paused
        elif event.key == "r":
            self.frame_index = 0
            self.paused = False

    def update(self, frame_index):
        t = min(self.duration, frame_index / self.fps * self.speed)
        idx = bisect.bisect_right(self.camera_times, t) - 1
        if idx >= 0 and idx != self.last_camera_index:
            raw = np.frombuffer(self.camera[idx][1], dtype=np.uint8)
            decoded = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if decoded is not None:
                rotated = cv2.rotate(decoded, cv2.ROTATE_90_COUNTERCLOCKWISE)
                self.image_artist.set_data(cv2.cvtColor(rotated, cv2.COLOR_BGR2RGB))
                self.last_camera_index = idx
        loc_idx = np.searchsorted(self.loc_t, t, side="right") - 1
        if loc_idx >= 0 and np.isfinite(self.x[loc_idx]) and np.isfinite(self.y[loc_idx]):
            self.xy_cursor.set_data([self.x[loc_idx]], [self.y[loc_idx]])
        else:
            self.xy_cursor.set_data([], [])
        for line in self.time_lines:
            line.set_xdata([t, t])
        self.ax_camera.set_title(f"Camera (90° CCW) | {t:.2f} / {self.duration:.2f} s")
        return (self.image_artist, self.xy_cursor, *self.time_lines)

    def export(self, output_path):
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not FFMpegWriter.isAvailable():
            raise RuntimeError("ffmpeg not found. Install it with: sudo apt install ffmpeg")
        frames = max(1, int(np.ceil(self.duration * self.fps / self.speed)) + 1)
        print(f"Exporting {frames} frames to {output_path} at {self.fps} FPS...")
        writer = FFMpegWriter(fps=self.fps, bitrate=3000, codec="libx264", extra_args=["-pix_fmt", "yuv420p"])
        with writer.saving(self.fig, str(output_path), dpi=110):
            for i in range(frames):
                self.update(i)
                writer.grab_frame()
        print(f"Saved: {output_path}")

    def show(self):
        n_frames = max(1, int(np.ceil(self.duration * self.fps / self.speed)) + 1)
        def frame_sequence():
            while True:
                if not self.paused:
                    self.frame_index = (self.frame_index + 1) % n_frames
                yield self.frame_index
        self.update(0)
        self.animation = FuncAnimation(
            self.fig, self.update, frames=frame_sequence,
            interval=1000 / self.fps, blit=False, cache_frame_data=False,
        )
        print("Interactive controls: Space = pause/resume; R = restart")
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Mobott ROS 2 episode player / MP4 exporter")
    parser.add_argument("bag_path", help="Directory containing metadata.yaml / rosbag_0.db3")
    parser.add_argument("--save", metavar="OUTPUT.mp4", help="Export MP4 instead of opening the player")
    parser.add_argument("--fps", type=int, default=20, help="Playback/export FPS (default: 20)")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (default: 1.0)")
    args = parser.parse_args()
    if args.fps <= 0 or args.speed <= 0:
        parser.error("--fps and --speed must be positive")
    bag_path = Path(args.bag_path).expanduser().resolve()
    if not bag_path.exists():
        parser.error(f"Bag directory does not exist: {bag_path}")
    data = read_episode(bag_path)
    print(f"Loaded {data['duration']:.2f}s | camera={len(data[CAMERA_TOPIC])} | localization={len(data[LOCALIZATION_TOPIC])} | commands={len(data[COMMAND_TOPIC])}")
    player = EpisodePlayer(data, fps=args.fps, speed=args.speed)
    if args.save:
        player.export(args.save)
        plt.close(player.fig)
    else:
        player.show()

if __name__ == "__main__":
    main()
