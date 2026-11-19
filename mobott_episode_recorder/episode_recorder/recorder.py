
import threading
import time

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.serialization import serialize_message
from rclpy.qos import qos_profile_sensor_data

import rosbag2_py
from rosidl_runtime_py.utilities import get_message


class ROSDataRecorder:
    def __init__(self, topics: list[str]):
        self.topics = topics
        self.node = Node("mobott_episode_recorder")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.writer = None
        self.subscriptions = []
        self.thread = None
        self.recording = False
        self.episode = None


    def validate_topics(self, timeout=10.0) -> bool:
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            available = dict(
                self.node.get_topic_names_and_types()
            )

            missing = [
                topic for topic in self.topics
                if topic not in available
            ]

            if not missing:
                self.topic_types = {
                    topic: available[topic][0]
                    for topic in self.topics
                }

                print("[Recorder] Topics validated")
                return True

            time.sleep(0.25)

        print("[Recorder] Discovered topics:", available)
        raise RuntimeError(
            f"Missing ROS topics after {timeout}s: {missing}"
        )


    def start(self, episode):
        self.episode = episode
        self.writer = rosbag2_py.SequentialWriter()

        output_path = episode.directory / "data" / "rosbag"

        self.writer.open(
            rosbag2_py.StorageOptions(
                uri=str(output_path),
                storage_id="sqlite3",
            ),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )

        self.episode.message_counts = {
            topic: 0 for topic in self.topics
        }

        for topic, msg_type in self.topic_types.items():
            self.writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=topic,
                    type=msg_type,
                    serialization_format="cdr",
                )
            )

            msg_class = get_message(msg_type)
            subscription = self.node.create_subscription(
                msg_class,
                topic,
                lambda msg, t=topic: self._record(t, msg),
                qos_profile_sensor_data,
            )
            self.subscriptions.append(subscription)

        self.recording = True
        self.thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.thread.start()

        print(f"[Recorder] Recording to {output_path}")

    def _record(self, topic, msg):
        if not self.recording:
            return

        timestamp_ns = time.time_ns()

        self.writer.write(
            topic,
            serialize_message(msg),
            timestamp_ns,
        )

        self.episode.message_counts[topic] += 1

    def stop(self):
        if not self.recording and self.thread is None:
            return

        self.recording = False

        # Stop executor and wait for callbacks to finish.
        self.executor.shutdown()

        if self.thread is not None:
            self.thread.join()
            self.thread = None

        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)

        self.subscriptions.clear()

        # Release bag writer after callbacks have stopped.
        self.writer = None

        print("[Recorder] Stopped")


    def close(self):
        self.stop()

        self.executor.remove_node(self.node)
        self.node.destroy_node()
