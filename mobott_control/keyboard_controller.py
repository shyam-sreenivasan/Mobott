
import json
import socket
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .keyboard_input import KeyboardInput


ESP_IP = "10.157.163.41"
ESP_PORT = 1234

CONTROL_PERIOD = 0.05  # 20 Hz


class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__("keyboard_controller")

        # ROS observability
        self.publisher = self.create_publisher(
            String,
            "/mobott/commands",
            10,
        )

        # self.subscription = self.create_subscription(
        #     String,
        #     "/mobott/localization",
        #     self.on_localization,
        #     10,
        # )

        self.keyboard = KeyboardInput()

        self.session_id = str(uuid.uuid4())
        self.command_id = 0
        self.latest_state_id = None

        # Direct TCP connection (original protocol)
        self.sock = socket.create_connection(
            (ESP_IP, ESP_PORT),
            timeout=3,
        )
        self.sock.setsockopt(
            socket.IPPROTO_TCP,
            socket.TCP_NODELAY,
            1,
        )

        self.get_logger().info(
            f"Connected to ESP32 at {ESP_IP}:{ESP_PORT}"
        )

        self.running = True

    def on_localization(self, msg):
        try:
            data = json.loads(msg.data)
            self.latest_state_id = data.get("message_id")
        except (ValueError, TypeError):
            pass

    def publish_ros_command(self, key, v, w):
        self.command_id += 1

        command = {
            "session_id": self.session_id,
            "command_id": self.command_id,
            "issued_at_ns": time.time_ns(),
            "control_mode": "MANUAL",
            "key": key,
            "linear": v,
            "angular": w,
            "source_state_id": self.latest_state_id,
        }

        msg = String()
        msg.data = json.dumps(command)

        self.publisher.publish(msg)

    def send_motor_command(self, v, w):
        message = f"{v},{w}\n"
        self.sock.sendall(message.encode())

    def run(self):
        self.get_logger().info(
            "Controller started: W/A/S/D"
        )

        try:
            while rclpy.ok() and self.running:
                loop_start = time.monotonic()

                # Process pending localization callbacks.
                rclpy.spin_once(self, timeout_sec=0.0)

                # Read keyboard (immediate velocity, no smoothing).
                key, v, w = self.keyboard.get_command()

                # Critical path: send command directly.
                self.send_motor_command(v, w)

                # Secondary path: publish for episode recording.
                self.publish_ros_command(key, v, w)

                elapsed = time.monotonic() - loop_start
                remaining = CONTROL_PERIOD - elapsed

                if remaining > 0:
                    time.sleep(remaining)

        finally:
            self.stop()

    def stop(self):
        self.running = False

        # Immediate zero-velocity command.
        try:
            self.send_motor_command(0.0, 0.0)
            self.publish_ros_command(
                "STOP", 0.0, 0.0
            )
        except (OSError, RuntimeError):
            pass

        self.keyboard.close()
        self.sock.close()

        self.get_logger().info("Controller stopped")


def main():
    rclpy.init()
    node = None

    try:
        node = KeyboardControllerNode()
        node.run()

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
