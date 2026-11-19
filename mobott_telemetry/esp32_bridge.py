
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .transport import MockESP32Transport
from .transport import ESP32WiFiTransport

class ESP32BridgeNode(Node):
    def __init__(self, transport):
        super().__init__("esp32_bridge")

        self.transport = transport

        self.subscription = self.create_subscription(
            String,
            "/mobott/commands",
            self.on_command,
            10,
        )

        self.get_logger().info("ESP32 bridge ready")

    def on_command(self, msg):
        try:
            command = json.loads(msg.data)

            # Minimum command validation
            required = (
                "command_id",
                "linear",
                "angular",
            )

            if not all(k in command for k in required):
                raise ValueError("Missing command fields")

            self.transport.send(command)

            self.get_logger().info(
                f"Forwarded command {command['command_id']}"
            )

        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(
                f"Invalid command: {exc}"
            )

        except Exception as exc:
            self.get_logger().error(
                f"Transport failure: {exc}"
            )

    def close(self):
        self.transport.close()


def main():
    rclpy.init()

    bridge = ESP32BridgeNode(
        transport=ESP32WiFiTransport()
    )

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        bridge.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
