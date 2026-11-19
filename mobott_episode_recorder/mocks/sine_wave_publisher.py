
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SineWavePublisher(Node):
    def __init__(self):
        super().__init__("sine_wave_publisher")

        self.publisher = self.create_publisher(
            Float64,
            "/test/sine_wave",
            10,
        )

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.publish_wave)

    def publish_wave(self):
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        msg = Float64()
        msg.data = math.sin(2 * math.pi * 0.5 * elapsed)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SineWavePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
x