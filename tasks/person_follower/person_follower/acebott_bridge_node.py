"""Bridge ROS Twist commands to the ACEBOTT car over TCP.

The car is not a ROS device: its firmware listens on port 1234 and reads
"v,w\\n" lines (see commRead() in robot/comm.cpp). This node is the only
place that knows that, so everything upstream can speak plain /cmd_vel.

It also republishes the car's telemetry lines ("T,millis,distance,...")
as /car/distance, which Stage 6 will need for the safety stop.

Run with:
  ros2 run person_follower acebott_bridge --ros-args -p esp_ip:=10.76.211.120
"""

import socket
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

CMD_TOPIC = "/cmd_vel"
DISTANCE_TOPIC = "/car/distance"


class AcebottBridge(Node):
    def __init__(self):
        super().__init__("acebott_bridge")

        # Default matches ESP_IP in roam.py; override when DHCP moves it.
        self.declare_parameter("esp_ip", "10.76.211.120")
        self.declare_parameter("esp_port", 1234)
        self.declare_parameter("rate", 20.0)
        # Scales normalised Twist values onto the car's -1..1 range. The
        # firmware deadband is 0.05 and it saturates at 1.0.
        #
        # v_scale +1.0: same polarity roam.py uses, where pressing W sends
        # "1.0,0.0" and the car drives forward.
        #
        # w_scale -1.0: this car turns the opposite way to the ROS
        # convention. roam.py is explicit about it -- turn_for() documents
        # "direction is -1 for left, +1 for right", and pressing A (left)
        # sends w = -1.0. ROS says +angular.z is left, so the sign has to
        # flip here. Keeping the flip in the bridge lets every upstream
        # node use standard ROS signs.
        self.declare_parameter("v_scale", 1.0)
        self.declare_parameter("w_scale", -1.0)
        self.declare_parameter("cmd_timeout", 0.4)
        self.declare_parameter("retry_interval", 2.0)

        self.ip = self.get_parameter("esp_ip").value
        self.port = self.get_parameter("esp_port").value
        self.v_scale = self.get_parameter("v_scale").value
        self.w_scale = self.get_parameter("w_scale").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.retry_interval = self.get_parameter("retry_interval").value

        self.sock = None
        self.v = 0.0
        self.w = 0.0
        self.last_cmd = None
        self.lock = threading.Lock()
        self._last_attempt = 0.0
        self._last_error = None

        self.create_subscription(Twist, CMD_TOPIC, self.on_cmd, 10)
        self.dist_pub = self.create_publisher(Float32, DISTANCE_TOPIC, 10)

        self._connect()

        rate = self.get_parameter("rate").value
        self.create_timer(1.0 / rate, self.tick)

        self.rx = threading.Thread(target=self._read_telemetry, daemon=True)
        self.rx.start()

    def _connect(self):
        # The firmware serves exactly one client (commRead() only calls
        # server.available() when it has none), so a second connection
        # steals the slot from the first and both end up broken. Always
        # tear the old socket down before dialling again.
        self._close()

        # Do not hammer a car that is off or unreachable: each failed
        # attempt otherwise blocks tick() for the full connect timeout.
        now = time.monotonic()
        if now - self._last_attempt < self.retry_interval:
            return
        self._last_attempt = now

        try:
            s = socket.create_connection((self.ip, self.port), timeout=3.0)
            s.settimeout(0.5)
            self.sock = s
            self.get_logger().info(f"Connected to car at {self.ip}:{self.port}")
        except OSError as e:
            self.sock = None
            # Repeated identical errors are noise once the cause is known.
            if str(e) != self._last_error:
                self._last_error = str(e)
                self.get_logger().error(
                    f"Could not reach car at {self.ip}:{self.port} ({e}). "
                    "Check it is powered on and the IP is right; retrying."
                )

    def on_cmd(self, msg):
        with self.lock:
            self.v = msg.linear.x * self.v_scale
            self.w = msg.angular.z * self.w_scale
            self.last_cmd = self.get_clock().now()

    def tick(self):
        if self.sock is None:
            self._connect()
            return

        with self.lock:
            v, w, last = self.v, self.w, self.last_cmd

        # Nothing recent from the controller: send explicit zeros. The car
        # would stop by itself after 300ms anyway, but saying so keeps the
        # link warm and the intent unambiguous.
        if last is None or (self.get_clock().now() - last).nanoseconds / 1e9 > self.cmd_timeout:
            v = w = 0.0

        try:
            self.sock.sendall(f"{v:.3f},{w:.3f}\n".encode())
        except OSError as e:
            self.get_logger().warn(f"Send failed ({e}); reconnecting")
            self._close()

    def _read_telemetry(self):
        """Parse "T,millis,distance_cm,..." lines into /car/distance."""
        buf = b""
        while rclpy.ok():
            sock = self.sock
            if sock is None:
                # Nothing to read from yet. Sleep rather than spin: this
                # thread would otherwise burn a core and race _connect().
                time.sleep(0.1)
                continue
            try:
                chunk = sock.recv(256)
            except socket.timeout:
                continue
            except OSError:
                time.sleep(0.1)
                continue
            if not chunk:
                # Clean EOF: the car closed the connection. Let tick()
                # rebuild it instead of reading a dead socket in a loop.
                time.sleep(0.1)
                continue

            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                parts = line.decode(errors="ignore").strip().split(",")
                if len(parts) >= 3 and parts[0] == "T":
                    try:
                        msg = Float32()
                        msg.data = float(parts[2])
                        self.dist_pub.publish(msg)
                    except ValueError:
                        pass

    def _close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def stop_car(self):
        """Best-effort halt so the car does not coast when this node exits."""
        if self.sock is not None:
            try:
                self.sock.sendall(b"0,0\n")
            except OSError:
                pass


def main():
    rclpy.init()
    node = AcebottBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_car()
        node._close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
