"""Drive the ACEBOTT car with a PS5 (DualSense) controller over WiFi.

Controls
  Left stick      push forward/back to drive, left/right to turn in place
  Right stick X   aim the ultrasonic sensor head
  L1 / R1         lower / raise the top speed (3 levels)
  Cross (X)       hold to brake
  Options         quit

The car has a single PWM line for all four motors, so it cannot curve:
it either drives straight or pivots. The stick is mapped the same way --
whichever axis is pushed further wins -- instead of sending a blended v,w
that the firmware would turn into a surprise pivot.

Run with:  python3 joystick_wifi_control.py --ip 10.0.0.18
(no sudo needed, unlike the keyboard scripts)

Verify the axis/button numbers below with checkcontrooler.py first; they
are the usual Linux (hid-playstation) layout but can differ by driver.
"""

import argparse
import math
import os
import socket
import threading
import time

# Keep receiving controller input even if another window (e.g. the camera
# viewer) has focus.
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
import pygame  # noqa: E402  (must come after the env var)

DEFAULT_IP = "10.0.0.18"
PORT = 1234
SEND_HZ = 20                 # firmware watchdog is 300 ms; 20 Hz is plenty

# ---------------- controller layout (check with checkcontrooler.py) -------
AXIS_LX, AXIS_LY, AXIS_RX = 0, 1, 3
BTN_CROSS, BTN_L1, BTN_R1, BTN_OPTIONS = 0, 4, 5, 9

# ---------------- tuning ----------------
DEADZONE = 0.15
# Lowest command that actually moves the car on the floor. Below this the
# motors just buzz. Raise it if small stick pushes do nothing.
MIN_SPEED = 1
SPEED_LEVELS = [0.5, 0.7, 1.0]
START_LEVEL = 2
# Pivots at full power overshoot badly; turn a bit gentler than you drive.
TURN_SCALE = 1

SERVO_ENABLED = True
SERVO_CENTER = 90
SERVO_RANGE = 60             # +/- degrees at full right-stick deflection
SERVO_MIN_CHANGE = 3         # don't resend tiny jitters


def shape(mag, top):
    """Map stick magnitude DEADZONE..1 onto MIN_SPEED..top."""
    if mag < DEADZONE:
        return 0.0
    frac = (mag - DEADZONE) / (1.0 - DEADZONE)
    low = min(MIN_SPEED, top)
    return low + frac * (top - low)


def stick_to_cmd(x, y, top):
    """Left stick -> (v, w) in the car's own convention (+w = turn right)."""
    fwd = -y                                   # stick up is negative
    if abs(x) < DEADZONE and abs(fwd) < DEADZONE:
        return 0.0, 0.0
    if abs(x) > abs(fwd):
        return 0.0, math.copysign(shape(abs(x), top * TURN_SCALE), x)
    return math.copysign(shape(abs(fwd), top), fwd), 0.0


class Link:
    """TCP connection to the car, with a reader that drains telemetry.

    The firmware streams a telemetry line every 100 ms. If nobody reads
    it, buffers eventually fill and the car's control loop can stall, so
    we always read -- and keep the latest distance for the status line.
    """

    def __init__(self, ip, port):
        self.sock = socket.create_connection((ip, port), timeout=3.0)
        self.sock.settimeout(0.5)
        self.alive = True
        self.distance = None
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.sock.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            *lines, buf = buf.split(b"\n")
            for line in lines:
                p = line.decode(errors="ignore").strip().split(",")
                if len(p) >= 3 and p[0] == "T":
                    try:
                        self.distance = float(p[2])
                    except ValueError:
                        pass
        self.alive = False

    def send(self, text):
        self.sock.sendall((text + "\n").encode())

    def stop_and_close(self):
        try:
            for _ in range(3):
                self.send("0,0")
                time.sleep(0.05)
            if SERVO_ENABLED:
                self.send(f"S,{SERVO_CENTER}")
        except OSError:
            pass
        self.alive = False
        self.sock.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ip", default=DEFAULT_IP, help="car IP (find-robot.py)")
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("No controller found. Pair it (or plug in USB) first.")
    pad = pygame.joystick.Joystick(0)
    pad.init()
    print(f"Controller: {pad.get_name()}")

    print(f"Connecting to {args.ip}:{args.port} ...")
    try:
        link = Link(args.ip, args.port)
    except OSError as e:
        raise SystemExit(f"Could not reach the car ({e}). Run find-robot.py.")
    print("Connected. Left stick drives, L1/R1 speed, X brakes, Options quits.\n")

    level = START_LEVEL
    last_servo = None
    period = 1.0 / SEND_HZ

    try:
        running = True
        while running:
            tick = time.monotonic()

            for ev in pygame.event.get():
                if ev.type == pygame.JOYDEVICEREMOVED:
                    print("\nController disconnected -- stopping.")
                    running = False
                elif ev.type == pygame.JOYBUTTONDOWN:
                    if ev.button == BTN_OPTIONS:
                        running = False
                    elif ev.button == BTN_R1:
                        level = min(level + 1, len(SPEED_LEVELS) - 1)
                    elif ev.button == BTN_L1:
                        level = max(level - 1, 0)
            if not running:
                break

            if not link.alive:
                print("\nLost connection to the car.")
                break

            top = SPEED_LEVELS[level]
            v, w = stick_to_cmd(pad.get_axis(AXIS_LX), pad.get_axis(AXIS_LY), top)
            if pad.get_button(BTN_CROSS):
                v, w = 0.0, 0.0

            link.send(f"{v:.3f},{w:.3f}")

            if SERVO_ENABLED:
                rx = pad.get_axis(AXIS_RX)
                rx = 0.0 if abs(rx) < DEADZONE else rx
                # Servo: 0 = full right, 180 = full left.
                deg = int(round(SERVO_CENTER - rx * SERVO_RANGE))
                if last_servo is None or abs(deg - last_servo) >= SERVO_MIN_CHANGE:
                    link.send(f"S,{deg}")
                    last_servo = deg

            d = link.distance
            dist = "  --  " if d is None else ("  far " if d < 0 else f"{d:5.0f}cm")
            print(f"\rspeed {top:.1f}  v={v:+.2f} w={w:+.2f}  dist {dist}   ",
                  end="", flush=True)

            time.sleep(max(0.0, period - (time.monotonic() - tick)))

    except KeyboardInterrupt:
        pass
    except OSError as e:
        print(f"\nSend failed ({e}).")
    finally:
        link.stop_and_close()
        pygame.quit()
        print("\nStopped.")


if __name__ == "__main__":
    main()
