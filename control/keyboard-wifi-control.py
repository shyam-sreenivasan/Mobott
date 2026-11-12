import socket
import keyboard
import time

# 🔧 change this to ESP32 IP shown in Serial Monitor
ESP_IP = "10.157.163.120"

PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ESP_IP, PORT))

print("Connected to ESP32")

while True:

    # ---------------- motion variables ----------------
    v = 0.0   # forward/backward
    w = 0.0   # turning

    # ---------------- keyboard mapping ----------------
    if keyboard.is_pressed('w'):
        v = 1.0
    elif keyboard.is_pressed('s'):
        v = -1.0

    if keyboard.is_pressed('a'):
        w = -1.0
    elif keyboard.is_pressed('d'):
        w = 1.0

    # ---------------- send to ESP32 ----------------
    msg = f"{v},{w}\n"
    sock.send(msg.encode())

    time.sleep(0.05)
