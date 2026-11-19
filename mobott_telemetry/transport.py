
class MockESP32Transport:
    def send(self, command: dict) -> None:
        print(
            f"[ESP32 MOCK] "
            f"id={command['command_id']} "
            f"linear={command['linear']} "
            f"angular={command['angular']}"
        )

    def close(self) -> None:
        pass



import socket


class ESP32WiFiTransport:
    def __init__(self, host="10.157.163.41", port=1234):
        self.socket = socket.create_connection(
            (host, port),
            timeout=3,
        )
        self.socket.settimeout(1)
        print(f"[ESP32] Connected to {host}:{port}")

    def send(self, command: dict) -> None:
        v = command["linear"]
        w = command["angular"]

        message = f"{v},{w}\n"
        self.socket.sendall(message.encode())

    def close(self) -> None:
        try:
            self.socket.sendall(b"0.0,0.0\n")
        except OSError:
            pass

        self.socket.close()
