
from pynput import keyboard
from threading import Lock


class KeyboardInput:
    def __init__(self):
        self.pressed = set()
        self.lock = Lock()

        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self.listener.start()

    def _on_press(self, key):
        try:
            with self.lock:
                self.pressed.add(key.char.lower())
        except AttributeError:
            pass

    def _on_release(self, key):
        try:
            with self.lock:
                self.pressed.discard(key.char.lower())
        except AttributeError:
            pass

    def get_command(self):
        with self.lock:
            keys = self.pressed.copy()

        v = 1.0 if "w" in keys else -1.0 if "s" in keys else 0.0
        w = -1.0 if "a" in keys else 1.0 if "d" in keys else 0.0

        return "KEYBOARD", v, w

    def close(self):
        self.listener.stop()
