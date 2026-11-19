
import time


class MockEpisodeCues:
    def countdown(self, seconds):
        print(f"[Cue] Starting in {seconds}s")
        time.sleep(seconds)

    def show_start(self):
        print("[Cue] START RECORDING")

    def show_stop(self):
        print("[Cue] STOP RECORDING")

    def wait(self, seconds):
        time.sleep(seconds)
