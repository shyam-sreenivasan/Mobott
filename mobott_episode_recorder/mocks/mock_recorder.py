
class MockRecorder:
    def validate_topics(self):
        print("[Recorder] Topics validated")
        return True

    def start(self, episode):
        self.episode = episode
        print(f"[Recorder] Started: {episode.episode_id}")

    def stop(self):
        self.episode.message_counts = {
            "/camera/image_raw": 600,
            "/mobott/localization": 600,
            "/mobott/esp32/events": 30,
        }
        print("[Recorder] Stopped")
