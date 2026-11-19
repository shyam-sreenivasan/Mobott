
from datetime import datetime

from .config import EpisodeConfig
from .models import Episode, EpisodeStatus


class EpisodeRunner:
    def __init__(self, config: EpisodeConfig, manager, recorder, cues):
        self.config = config
        self.manager = manager
        self.recorder = recorder
        self.cues = cues

    def run_episode(self) -> Episode:
        self.recorder.validate_topics()
        episode = self.manager.create_episode(self.config)

        recorder_started = False

        try:
            self.cues.countdown(self.config.pre_cue_seconds)

            self.recorder.start(episode)
            recorder_started = True

            episode.recording_started_at = datetime.now().astimezone()
            episode.status = EpisodeStatus.RECORDING

            self.cues.show_start()
            self.cues.wait(self.config.duration_seconds)

            episode.status = EpisodeStatus.COMPLETED

        except KeyboardInterrupt:
            episode.status = EpisodeStatus.INTERRUPTED
            print("\n[Runner] Episode interrupted")

        except Exception as exc:
            episode.status = EpisodeStatus.FAILED
            print(f"[Runner] Episode failed: {exc}")

        finally:
            if recorder_started:
                try:
                    self.recorder.stop()
                except Exception as exc:
                    episode.status = EpisodeStatus.FAILED
                    print(f"[Runner] Recorder cleanup failed: {exc}")

                episode.recording_ended_at = datetime.now().astimezone()

            try:
                self.cues.show_stop()
                self.cues.wait(self.config.post_cue_seconds)
            finally:
                self.manager.finalize_episode(episode)

        return episode
