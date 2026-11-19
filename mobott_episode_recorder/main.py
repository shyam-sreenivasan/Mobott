
import rclpy

from episode_recorder.config import EpisodeConfig
from episode_recorder.runner import EpisodeRunner
from episode_recorder.manager import EpisodeManager
from episode_recorder.recorder import ROSDataRecorder
from mocks.mock_cues import MockEpisodeCues


def main():
    rclpy.init()
    recorder = None

    try:
        config = EpisodeConfig(
            experiment_name="marker_localization"
        )

        recorder = ROSDataRecorder(config.topics)

        runner = EpisodeRunner(
            config=config,
            manager=EpisodeManager(),
            recorder=recorder,
            cues=MockEpisodeCues(),
        )

        runner.run_episode()

    finally:
        if recorder is not None:
            recorder.close()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
