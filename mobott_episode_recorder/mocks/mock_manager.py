
from datetime import datetime
from episode_recorder.models import Episode, EpisodeStatus


class MockEpisodeManager:
    def __init__(self):
        self.next_number = 1

    def create_episode(self, config):
        number = self.next_number
        self.next_number += 1

        episode = Episode(
            experiment_name=config.experiment_name,
            episode_number=number,
            episode_id=f"{config.experiment_name}_{number:03d}",
            directory=config.output_dir / config.experiment_name,
            created_at=datetime.now().astimezone(),
        )

        print(f"[Manager] Created: {episode.episode_id}")
        return episode

    def finalize_episode(self, episode):
        episode.status = EpisodeStatus.COMPLETED
        print(f"[Manager] Finalized: {episode.episode_id}")
