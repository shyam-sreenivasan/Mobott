
from datetime import datetime
from pathlib import Path
from dataclasses import asdict
import yaml

from .models import Episode, EpisodeStatus


class EpisodeManager:
    def create_episode(self, config) -> Episode:
        experiment_dir = config.output_dir / config.experiment_name
        experiment_dir.mkdir(parents=True, exist_ok=True)

        existing = list(experiment_dir.glob("*_episode_*"))
        numbers = [
            int(p.name.rsplit("_episode_", 1)[1])
            for p in existing
            if p.name.rsplit("_episode_", 1)[1].isdigit()
        ]

        number = max(numbers, default=0) + 1
        now = datetime.now().astimezone()

        episode_id = f"{config.experiment_name}_{number:03d}"
        directory = experiment_dir / (
            f"{now.strftime('%Y-%m-%d_%H%M%S')}_episode_{number:03d}"
        )

        directory.mkdir(parents=True, exist_ok=False)
        (directory / "data").mkdir()

        episode = Episode(
            experiment_name=config.experiment_name,
            episode_number=number,
            episode_id=episode_id,
            directory=directory,
            created_at=now,
        )

        self._save_metadata(episode)
        return episode

    def finalize_episode(self, episode: Episode) -> None:
        self._save_metadata(episode)

    def _save_metadata(self, episode: Episode) -> None:
        data = asdict(episode)

        data["directory"] = str(episode.directory)
        data["status"] = episode.status.value

        for key in ("created_at", "recording_started_at", "recording_ended_at"):
            value = data[key]
            data[key] = value.isoformat() if value else None

        with open(episode.directory / "metadata.yaml", "w") as file:
            yaml.safe_dump(data, file, sort_keys=False)
