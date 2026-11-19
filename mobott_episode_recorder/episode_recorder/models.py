
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from pathlib import Path


class EpisodeStatus(str, Enum):
    CREATED = "created"
    RECORDING = "recording"
    COMPLETED = "completed"
    FAILED = "failed"
    INTERRUPTED = "interrupted"


@dataclass
class Episode:
    experiment_name: str
    episode_number: int
    episode_id: str
    directory: Path
    created_at: datetime

    status: EpisodeStatus = EpisodeStatus.CREATED
    recording_started_at: datetime | None = None
    recording_ended_at: datetime | None = None
