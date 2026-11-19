
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class EpisodeConfig:
    experiment_name: str

    duration_seconds: float = 20.0
    pre_cue_seconds: float = 3.0
    post_cue_seconds: float = 3.0

    output_dir: Path = Path("experiments")

    topics: list[str] = field(default_factory=lambda: [
    "/conduit/camera/wide/image_raw/compressed",
    "/mobott/localization",
    "/mobott/commands"

    # topics: list[str] = field(default_factory=lambda: [
    #     "/camera/image_raw",
    #     "/mobott/localization",
    #     "/mobott/esp32/events",
    ])
