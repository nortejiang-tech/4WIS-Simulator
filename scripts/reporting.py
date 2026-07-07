"""Small report-generation helpers shared by research scripts."""

from __future__ import annotations

import base64
import json
from pathlib import Path
from typing import Any


def image_to_base64(path: Path) -> str:
    """Return base64 text for embedding a local image in self-contained HTML."""
    return base64.b64encode(path.read_bytes()).decode("ascii")


def image_data_uri(path: Path, mime: str = "image/png") -> str:
    """Return a data URI for a local image."""
    return f"data:{mime};base64,{image_to_base64(path)}"


def write_json(path: Path, data: Any) -> None:
    """Write stable UTF-8 JSON for report sidecar metrics."""
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=1, default=float),
        encoding="utf-8",
    )
