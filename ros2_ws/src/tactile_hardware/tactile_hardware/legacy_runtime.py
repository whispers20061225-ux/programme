from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional


def ensure_legacy_import_paths() -> Optional[Path]:
    """Best-effort resolver for the legacy project root.

    Returns:
        Resolved project root path if found, otherwise None.
    """

    candidates = []

    explicit_root = os.environ.get("PROGRAMME_PROJECT_ROOT")
    if explicit_root:
        candidates.append(Path(explicit_root).expanduser())

    cwd = Path.cwd()
    candidates.extend([cwd, *cwd.parents])

    this_file = Path(__file__).resolve()
    candidates.extend(this_file.parents)

    visited = set()
    for candidate in candidates:
        try:
            resolved = candidate.resolve()
        except Exception:
            continue

        key = str(resolved)
        if key in visited:
            continue
        visited.add(key)

        if _looks_like_project_root(resolved):
            _prepend_sys_path(str(resolved))
            _prepend_sys_path(str(resolved / "src"))
            return resolved

    return None


def _looks_like_project_root(path: Path) -> bool:
    return (
        (path / "main.py").is_file()
        and (path / "src").is_dir()
        and (path / "config").is_dir()
    )


def _prepend_sys_path(path: str) -> None:
    if path not in sys.path:
        sys.path.insert(0, path)

