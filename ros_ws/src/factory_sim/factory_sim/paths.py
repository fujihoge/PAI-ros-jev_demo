"""Locate the repository root (the folder holding factory_b/, policies/, adapters/, recorded/) and put it on sys.path."""
from __future__ import annotations

import os
import sys
from pathlib import Path

MARKER = Path("factory_b") / "__init__.py"


def find_project_root(start: str | Path | None = None) -> Path:
    """Order: env B1_PROJECT_ROOT -> walk up from `start` (default: this file) -> walk up from the working directory."""
    env = os.environ.get("B1_PROJECT_ROOT")
    if env and (Path(env) / MARKER).exists():
        return Path(env)
    bases = [Path(start).resolve() if start else Path(__file__).resolve(), Path.cwd().resolve()]
    for base in bases:
        for parent in [base, *base.parents]:
            if (parent / MARKER).exists():
                return parent
    raise RuntimeError("repository root not found: set B1_PROJECT_ROOT to the folder that contains factory_b/")


def ensure_importable(root: str | Path | None = None) -> Path:
    r = Path(root) if root else find_project_root()
    if str(r) not in sys.path:
        sys.path.insert(0, str(r))
    return r
