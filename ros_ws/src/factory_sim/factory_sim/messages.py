"""JSON payloads for the std_msgs/String topics. Every message carries schema_version and a monotonic state_version.

state_version is ONE counter for the whole node, incremented at every published message of any topic, so a
consumer can order messages from different topics.
"""
from __future__ import annotations

import json
import math

SCHEMA_VERSION = 1


class Versioner:
    def __init__(self) -> None:
        self._v = 0

    def next(self) -> int:
        self._v += 1
        return self._v

    @property
    def current(self) -> int:
        return self._v


def _clean(x):
    if isinstance(x, float):
        if math.isinf(x) or math.isnan(x):
            return None
        return round(x, 3)
    if isinstance(x, dict):
        return {str(k): _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    return x


def envelope(version: int, kind: str, body: dict) -> dict:
    d = {"schema_version": SCHEMA_VERSION, "state_version": version, "kind": kind}
    d.update(body)
    return _clean(d)


def dumps(d: dict) -> str:
    return json.dumps(d, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
