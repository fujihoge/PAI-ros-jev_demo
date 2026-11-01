"""CountingTransport: wraps a JevTransport, counts every call (retries included), enforces a call limit and stops all
further calls after a fatal error (429, authentication or other API errors). No ROS; never imports typesafe_sdk."""
from __future__ import annotations

import threading
from dataclasses import dataclass, field

from adapters.jev import JevAnswer, JevError

FATAL_REASONS = ("http_429", "auth_error")


@dataclass
class CountingTransport:
    """`limit` = most calls allowed in total (retries included). After a fatal error or when the limit is reached,
    every later call raises a non-retryable JevError, so no further request leaves."""
    inner: object
    limit: int
    calls: int = 0
    fatal: str | None = None
    errors: list = field(default_factory=list)
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    @property
    def real_api(self) -> bool:
        return self.inner.real_api

    def ask(self, state: dict, instructions: str, criteria: dict, timeout_s: float) -> JevAnswer:
        with self._lock:
            if self.fatal:
                raise JevError(f"stopped_after_{self.fatal}")
            if self.calls >= self.limit:
                self.fatal = "call_limit"
                raise JevError("call_limit")
            self.calls += 1
        try:
            return self.inner.ask(state, instructions, criteria, timeout_s)
        except JevError as e:
            with self._lock:
                self.errors.append(e.reason)
                if e.reason in FATAL_REASONS or e.reason.startswith("api_error"):
                    self.fatal = e.reason
            raise
