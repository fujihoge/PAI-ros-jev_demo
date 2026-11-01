"""Jev (TypeSafe System One) adapter: transports behind one interface, so a mock can never pass for the real API.

The API key is read from TYPESAFE_API_KEY by the SDK. It is never logged, printed or stored here.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol

QUESTION_NAME = "next_delivery"


class JevError(Exception):
    """Any failure to obtain a usable answer. `reason` is a short, loggable code (no secrets)."""

    def __init__(self, reason: str, retryable: bool = False) -> None:
        super().__init__(reason)
        self.reason = reason
        self.retryable = retryable


@dataclass(frozen=True)
class JevAnswer:
    choice: str
    confidence: float | None
    probabilities: dict
    model: str  # the model name actually returned by the service (or "mock")
    input_tokens: int | None
    output_tokens: int | None
    latency_s: float  # wall-clock
    real_api: bool  # False for mock/replay: such runs must never be reported as Jev API experiments
    extra: dict = field(default_factory=dict)


class JevTransport(Protocol):
    real_api: bool

    def ask(self, state: dict, instructions: str, criteria: dict[str, str | dict], timeout_s: float) -> JevAnswer: ...


class SdkTransport:
    """Real Jev API through the official `typesafe-sdk`."""

    real_api = True

    def __init__(self, model: str = "jev-latest") -> None:
        import os

        from typesafe_sdk import RetryPolicy, TypeSafeClient

        if not os.environ.get("TYPESAFE_API_KEY"):
            raise JevError("api_key_not_set")
        # Retries are done by JevPolicy (finite, logged); the SDK itself must not retry silently.
        self._client = TypeSafeClient(model=model, retry=RetryPolicy(max_retries=0))
        self.model_requested = model

    def ask(self, state: dict, instructions: str, criteria: dict[str, str | dict], timeout_s: float) -> JevAnswer:
        from typesafe_sdk import (Choice, TypeSafeAPIConnectionError, TypeSafeAPIError, TypeSafeAPITimeoutError,
                                  TypeSafeAuthenticationError, TypeSafeInternalServerError, TypeSafeRateLimitError)

        t0 = time.perf_counter()
        try:
            resp = self._client.system_one(
                state=state,
                questions={QUESTION_NAME: Choice(instructions=instructions, criteria=criteria)},
                timeout=timeout_s,
            )
        except TypeSafeAPITimeoutError as e:  # subclass of connection error: check first
            raise JevError("timeout", retryable=True) from e
        except TypeSafeAPIConnectionError as e:
            raise JevError("connection_error", retryable=True) from e
        except TypeSafeRateLimitError as e:
            raise JevError("http_429", retryable=True) from e
        except TypeSafeInternalServerError as e:
            raise JevError("http_5xx", retryable=True) from e
        except TypeSafeAuthenticationError as e:
            raise JevError("auth_error") from e
        except TypeSafeAPIError as e:
            raise JevError(f"api_error_{type(e).__name__}") from e
        latency = time.perf_counter() - t0
        ans = resp.answers.get(QUESTION_NAME)
        if ans is None or getattr(ans, "type", None) != "choice":
            raise JevError("unexpected_answer_type")
        return JevAnswer(
            choice=ans.choice, confidence=ans.confidence, probabilities=dict(ans.probabilities),
            model=resp.model, input_tokens=resp.usage.input_tokens, output_tokens=resp.usage.output_tokens,
            latency_s=latency, real_api=True,
        )


class MockTransport:
    """Deterministic stand-in for development and tests. NOT the Jev API; results must be labelled mock.

    `picker(criteria)` returns the chosen id (default: the lexicographically smallest id).
    `failures` is a list of JevError reasons raised on the first calls (to test retry/fallback).
    """

    real_api = False

    def __init__(self, picker=None, failures: list[JevError] | None = None, latency_s: float = 0.0) -> None:
        self.picker = picker or (lambda criteria: min(criteria))
        self.failures = list(failures or [])
        self.latency_s = latency_s
        self.calls = 0

    def ask(self, state: dict, instructions: str, criteria: dict[str, str | dict], timeout_s: float) -> JevAnswer:
        self.calls += 1
        if self.failures:
            raise self.failures.pop(0)
        choice = self.picker(criteria)
        probs = {k: (1.0 if k == choice else 0.0) for k in criteria}
        return JevAnswer(choice=choice, confidence=1.0, probabilities=probs, model="mock",
                         input_tokens=None, output_tokens=None, latency_s=self.latency_s, real_api=False)
