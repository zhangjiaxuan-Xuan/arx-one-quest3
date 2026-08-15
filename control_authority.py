"""Explicit authority hierarchy for ARX collection and deployment."""

from dataclasses import dataclass
from enum import IntEnum
import threading


class AuthorityLevel(IntEnum):
    VR = 10
    COMPUTER_WORKFLOW = 20
    FAULT_AND_SIGNAL_SAFETY = 30
    ROBOT_SDK_SESSION = 40


@dataclass(frozen=True)
class AuthoritySnapshot:
    shutdown_latched: bool
    vr_enabled: bool
    reason: str


class SafetyAuthority:
    """A one-way safety latch: lower levels cannot clear a higher-level stop."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._shutdown_latched = False
        self._vr_enabled = False
        self._reason = "persistent robot SDK session initializing"

    def enable_vr(self) -> None:
        with self._lock:
            if self._shutdown_latched:
                raise RuntimeError(
                    f"安全停机已锁定，拒绝重新开放VR：{self._reason}"
                )
            self._vr_enabled = True
            self._reason = "VR motion authorized by workflow"

    def disable_vr(self, reason: str) -> None:
        with self._lock:
            self._vr_enabled = False
            if not self._shutdown_latched:
                self._reason = reason

    def request_safe_shutdown(self, reason: str) -> None:
        with self._lock:
            self._shutdown_latched = True
            self._vr_enabled = False
            self._reason = reason

    def vr_allowed(self) -> bool:
        with self._lock:
            return self._vr_enabled and not self._shutdown_latched

    def snapshot(self) -> AuthoritySnapshot:
        with self._lock:
            return AuthoritySnapshot(
                shutdown_latched=self._shutdown_latched,
                vr_enabled=self._vr_enabled,
                reason=self._reason,
            )
