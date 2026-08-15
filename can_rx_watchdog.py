"""Kernel-level CAN feedback watchdog independent of the vendor SDK clock."""

from __future__ import annotations

from pathlib import Path
import threading
import time


class CanRxWatchdog:
    def __init__(
        self,
        interfaces: dict[str, str] | None = None,
        *,
        sys_class_net: Path = Path("/sys/class/net"),
    ) -> None:
        self.interfaces = interfaces or {"left": "can0", "right": "can1"}
        self.sys_class_net = sys_class_net
        self._rx: dict[str, int] = {}
        self._errors: dict[str, int] = {}
        self._progress: dict[str, float] = {}
        self._lock = threading.RLock()

    def _counter(self, interface: str, name: str) -> int:
        path = self.sys_class_net / interface / "statistics" / name
        return int(path.read_text(encoding="ascii").strip())

    def reset(self) -> None:
        with self._lock:
            now = time.monotonic()
            self._rx.clear()
            self._errors.clear()
            self._progress.clear()
            for side, interface in self.interfaces.items():
                root = self.sys_class_net / interface
                if not root.exists():
                    raise RuntimeError(f"{interface} 不存在")
                self._rx[side] = self._counter(interface, "rx_packets")
                self._errors[side] = self._counter(interface, "rx_errors")
                self._progress[side] = now

    def fault(self, timeout_seconds: float = 0.20) -> str | None:
        with self._lock:
            if not self._rx:
                self.reset()
                return None
            now = time.monotonic()
            for side, interface in self.interfaces.items():
                try:
                    rx = self._counter(interface, "rx_packets")
                    errors = self._counter(interface, "rx_errors")
                except (FileNotFoundError, OSError, ValueError) as exc:
                    return f"{side} {interface} 统计不可读：{exc}"
                if errors > self._errors[side]:
                    return (
                        f"{side} {interface} 新增RX错误="
                        f"{errors - self._errors[side]}"
                    )
                if rx < self._rx[side]:
                    return f"{side} {interface} RX计数回退，接口可能被重建"
                if rx > self._rx[side]:
                    self._rx[side] = rx
                    self._errors[side] = errors
                    self._progress[side] = now
                elif now - self._progress[side] > timeout_seconds:
                    return (
                        f"{side} {interface} 超过{timeout_seconds:.2f}s无电机反馈"
                        f"（rx_packets={rx}）"
                    )
            return None

    def snapshot(self) -> dict[str, int]:
        with self._lock:
            return dict(self._rx)
