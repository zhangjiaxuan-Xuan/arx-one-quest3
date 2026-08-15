"""Passive ARX CAN checks with no dependency on the vendor SDK."""

from pathlib import Path
import time


CANABLE_SERIALS = {
    "can0": "2074339F5743",
    "can1": "207A33695743",
}
def verify_canable_mapping(interface: str) -> str:
    serial = CANABLE_SERIALS[interface]
    directory = Path("/dev/serial/by-id")
    matches = [path for path in directory.glob(f"*{serial}*if00") if path.exists()]
    if len(matches) != 1:
        raise RuntimeError(
            f"{interface}对应CANable串口不存在或不唯一（序列号={serial}）"
        )
    return str(matches[0].resolve())


def verify_can_activity(
    interfaces: tuple[str, ...] = ("can0", "can1"),
    *,
    window_seconds: float = 0.5,
    minimum_frames: int = 20,
) -> dict[str, dict[str, int | str]]:
    stats: dict[str, tuple[int, int, str]] = {}
    for interface in interfaces:
        tty_device = verify_canable_mapping(interface)
        root = Path("/sys/class/net") / interface
        if not root.exists():
            raise RuntimeError(f"{interface} 不存在")
        operstate = (root / "operstate").read_text(encoding="ascii").strip()
        if operstate not in {"up", "unknown"}:
            raise RuntimeError(f"{interface} 未启用：operstate={operstate}")
        rx = int((root / "statistics/rx_packets").read_text(encoding="ascii"))
        errors = int((root / "statistics/rx_errors").read_text(encoding="ascii"))
        stats[interface] = (rx, errors, tty_device)
    time.sleep(window_seconds)
    report: dict[str, dict[str, int | str]] = {}
    failures: list[str] = []
    for interface, (rx_before, errors_before, tty_device) in stats.items():
        root = Path("/sys/class/net") / interface / "statistics"
        rx_after = int((root / "rx_packets").read_text(encoding="ascii"))
        errors_after = int((root / "rx_errors").read_text(encoding="ascii"))
        delta = rx_after - rx_before
        report[interface] = {
            "rx_delta": delta,
            "traffic": "active" if delta >= minimum_frames else "idle",
            "tty_device": tty_device,
        }
        if errors_after != errors_before:
            failures.append(f"{interface} 新增 {errors_after - errors_before} 个 RX 错误")
    # Both-idle is valid after a passive shutdown; both-active is valid after a
    # live query session.  One active and one idle is not a safe SDK startup
    # condition on this AC One: repeated tests showed that loading the vendor
    # library in this state can fail construction and abort in its destructor.
    # Rebuild only the host SLCAN transport first; do not probe with the SDK.
    activity = {name: item["traffic"] for name, item in report.items()}
    if len(set(activity.values())) > 1:
        failures.append(
            "双臂CAN活动不对称（"
            + "，".join(f"{name}={traffic}" for name, traffic in activity.items())
            + "）；禁止加载ARX SDK。请先执行 sudo ./tools/recover_arx_can.sh "
            "仅重建SLCAN（不要使用--usb-reset）"
        )
    if failures:
        raise RuntimeError("；".join(failures))
    return report
