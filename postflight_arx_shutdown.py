#!/usr/bin/env python3
"""Read-only CAN symmetry check after the ARX SDK has safely disconnected."""

from arx_can_preflight import verify_can_activity


def main() -> int:
    try:
        report = verify_can_activity()
    except RuntimeError as exc:
        print(f"退出后CAN检查异常：{exc}", flush=True)
        return 1
    summary = "，".join(
        f"{name}={item['traffic']}({item['rx_delta']}帧/0.5s)"
        for name, item in report.items()
    )
    print(f"退出后CAN检查通过：{summary}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
