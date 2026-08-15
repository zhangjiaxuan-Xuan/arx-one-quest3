#!/usr/bin/env python3
"""Fail-fast passive ARX CAN preflight; intentionally does not import the SDK."""

from arx_can_preflight import verify_can_activity


def main() -> None:
    try:
        report = verify_can_activity()
    except RuntimeError as exc:
        raise SystemExit(f"SDK前CAN预检失败：{exc}") from None
    all_idle = all(item["traffic"] == "idle" for item in report.values())
    prefix = (
        "主机CAN接口预检通过（双臂电机在线状态未知）："
        if all_idle
        else "SDK前CAN接口预检通过："
    )
    print(
        prefix
        + "，".join(
            f"{name}={item['traffic']}({item['rx_delta']}帧/0.5s)"
            f"@{item['tty_device']}"
            for name, item in report.items()
        )
        + (
            "；idle仅表示主机当前未收到帧，不能证明电机已初始化；"
            "最终以官方SDK构造结果为准"
            if all_idle
            else "；当前双臂CAN均有反馈"
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
