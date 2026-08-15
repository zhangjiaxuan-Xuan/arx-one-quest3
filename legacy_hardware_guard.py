"""Block legacy entrypoints that bypass the persistent robot command session."""


def reject_legacy_direct_hardware(entrypoint: str) -> None:
    raise RuntimeError(
        f"{entrypoint} 的直接机械臂执行已被安全等级检阅禁用：该旧入口会自行创建/"
        "销毁 SDK，绕过停机位、权限闩锁和常驻命令会话。请使用 "
        "tools/start_quest3_collection_test.sh；模型部署执行器迁移到同一常驻会话后再开放。"
    )
