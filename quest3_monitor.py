#!/usr/bin/env python3
import argparse
import time

from quest3_input import Quest3Receiver


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor Quest bimanual controller packets without robot control")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--port", type=int, default=7447)
    args = parser.parse_args()

    receiver = Quest3Receiver(port=args.port)
    receiver.start()
    receiver.ready.wait(2.0)
    if receiver.error:
        raise RuntimeError(receiver.error)

    started = time.monotonic()
    last_print = started
    first_sequence = None
    last_sequence = None
    received_updates = 0
    try:
        while time.monotonic() - started < args.duration:
            snapshot = receiver.snapshot()
            if snapshot.sequence != last_sequence and snapshot.sequence >= 0:
                if first_sequence is None:
                    first_sequence = snapshot.sequence
                last_sequence = snapshot.sequence
                received_updates += 1
            now = time.monotonic()
            if now - last_print >= 0.5:
                print(
                    f"sender={receiver.sender or '-'} seq={snapshot.sequence} fresh={snapshot.fresh(now)} "
                    f"L(track={snapshot.left.tracking} grip={snapshot.left.grip} trigger={snapshot.left.trigger:.2f}) "
                    f"R(track={snapshot.right.tracking} grip={snapshot.right.grip} trigger={snapshot.right.trigger:.2f})",
                    flush=True,
                )
                last_print = now
            time.sleep(0.005)
    finally:
        receiver.close()

    elapsed = time.monotonic() - started
    span = 0 if first_sequence is None or last_sequence is None else last_sequence - first_sequence + 1
    missing = max(0, span - received_updates)
    print(
        f"SUMMARY elapsed={elapsed:.2f}s updates={received_updates} rate={received_updates / elapsed:.1f}Hz "
        f"sequence_span={span} missing_observed={missing} invalid={receiver.invalid_packets}",
        flush=True,
    )


if __name__ == "__main__":
    main()
