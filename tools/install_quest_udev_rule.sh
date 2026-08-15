#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rule_source="$script_dir/51-oculus-quest.rules"
rule_target="/etc/udev/rules.d/51-oculus-quest.rules"

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run with sudo: sudo $0" >&2
  exit 1
fi

install -o root -g root -m 0644 "$rule_source" "$rule_target"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=2833
echo "Installed $rule_target. Replug the Quest USB cable, then accept USB debugging in-headset."
