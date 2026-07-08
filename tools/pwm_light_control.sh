#!/usr/bin/env bash
set -euo pipefail

chip="${1:-pwmchip1}"
percent="${2:-0}"
period_ns="${3:-50000}"

case "$chip" in
  pwmchip0)
    : "${period_ns:=1000000}"
    ;;
  pwmchip1)
    if [ "$period_ns" -gt 100000 ]; then
      period_ns=50000
    fi
    ;;
  *)
    echo "Usage: $0 pwmchip0|pwmchip1 duty_percent [period_ns]" >&2
    exit 2
    ;;
esac

case "$percent" in
  ''|*[!0-9]*)
    echo "duty_percent must be an integer from 0 to 50" >&2
    exit 2
    ;;
esac

if [ "$percent" -gt 50 ]; then
  percent=50
fi

pwm_root="/sys/class/pwm/${chip}"
pwm="${pwm_root}/pwm0"

if [ ! -d "$pwm_root" ]; then
  echo "PWM chip not found: $pwm_root" >&2
  exit 1
fi

if [ ! -d "$pwm" ]; then
  echo 0 > "${pwm_root}/export" 2>/dev/null || true
  sleep 0.1
fi

if [ "$percent" -eq 0 ]; then
  echo 0 > "${pwm}/enable" 2>/dev/null || true
  echo 0 > "${pwm}/duty_cycle"
  echo "light pwm disabled on ${chip}"
  exit 0
fi

duty_ns=$(( period_ns * percent / 100 ))

echo 0 > "${pwm}/enable" 2>/dev/null || true
echo "$period_ns" > "${pwm}/period"
echo "$duty_ns" > "${pwm}/duty_cycle"
echo 1 > "${pwm}/enable"

echo "light pwm enabled on ${chip}: duty=${percent}% period=${period_ns}ns"
