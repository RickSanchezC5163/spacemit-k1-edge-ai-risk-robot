#!/usr/bin/env bash
set -euo pipefail

PWM_DEVICE_NAME="d401bc00.pwm"
PWM_DEVICE="/sys/devices/platform/soc/${PWM_DEVICE_NAME}"
PWM_DRIVER="/sys/bus/platform/drivers/pxa25x-pwm"
RUNTIME_LINK="/run/k1_pwm7_pwmchip"
PERIOD_NS=20000000
DUTY_NS=1900000
ACTION="${1:-status}"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "k1-light-mode must run as root" >&2
  exit 2
fi

find_pwmchip() {
  local candidate
  for candidate in /sys/class/pwm/pwmchip*; do
    if [[ "$(readlink -f "${candidate}/device")" == "${PWM_DEVICE}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

pwm_prepare() {
  local pwmchip=""
  pwmchip="$(find_pwmchip 2>/dev/null || true)"
  if [[ ! -L "${PWM_DRIVER}/${PWM_DEVICE_NAME}" ]]; then
    printf '%s' "${PWM_DEVICE_NAME}" > "${PWM_DRIVER}/bind"
  fi
  for _ in $(seq 1 50); do
    pwmchip="$(find_pwmchip 2>/dev/null || true)"
    [[ -n "${pwmchip}" ]] && break
    sleep 0.02
  done
  [[ -n "${pwmchip}" ]] || { echo "PWM7 pwmchip was not found" >&2; exit 3; }
  if [[ ! -d "${pwmchip}/pwm0" ]]; then
    printf '0' > "${pwmchip}/export"
  fi
  for _ in $(seq 1 50); do
    [[ -d "${pwmchip}/pwm0" ]] && break
    sleep 0.02
  done
  [[ -d "${pwmchip}/pwm0" ]] || { echo "PWM7 channel 0 was not exported" >&2; exit 4; }
  if [[ "$(<"${pwmchip}/pwm0/period")" != "${PERIOD_NS}" ]]; then
    [[ "$(<"${pwmchip}/pwm0/enable")" != "1" ]] || {
      echo "refusing to change the live PWM7 period" >&2
      exit 5
    }
    printf '%s' "${PERIOD_NS}" > "${pwmchip}/pwm0/period"
  fi
  if [[ "$(<"${pwmchip}/pwm0/enable")" != "1" ]]; then
    printf '0' > "${pwmchip}/pwm0/duty_cycle"
    printf '1' > "${pwmchip}/pwm0/enable"
  fi
  ln -sfn "${pwmchip}" "${RUNTIME_LINK}"
  printf '%s\n' "${pwmchip}"
}

pwm_set_duty() {
  local pwmchip
  pwmchip="$(pwm_prepare)"
  printf '%s' "$1" > "${pwmchip}/pwm0/duty_cycle"
}

case "${ACTION}" in
  on)
    pwm_set_duty "${DUTY_NS}"
    echo "mode=pwm_on period_ns=${PERIOD_NS} duty_ns=${DUTY_NS}"
    ;;
  off)
    pwm_set_duty 0
    echo "mode=pwm_idle_low period_ns=${PERIOD_NS} duty_ns=0"
    ;;
  status)
    pwmchip="$(find_pwmchip 2>/dev/null || true)"
    if [[ -n "${pwmchip}" && -d "${pwmchip}/pwm0" ]]; then
      echo "mode=pwm period_ns=$(<"${pwmchip}/pwm0/period") duty_ns=$(<"${pwmchip}/pwm0/duty_cycle") enabled=$(<"${pwmchip}/pwm0/enable")"
    else
      echo "mode=unknown"
    fi
    ;;
  *) echo "usage: k1-light-mode {on|off|status}" >&2; exit 1 ;;
esac
