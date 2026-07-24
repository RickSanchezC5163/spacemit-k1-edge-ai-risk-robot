import subprocess

from tools.k1_pwm7_light import Pwm7Light, brightness_to_pulse_us


def test_brightness_to_pulse_us() -> None:
    assert brightness_to_pulse_us(-1) == 0
    assert brightness_to_pulse_us(0) == 0
    assert brightness_to_pulse_us(5) == 1140
    assert brightness_to_pulse_us(20) == 1260
    assert brightness_to_pulse_us(100) == 1900
    assert brightness_to_pulse_us(150) == 1900


def test_light_uses_fixed_privileged_commands(monkeypatch) -> None:
    calls: list[list[str]] = []

    def fake_run(command, **kwargs):
        calls.append(command)
        return subprocess.CompletedProcess(command, 0, stdout="mode=pwm_idle_low\n")

    monkeypatch.setattr(subprocess, "run", fake_run)
    light = Pwm7Light()
    assert light.set_brightness(100) == 1900
    light.off()
    assert calls[0][-1] == "on"
    assert calls[1][-1] == "off"
