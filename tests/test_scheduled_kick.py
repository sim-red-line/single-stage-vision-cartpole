import sys

import pytest

from experiments import run_scheduled_swing_up as scheduled


@pytest.mark.parametrize("value", ["nan", "inf", "-1", "1500.1"])
def test_invalid_startup_acceleration_is_rejected_before_hardware(monkeypatch, value):
    calls = []
    monkeypatch.setattr(scheduled, "request", lambda *a, **k: calls.append(a))
    monkeypatch.setattr(sys, "argv", ["trial", "--start-at-unix", "100",
                                     "--kick-acceleration", value])
    with pytest.raises(SystemExit) as error:
        scheduled.main()
    assert error.value.code == 2
    assert calls == []
