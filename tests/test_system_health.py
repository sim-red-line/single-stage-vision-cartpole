from raspberry_pi.system_health import PiPowerMonitor


def test_clean_pi_power_status_allows_control():
    status = PiPowerMonitor(
        required=True, runner=lambda: "throttled=0x0"
    ).poll()
    assert status.available
    assert status.clean
    assert status.safe_for_control


def test_historical_undervoltage_blocks_control():
    status = PiPowerMonitor(
        required=True, runner=lambda: "throttled=0x50000"
    ).poll()
    assert not status.clean
    assert not status.safe_for_control
    assert "undervoltage occurred" in status.reason
    assert "throttling occurred" in status.reason


def test_missing_vcgencmd_only_blocks_when_required():
    def unavailable():
        raise FileNotFoundError("vcgencmd")

    optional = PiPowerMonitor(required=False, runner=unavailable).poll()
    required = PiPowerMonitor(required=True, runner=unavailable).poll()
    assert optional.safe_for_control
    assert not required.safe_for_control
