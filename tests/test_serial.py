import json

import pytest

from raspberry_pi.motor_interface import MotorInterface, MotorProtocolError, parse_status_line


class FakeSerial:
    def __init__(self):
        self.writes = []

    def write(self, payload):
        self.writes.append(payload)


def config():
    return {
        "serial_port": "fake",
        "baudrate": 115200,
        "serial_timeout_s": 0.1,
        "watchdog_s": 0.4,
    }


def test_status_parser_and_state_update():
    payload = {
        "homed": True,
        "estop": False,
        "fault": "",
        "position_mm": 123.4,
        "velocity_mm_s": -20.0,
        "target_velocity_mm_s": -25.0,
        "left_limit": False,
        "right_limit": False,
        "watchdog_ok": True,
        "rail_length_mm": 498.0,
    }
    line = "STATUS:" + json.dumps(payload)
    assert parse_status_line(line)["homed"] is True
    interface = MotorInterface(config())
    interface._consume_line(line, now_s=10.0)
    status = interface._status
    assert status.homed
    assert status.position_mm == pytest.approx(123.4)
    assert status.velocity_mm_s == pytest.approx(-20.0)


def test_velocity_protocol_rejects_unsafe_or_nonfinite_values():
    interface = MotorInterface(config())
    fake = FakeSerial()
    interface._serial = fake
    interface.set_velocity(-123.4567)
    assert fake.writes == [b"VEL:-123.457\n"]
    with pytest.raises(ValueError):
        interface.set_velocity(601.0)
    with pytest.raises(ValueError):
        interface.set_velocity(float("nan"))


def test_malformed_status_is_not_silently_accepted():
    with pytest.raises(MotorProtocolError):
        parse_status_line("STATUS:{not-json}")

