# RP2040 firmware analysis and replacement rationale

The original `main.ino` referenced by the project brief was not present in the supplied empty repository. The replacement therefore starts from the pinout and behaviors documented in the brief rather than claiming a line-by-line migration.

The documented prior design had several safety and control problems:

- Position commands are a poor real-time interface for a Raspberry Pi velocity controller.
- An unsigned speed cannot represent direction or cross zero continuously.
- A forced 50 mm/s minimum creates a discontinuity near zero and makes fine balancing impossible.
- Blocking pulse generation and blocking homing prevent the firmware from servicing limits and communications promptly.
- Assuming a 250 mm power-on position makes software limits meaningless until a real reference is found.
- Without a watchdog, a disconnected host can leave the last motion command active.

The replacement in `firmware/main.ino` uses RP2040 hardware PWM for STEP generation and a PWM-wrap interrupt for signed step counting. The main loop remains available for serial input, switches, homing, braking, and the watchdog. Velocity is signed and acceleration-limited; zero is a true stop. Direction is changed only after PWM is disabled and the DIR setup delay has elapsed. Because the [official Pico SDK PWM documentation](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html) notes that disabling PWM can leave its output at either level, the firmware applies a GPIO output override to force STEP low while stopped.

Homing is a non-blocking state machine: seek left, back off, seek right, measure travel, validate the measured range, persist the travel length, and return to center. `homed` is never restored from EEPROM because persisted rail length does not establish the current position. Arduino-Pico emulates EEPROM in flash, so the code only commits when the measured length changes materially; see the [Arduino-Pico EEPROM documentation](https://arduino-pico.readthedocs.io/en/latest/eeprom.html).

The normal command ceiling is 600 mm/s. Commands between 600 and 800 mm/s are rejected; a command above the absolute 800 mm/s ceiling latches ESTOP. Dynamic end-of-rail limiting uses braking distance plus latency and mechanical margin. Hardware limit inputs, software limits, heartbeat loss, homing failure, faults, and ESTOP are enforced locally even if the Raspberry Pi misbehaves.

Supported protocol:

- `VEL:<signed mm/s>`
- `STOP`
- `ESTOP`
- `CLEAR_ESTOP`
- `CLEAR_FAULT`
- `STATUS?`
- `HEARTBEAT`
- `FULL_HOME` or `HOME`
- `POS:<mm>` for limited compatibility with the former position-oriented host
- `CAL_JOG:<-20..20 mm/s>` as a 250 ms dead-man commissioning command available only before homing

Before physical use, verify switch polarity, ENA polarity, and the DIR-to-right mapping at low speed with the motor mechanically unloaded or the cart secured. Send `HEARTBEAT`, then repeatedly send `CAL_JOG:10` while observing motion; each command expires after 250 ms. These polarity values are explicit constants near the top of the firmware.

