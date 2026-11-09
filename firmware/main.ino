/*
 * RP2040 cart motion controller for the vision cart-pole project.
 *
 * Arduino core: Earle Philhower arduino-pico (uses the RP2040 hardware PWM SDK).
 * Pin assignments are fixed by the existing machine: ENA=6, STEP=7, DIR=8,
 * left limit=9, right limit=10.
 *
 * The Raspberry Pi may request velocity, but cannot bypass this firmware's
 * watchdog, homing, limits, acceleration, braking, or ESTOP latches.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// Fixed wiring.
constexpr uint8_t PIN_ENA = 6;
constexpr uint8_t PIN_STEP = 7;
constexpr uint8_t PIN_DIR = 8;
constexpr uint8_t PIN_LIMIT_LEFT = 9;
constexpr uint8_t PIN_LIMIT_RIGHT = 10;

constexpr bool DRIVER_ENABLE_LEVEL = LOW;
constexpr bool DRIVER_DISABLE_LEVEL = HIGH;
// The installed normally-closed switches hold the input LOW at rest and open
// to the internal pull-up when pressed. Commissioning must verify both inputs.
constexpr bool LIMIT_ACTIVE_LEVEL = HIGH;
constexpr bool DIR_RIGHT_LEVEL = LOW;     // Verified: positive command must move the cart right.

constexpr float STEP_MM = 0.0375f;
constexpr float OPERATING_MAX_SPEED_MM_S = 600.0f;
constexpr float ABSOLUTE_MAX_SPEED_MM_S = 800.0f;
constexpr float POSITION_MODE_MAX_SPEED_MM_S = 20.0f;
constexpr float ACCEL_MM_S2 = 2000.0f;
constexpr float SOFT_MARGIN_MM = 10.0f;
constexpr float BRAKING_MARGIN_MM = 20.0f;
constexpr float CONTROL_LATENCY_S = 0.080f;
constexpr uint32_t WATCHDOG_MS = 400;
constexpr uint32_t HOME_TIMEOUT_MS = 18000;
constexpr float HOME_SEEK_SPEED_MM_S = 40.0f;
constexpr float HOME_BACKOFF_SPEED_MM_S = 20.0f;
constexpr float HOME_BACKOFF_MM = 6.0f;
constexpr float HOME_MIN_TRAVEL_MM = 300.0f;
constexpr float HOME_MAX_TRAVEL_MM = 700.0f;
constexpr float DEFAULT_RAIL_LENGTH_MM = 500.0f;
constexpr float PWM_MIN_FREQUENCY_HZ = 8.0f;
constexpr uint16_t PWM_LOW_SPEED_WRAP = 65535u;
constexpr uint16_t PWM_HIGH_SPEED_WRAP = 1999u;
constexpr float PWM_BAND_UP_HZ = 1200.0f;
constexpr float PWM_BAND_DOWN_HZ = 800.0f;
constexpr uint32_t CONFIG_MAGIC = 0x43504F4Cu;  // "CPOL"
constexpr size_t EEPROM_BYTES = 256;
constexpr char FIRMWARE_VERSION[] = "cartpole-safe-8";
constexpr char CAL_JOG_PREFIX[] = "CAL_JOG:";

struct PersistentConfig {
  uint32_t magic;
  float railLengthMm;
};

enum class HomeState : uint8_t {
  NONE,
  SEEK_LEFT,
  BACK_OFF_LEFT,
  SEEK_RIGHT,
  MOVE_CENTER,
  FAILED
};

volatile int32_t g_stepPosition = 0;
volatile int8_t g_stepDirection = 1;
volatile bool g_pwmCounting = false;
uint g_pwmSlice = 0;
uint g_pwmChannel = 0;
uint16_t g_pwmWrap = PWM_LOW_SPEED_WRAP;
float g_programmedStepFrequencyHz = 0.0f;

float g_targetVelocity = 0.0f;
float g_currentVelocity = 0.0f;
float g_railLengthMm = DEFAULT_RAIL_LENGTH_MM;
float g_persistedRailLengthMm = DEFAULT_RAIL_LENGTH_MM;
bool g_persistedConfigValid = false;
bool g_homed = false;
// Fail closed from the first instruction after reset. CLEAR_ESTOP is the only
// command that may release this latch, and it still leaves ENA disabled.
bool g_estop = true;
bool g_watchdogFault = false;
bool g_positionMode = false;
float g_positionTargetMm = 0.0f;
HomeState g_homeState = HomeState::NONE;
uint32_t g_homeStateStartedMs = 0;
int32_t g_homeStartSteps = 0;
uint32_t g_lastHeartbeatMs = 0;
uint32_t g_calibrationJogDeadlineMs = 0;
uint32_t g_lastMotionUpdateUs = 0;
char g_fault[32] = "ESTOP";

char g_rxBuffer[96];
size_t g_rxLength = 0;

bool leftLimitActive() { return digitalRead(PIN_LIMIT_LEFT) == LIMIT_ACTIVE_LEVEL; }
bool rightLimitActive() { return digitalRead(PIN_LIMIT_RIGHT) == LIMIT_ACTIVE_LEVEL; }

int32_t atomicStepPosition() {
  noInterrupts();
  const int32_t value = g_stepPosition;
  interrupts();
  return value;
}

void atomicSetStepPosition(int32_t value) {
  noInterrupts();
  g_stepPosition = value;
  interrupts();
}

float positionMm() { return atomicStepPosition() * STEP_MM; }

void onPwmWrap() {
  pwm_clear_irq(g_pwmSlice);
  if (g_pwmCounting) {
    g_stepPosition += g_stepDirection;
  }
}

void disableStepPwm() {
  g_pwmCounting = false;
  g_programmedStepFrequencyHz = 0.0f;
  pwm_set_chan_level(g_pwmSlice, g_pwmChannel, 0);
  pwm_set_enabled(g_pwmSlice, false);
  // A disabled PWM can otherwise freeze high at an arbitrary counter phase.
  gpio_set_outover(PIN_STEP, GPIO_OVERRIDE_LOW);
}

bool setStepFrequency(float frequencyHz, int8_t direction) {
  if (!isfinite(frequencyHz) || frequencyHz < PWM_MIN_FREQUENCY_HZ) {
    disableStepPwm();
    return false;
  }

  uint16_t desiredWrap = g_pwmWrap;
  if (!g_pwmCounting) {
    desiredWrap = frequencyHz >= PWM_BAND_UP_HZ ? PWM_HIGH_SPEED_WRAP : PWM_LOW_SPEED_WRAP;
  } else if (g_pwmWrap == PWM_LOW_SPEED_WRAP && frequencyHz >= PWM_BAND_UP_HZ) {
    desiredWrap = PWM_HIGH_SPEED_WRAP;
  } else if (g_pwmWrap == PWM_HIGH_SPEED_WRAP && frequencyHz <= PWM_BAND_DOWN_HZ) {
    desiredWrap = PWM_LOW_SPEED_WRAP;
  }

  const float clockHz = static_cast<float>(clock_get_hz(clk_sys));
  float divider = clockHz / (frequencyHz * (static_cast<float>(desiredWrap) + 1.0f));
  if (!isfinite(divider) || divider < 1.0f || divider >= 256.0f) {
    disableStepPwm();
    return false;
  }

  const bool restart = !g_pwmCounting || direction != g_stepDirection || desiredWrap != g_pwmWrap;
  if (!restart && fabsf(frequencyHz - g_programmedStepFrequencyHz) < 0.05f) return true;

  if (restart) {
    disableStepPwm();
    gpio_put(PIN_DIR, direction > 0 ? DIR_RIGHT_LEVEL : !DIR_RIGHT_LEVEL);
    delayMicroseconds(5);  // DIR setup time before the first STEP edge.
    g_stepDirection = direction;
    g_pwmWrap = desiredWrap;
    pwm_set_clkdiv(g_pwmSlice, divider);
    pwm_set_wrap(g_pwmSlice, desiredWrap);
    pwm_set_chan_level(
        g_pwmSlice, g_pwmChannel,
        static_cast<uint16_t>((static_cast<uint32_t>(desiredWrap) + 1u) / 2u));
    pwm_set_counter(g_pwmSlice, 0);
    pwm_clear_irq(g_pwmSlice);
    gpio_set_outover(PIN_STEP, GPIO_OVERRIDE_NORMAL);
    g_pwmCounting = true;
    pwm_set_enabled(g_pwmSlice, true);
  } else {
    // Updating only the divider preserves the in-flight PWM period. Resetting
    // the counter here would suppress low-frequency pulses in the 1 kHz loop.
    pwm_set_clkdiv(g_pwmSlice, divider);
  }
  g_programmedStepFrequencyHz = frequencyHz;
  return true;
}

void applyCurrentVelocity() {
  if (g_estop || fabsf(g_currentVelocity) < STEP_MM * PWM_MIN_FREQUENCY_HZ) {
    disableStepPwm();
    return;
  }
  const int8_t direction = g_currentVelocity > 0.0f ? 1 : -1;
  const float frequency = fabsf(g_currentVelocity) / STEP_MM;
  setStepFrequency(frequency, direction);
}

void driverEnable(bool enabled) {
  digitalWrite(PIN_ENA, enabled ? DRIVER_ENABLE_LEVEL : DRIVER_DISABLE_LEVEL);
}

void immediateStop() {
  g_targetVelocity = 0.0f;
  g_currentVelocity = 0.0f;
  g_positionMode = false;
  disableStepPwm();
}

void latchFault(const char* code, bool disableDriver = false) {
  strncpy(g_fault, code, sizeof(g_fault) - 1);
  g_fault[sizeof(g_fault) - 1] = '\0';
  immediateStop();
  if (disableDriver) driverEnable(false);
  Serial.print("FAULT:");
  Serial.println(g_fault);
}

void latchEstop(const char* reason) {
  g_estop = true;
  g_homed = false;
  strncpy(g_fault, reason, sizeof(g_fault) - 1);
  g_fault[sizeof(g_fault) - 1] = '\0';
  immediateStop();
  driverEnable(false);
}

bool heartbeatFresh() {
  return g_lastHeartbeatMs != 0 && static_cast<uint32_t>(millis() - g_lastHeartbeatMs) <= WATCHDOG_MS;
}

float maximumSafeVelocity(float requested) {
  if (!g_homed || g_homeState != HomeState::NONE) return requested;
  const float position = positionMm();
  const float leftBoundary = SOFT_MARGIN_MM;
  const float rightBoundary = g_railLengthMm - SOFT_MARGIN_MM;
  const float latencyTravel = fabsf(g_currentVelocity) * CONTROL_LATENCY_S;
  const float brakingDistance =
      g_currentVelocity * g_currentVelocity / (2.0f * ACCEL_MM_S2) +
      latencyTravel + BRAKING_MARGIN_MM;

  if (requested > 0.0f) {
    const float remaining = rightBoundary - position;
    if (remaining <= 0.0f || (g_currentVelocity > 0.0f && remaining <= brakingDistance)) return 0.0f;
    const float usable = max(0.0f, remaining - BRAKING_MARGIN_MM);
    return min(requested, sqrtf(2.0f * ACCEL_MM_S2 * usable));
  }
  if (requested < 0.0f) {
    const float remaining = position - leftBoundary;
    if (remaining <= 0.0f || (g_currentVelocity < 0.0f && remaining <= brakingDistance)) return 0.0f;
    const float usable = max(0.0f, remaining - BRAKING_MARGIN_MM);
    return max(requested, -sqrtf(2.0f * ACCEL_MM_S2 * usable));
  }
  return 0.0f;
}

void beginHomeState(HomeState state) {
  g_homeState = state;
  g_homeStateStartedMs = millis();
  g_homeStartSteps = atomicStepPosition();
}

void failHoming(const char* reason) {
  g_homed = false;
  g_homeState = HomeState::FAILED;
  // Homing faults must release the motor driver as well as stop pulses.
  latchFault(reason, true);
}

void startHoming() {
  if (g_estop) {
    Serial.println("ERR:ESTOP_LATCHED");
    return;
  }
  if (!heartbeatFresh()) {
    Serial.println("ERR:HEARTBEAT_REQUIRED");
    return;
  }
  g_fault[0] = '\0';
  g_homed = false;
  g_positionMode = false;
  driverEnable(true);
  beginHomeState(HomeState::SEEK_LEFT);
  g_targetVelocity = -HOME_SEEK_SPEED_MM_S;
  Serial.println("OK:HOME_STARTED");
}

bool homingTimedOut() {
  return static_cast<uint32_t>(millis() - g_homeStateStartedMs) > HOME_TIMEOUT_MS;
}

void updateHoming() {
  if (g_homeState == HomeState::NONE || g_homeState == HomeState::FAILED) return;
  if (!heartbeatFresh()) {
    failHoming("HOME_WATCHDOG");
    return;
  }
  if (homingTimedOut()) {
    failHoming("HOME_TIMEOUT");
    return;
  }
  const float moved = fabsf((atomicStepPosition() - g_homeStartSteps) * STEP_MM);
  if (moved > HOME_MAX_TRAVEL_MM + 100.0f) {
    failHoming("HOME_DISTANCE");
    return;
  }
  switch (g_homeState) {
    case HomeState::SEEK_LEFT:
      if (leftLimitActive()) {
        immediateStop();
        atomicSetStepPosition(0);
        beginHomeState(HomeState::BACK_OFF_LEFT);
        g_targetVelocity = HOME_BACKOFF_SPEED_MM_S;
      } else {
        g_targetVelocity = -HOME_SEEK_SPEED_MM_S;
      }
      break;
    case HomeState::BACK_OFF_LEFT:
      if (!leftLimitActive() && positionMm() >= HOME_BACKOFF_MM) {
        beginHomeState(HomeState::SEEK_RIGHT);
        g_targetVelocity = HOME_SEEK_SPEED_MM_S;
      } else {
        g_targetVelocity = HOME_BACKOFF_SPEED_MM_S;
      }
      break;
    case HomeState::SEEK_RIGHT:
      if (rightLimitActive()) {
        immediateStop();
        g_railLengthMm = positionMm();
        if (g_railLengthMm < HOME_MIN_TRAVEL_MM || g_railLengthMm > HOME_MAX_TRAVEL_MM) {
          failHoming("HOME_RANGE_INVALID");
          return;
        }
        if (!g_persistedConfigValid ||
            fabsf(g_railLengthMm - g_persistedRailLengthMm) > 0.25f) {
          PersistentConfig persisted{CONFIG_MAGIC, g_railLengthMm};
          EEPROM.put(0, persisted);
          EEPROM.commit();
          g_persistedRailLengthMm = g_railLengthMm;
          g_persistedConfigValid = true;
        }
        beginHomeState(HomeState::MOVE_CENTER);
        g_targetVelocity = -HOME_SEEK_SPEED_MM_S;
      } else {
        g_targetVelocity = HOME_SEEK_SPEED_MM_S;
      }
      break;
    case HomeState::MOVE_CENTER: {
      const float error = g_railLengthMm * 0.5f - positionMm();
      if (fabsf(error) < 0.6f && fabsf(g_currentVelocity) < 4.0f) {
        immediateStop();
        g_homeState = HomeState::NONE;
        g_homed = true;
        g_watchdogFault = false;
        g_fault[0] = '\0';
        Serial.println("OK:HOME_COMPLETE");
      } else {
        const float brakingSpeed = sqrtf(2.0f * ACCEL_MM_S2 * fabsf(error));
        g_targetVelocity = (error > 0.0f ? 1.0f : -1.0f) * min(HOME_SEEK_SPEED_MM_S, brakingSpeed);
      }
      break;
    }
    default:
      break;
  }
}

void updatePositionMode() {
  if (!g_positionMode || !g_homed || g_homeState != HomeState::NONE) return;
  const float error = g_positionTargetMm - positionMm();
  if (fabsf(error) < 0.5f && fabsf(g_currentVelocity) < 3.0f) {
    g_positionMode = false;
    g_targetVelocity = 0.0f;
    Serial.println("OK:POSITION_REACHED");
    return;
  }
  const float speed = min(POSITION_MODE_MAX_SPEED_MM_S, sqrtf(2.0f * ACCEL_MM_S2 * fabsf(error)));
  g_targetVelocity = (error > 0.0f ? 1.0f : -1.0f) * speed;
}

void updateMotion() {
  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - g_lastMotionUpdateUs;
  if (elapsedUs < 1000u) return;
  g_lastMotionUpdateUs = nowUs;
  const float dt = min(elapsedUs * 1.0e-6f, 0.02f);

  if (g_estop) {
    immediateStop();
    return;
  }
  if (g_homeState == HomeState::NONE && !heartbeatFresh() &&
      (fabsf(g_targetVelocity) > 0.01f || fabsf(g_currentVelocity) > 0.01f)) {
    g_watchdogFault = true;
    strncpy(g_fault, "WATCHDOG", sizeof(g_fault));
    g_targetVelocity = 0.0f;
    g_positionMode = false;
  }
  if (g_calibrationJogDeadlineMs != 0 &&
      static_cast<int32_t>(millis() - g_calibrationJogDeadlineMs) >= 0) {
    g_calibrationJogDeadlineMs = 0;
    g_targetVelocity = 0.0f;
  }

  float requested = constrain(g_targetVelocity, -OPERATING_MAX_SPEED_MM_S, OPERATING_MAX_SPEED_MM_S);
  if (g_homeState == HomeState::NONE) requested = maximumSafeVelocity(requested);
  const float maxChange = ACCEL_MM_S2 * dt;
  if (g_currentVelocity < requested) {
    g_currentVelocity = min(requested, g_currentVelocity + maxChange);
  } else if (g_currentVelocity > requested) {
    g_currentVelocity = max(requested, g_currentVelocity - maxChange);
  }
  if (fabsf(g_currentVelocity) < 0.02f && fabsf(requested) < 0.02f) g_currentVelocity = 0.0f;

  if (g_homeState == HomeState::NONE) {
    if (leftLimitActive() && g_currentVelocity < 0.0f) {
      g_homed = false;
      latchFault("LEFT_LIMIT");
      return;
    }
    if (rightLimitActive() && g_currentVelocity > 0.0f) {
      g_homed = false;
      latchFault("RIGHT_LIMIT");
      return;
    }
  }
  applyCurrentVelocity();
  if (!g_homed && g_homeState == HomeState::NONE && g_calibrationJogDeadlineMs == 0 &&
      fabsf(g_targetVelocity) < 0.02f && fabsf(g_currentVelocity) < 0.02f) {
    driverEnable(false);
  }
}

const char* homeStateName() {
  switch (g_homeState) {
    case HomeState::NONE: return "none";
    case HomeState::SEEK_LEFT: return "seek_left";
    case HomeState::BACK_OFF_LEFT: return "back_off_left";
    case HomeState::SEEK_RIGHT: return "seek_right";
    case HomeState::MOVE_CENTER: return "move_center";
    case HomeState::FAILED: return "failed";
  }
  return "unknown";
}

void printStatus() {
  Serial.print("STATUS:{\"firmware\":\""); Serial.print(FIRMWARE_VERSION); Serial.print("\"");
  Serial.print(",\"homed\":"); Serial.print(g_homed ? "true" : "false");
  Serial.print(",\"estop\":"); Serial.print(g_estop ? "true" : "false");
  Serial.print(",\"fault\":\""); Serial.print(g_fault); Serial.print("\"");
  Serial.print(",\"position_mm\":"); Serial.print(positionMm(), 3);
  Serial.print(",\"velocity_mm_s\":"); Serial.print(g_currentVelocity, 3);
  Serial.print(",\"target_velocity_mm_s\":"); Serial.print(g_targetVelocity, 3);
  Serial.print(",\"left_limit\":"); Serial.print(leftLimitActive() ? "true" : "false");
  Serial.print(",\"right_limit\":"); Serial.print(rightLimitActive() ? "true" : "false");
  Serial.print(",\"watchdog_ok\":"); Serial.print(heartbeatFresh() ? "true" : "false");
  Serial.print(",\"rail_length_mm\":"); Serial.print(g_railLengthMm, 3);
  Serial.print(",\"home_state\":\""); Serial.print(homeStateName()); Serial.println("\"}");
}

void handleCommand(char* command) {
  while (*command == ' ') ++command;
  if (strcmp(command, "HEARTBEAT") == 0) {
    g_lastHeartbeatMs = millis();
    if (g_watchdogFault && fabsf(g_currentVelocity) < 0.01f) {
      g_watchdogFault = false;
      if (strcmp(g_fault, "WATCHDOG") == 0) g_fault[0] = '\0';
    }
    Serial.println("OK:HEARTBEAT");
    return;
  }
  if (strcmp(command, "STATUS?") == 0) {
    printStatus();
    return;
  }
  if (strcmp(command, "VERSION?") == 0) {
    Serial.print("VERSION:");
    Serial.println(FIRMWARE_VERSION);
    return;
  }
  if (strcmp(command, "STOP") == 0) {
    g_targetVelocity = 0.0f;
    g_positionMode = false;
    g_calibrationJogDeadlineMs = 0;
    Serial.println("OK:STOPPING");
    return;
  }
  if (strcmp(command, "ESTOP") == 0) {
    latchEstop("ESTOP");
    Serial.println("OK:ESTOP_LATCHED");
    return;
  }
  if (strcmp(command, "CLEAR_ESTOP") == 0) {
    if (leftLimitActive() || rightLimitActive()) {
      Serial.println("ERR:LIMIT_ACTIVE");
      return;
    }
    g_estop = false;
    g_homed = false;
    g_fault[0] = '\0';
    // Keep ENA disabled until an explicit homing or commissioning jog command.
    driverEnable(false);
    Serial.println("OK:ESTOP_CLEARED_HOME_REQUIRED");
    return;
  }
  if (strcmp(command, "CLEAR_FAULT") == 0) {
    if (g_estop || leftLimitActive() || rightLimitActive()) {
      Serial.println("ERR:UNSAFE_TO_CLEAR");
      return;
    }
    g_fault[0] = '\0';
    g_watchdogFault = false;
    if (g_homeState == HomeState::FAILED) g_homeState = HomeState::NONE;
    Serial.println("OK:FAULT_CLEARED");
    return;
  }
  if (strcmp(command, "FULL_HOME") == 0 || strcmp(command, "HOME") == 0) {
    startHoming();
    return;
  }
  // Commissioning-only dead-man jog. It permits direction verification before
  // homing but is capped at 20 mm/s and expires after 250 ms unless repeated.
  constexpr size_t calJogPrefixLength = sizeof(CAL_JOG_PREFIX) - 1;
  if (strncmp(command, CAL_JOG_PREFIX, calJogPrefixLength) == 0) {
    char* end = nullptr;
    const char* value = command + calJogPrefixLength;
    const float requested = strtof(value, &end);
    if (end == value || *end != '\0' || !isfinite(requested) ||
        fabsf(requested) > 20.0f) {
      Serial.println("ERR:BAD_CAL_JOG");
      return;
    }
    if (g_estop || g_homed || g_homeState != HomeState::NONE || !heartbeatFresh()) {
      Serial.println("ERR:CAL_JOG_NOT_ALLOWED");
      return;
    }
    driverEnable(true);
    g_positionMode = false;
    g_targetVelocity = requested;
    g_calibrationJogDeadlineMs = millis() + 250u;
    Serial.println("OK:CAL_JOG");
    return;
  }
  if (strncmp(command, "VEL:", 4) == 0) {
    char* end = nullptr;
    const float requested = strtof(command + 4, &end);
    if (end == command + 4 || *end != '\0' || !isfinite(requested)) {
      Serial.println("ERR:BAD_VELOCITY");
      return;
    }
    if (fabsf(requested) > ABSOLUTE_MAX_SPEED_MM_S) {
      latchEstop("ABS_SPEED_COMMAND");
      Serial.println("ERR:ABSOLUTE_SPEED_LIMIT");
      return;
    }
    if (fabsf(requested) > OPERATING_MAX_SPEED_MM_S) {
      Serial.println("ERR:OPERATING_SPEED_LIMIT");
      return;
    }
    if (!g_homed || g_homeState != HomeState::NONE) {
      Serial.println("ERR:HOME_REQUIRED");
      return;
    }
    if (g_estop || !heartbeatFresh()) {
      Serial.println("ERR:NOT_SAFE");
      return;
    }
    g_positionMode = false;
    g_targetVelocity = requested;
    Serial.println("OK:VELOCITY");
    return;
  }
  // Compatibility for the previous position-oriented host protocol.
  if (strncmp(command, "POS:", 4) == 0) {
    char* end = nullptr;
    const float requested = strtof(command + 4, &end);
    if (end == command + 4 || *end != '\0' || !isfinite(requested)) {
      Serial.println("ERR:BAD_POSITION");
      return;
    }
    if (!g_homed || g_estop || !heartbeatFresh()) {
      Serial.println("ERR:NOT_SAFE");
      return;
    }
    g_positionTargetMm = constrain(requested, SOFT_MARGIN_MM, g_railLengthMm - SOFT_MARGIN_MM);
    g_positionMode = true;
    Serial.println("OK:POSITION");
    return;
  }
  Serial.println("ERR:UNKNOWN_COMMAND");
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\r') continue;
    if (value == '\n') {
      g_rxBuffer[g_rxLength] = '\0';
      if (g_rxLength > 0) handleCommand(g_rxBuffer);
      g_rxLength = 0;
      continue;
    }
    if (g_rxLength + 1 < sizeof(g_rxBuffer)) {
      g_rxBuffer[g_rxLength++] = value;
    } else {
      g_rxLength = 0;
      Serial.println("ERR:COMMAND_TOO_LONG");
    }
  }
}

void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_LIMIT_LEFT, INPUT_PULLUP);
  pinMode(PIN_LIMIT_RIGHT, INPUT_PULLUP);
  driverEnable(false);
  digitalWrite(PIN_DIR, DIR_RIGHT_LEVEL);

  gpio_set_function(PIN_STEP, GPIO_FUNC_PWM);
  gpio_set_outover(PIN_STEP, GPIO_OVERRIDE_LOW);
  g_pwmSlice = pwm_gpio_to_slice_num(PIN_STEP);
  g_pwmChannel = pwm_gpio_to_channel(PIN_STEP);
  pwm_config pwmConfig = pwm_get_default_config();
  pwm_config_set_wrap(&pwmConfig, PWM_LOW_SPEED_WRAP);
  pwm_init(g_pwmSlice, &pwmConfig, false);
  pwm_set_chan_level(g_pwmSlice, g_pwmChannel, 0);
  pwm_clear_irq(g_pwmSlice);
  pwm_set_irq_enabled(g_pwmSlice, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, onPwmWrap);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  EEPROM.begin(EEPROM_BYTES);
  PersistentConfig persisted{};
  EEPROM.get(0, persisted);
  if (persisted.magic == CONFIG_MAGIC && isfinite(persisted.railLengthMm) &&
      persisted.railLengthMm >= HOME_MIN_TRAVEL_MM && persisted.railLengthMm <= HOME_MAX_TRAVEL_MM) {
    g_railLengthMm = persisted.railLengthMm;
    g_persistedRailLengthMm = persisted.railLengthMm;
    g_persistedConfigValid = true;
  }

  Serial.begin(115200);
  const uint32_t serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < 1500) delay(10);
  g_lastMotionUpdateUs = micros();
  // Reassert the complete stopped/driver-disabled state after PWM setup.
  latchEstop("ESTOP");
  Serial.print("READY:");
  Serial.print(FIRMWARE_VERSION);
  Serial.println(":HOME_REQUIRED");
}

void loop() {
  pollSerial();
  updateHoming();
  updatePositionMode();
  updateMotion();
}

