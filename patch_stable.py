from pathlib import Path

root = Path("Firmware_Leonardo/brWheel_my")

mt = r'''#ifndef MT6701_COMPAT_H
#define MT6701_COMPAT_H

#include <Arduino.h>
#include <avr/io.h>

// Stable MT6701 reader for Leonardo.
// Uses bounded software I2C on D2/SDA and D3/SCL so a bus fault cannot
// block the FFB/USB/UART main loop indefinitely.
class AS5600L {
public:
  explicit AS5600L(uint8_t address = 0x06) : _address(address) {}

  bool begin() {
    // Wire.begin() is called by the original firmware. Disable AVR TWI here
    // because this wrapper owns D2/D3 using bounded software I2C.
#ifdef TWEN
    TWCR &= (uint8_t)~_BV(TWEN);
#endif
    releaseSDA();
    releaseSCL();
    delayMicroseconds(10);
    recoverBus();

    uint16_t raw;
    bool ok = readRaw(raw);
    if (ok) {
      _lastRaw = raw;
      _accum = 0;
      _zeroOffset = 0;
      _initialized = true;
      _consecutiveFailures = 0;
      _lastReadOK = true;
    }
    return ok;
  }

  void setFastFilter(uint8_t) {}
  void setSlowFilter(uint8_t) {}

  int32_t resetCumulativePosition(int32_t newPosition = 0) {
    uint16_t raw;
    if (readRaw(raw)) {
      _lastRaw = raw;
      _accum = 0;
      _zeroOffset = newPosition;
      _initialized = true;
    }
    return newPosition;
  }

  int32_t getCumulativePosition() {
    uint16_t raw;
    if (!readRaw(raw)) {
      return _zeroOffset + _accum; // keep last valid angle, never block
    }

    if (!_initialized) {
      _lastRaw = raw;
      _initialized = true;
      return _zeroOffset;
    }

    int16_t delta = (int16_t)raw - (int16_t)_lastRaw;
    if (delta > 8192) delta -= 16384;
    else if (delta < -8192) delta += 16384;

    _accum += delta;
    _lastRaw = raw;
    return _zeroOffset + _accum;
  }

  bool healthy() const {
    // One or two bad samples are tolerated. Three consecutive failed reads
    // force motor torque to zero until valid angle data returns.
    return _initialized && (_consecutiveFailures < 3);
  }

  bool lastReadOK() const { return _lastReadOK; }
  uint8_t consecutiveFailures() const { return _consecutiveFailures; }

private:
  static const uint8_t SDA_PIN = 2;
  static const uint8_t SCL_PIN = 3;
  static const uint16_t CLOCK_STRETCH_TIMEOUT_US = 120;

  static inline void driveSDALow() {
    digitalWrite(SDA_PIN, LOW);
    pinMode(SDA_PIN, OUTPUT);
  }

  static inline void releaseSDA() {
    pinMode(SDA_PIN, INPUT_PULLUP);
  }

  static inline void driveSCLLow() {
    digitalWrite(SCL_PIN, LOW);
    pinMode(SCL_PIN, OUTPUT);
  }

  static inline void releaseSCL() {
    pinMode(SCL_PIN, INPUT_PULLUP);
  }

  bool waitSCLHigh() {
    releaseSCL();
    uint32_t started = micros();
    while (digitalRead(SCL_PIN) == LOW) {
      if ((uint32_t)(micros() - started) >= CLOCK_STRETCH_TIMEOUT_US) {
        return false;
      }
    }
    return true;
  }

  void halfDelay() {
    delayMicroseconds(3); // ~100-150 kHz effective bus speed on AVR
  }

  bool startCondition() {
    releaseSDA();
    if (!waitSCLHigh()) return false;
    halfDelay();
    if (digitalRead(SDA_PIN) == LOW) return false;
    driveSDALow();
    halfDelay();
    driveSCLLow();
    return true;
  }

  void stopCondition() {
    driveSDALow();
    halfDelay();
    if (waitSCLHigh()) {
      halfDelay();
      releaseSDA();
      halfDelay();
    } else {
      releaseSDA();
      releaseSCL();
    }
  }

  bool writeByte(uint8_t value) {
    for (uint8_t i = 0; i < 8; ++i) {
      if (value & 0x80) releaseSDA();
      else driveSDALow();
      halfDelay();
      if (!waitSCLHigh()) return false;
      halfDelay();
      driveSCLLow();
      value <<= 1;
    }

    releaseSDA();
    halfDelay();
    if (!waitSCLHigh()) return false;
    bool ack = (digitalRead(SDA_PIN) == LOW);
    halfDelay();
    driveSCLLow();
    return ack;
  }

  bool readByte(uint8_t &value, bool ack) {
    value = 0;
    releaseSDA();
    for (uint8_t i = 0; i < 8; ++i) {
      value <<= 1;
      halfDelay();
      if (!waitSCLHigh()) return false;
      if (digitalRead(SDA_PIN)) value |= 1;
      halfDelay();
      driveSCLLow();
    }

    if (ack) driveSDALow();
    else releaseSDA();
    halfDelay();
    if (!waitSCLHigh()) return false;
    halfDelay();
    driveSCLLow();
    releaseSDA();
    return true;
  }

  void recoverBus() {
    releaseSDA();
    for (uint8_t i = 0; i < 9; ++i) {
      driveSCLLow();
      halfDelay();
      if (!waitSCLHigh()) break;
      halfDelay();
    }
    stopCondition();
  }

  void noteResult(bool ok) {
    _lastReadOK = ok;
    if (ok) {
      _consecutiveFailures = 0;
    } else {
      if (_consecutiveFailures < 255) ++_consecutiveFailures;
      recoverBus();
    }
  }

  bool readRaw(uint16_t &value) {
#ifdef TWEN
    TWCR &= (uint8_t)~_BV(TWEN);
#endif
    bool ok = false;
    uint8_t msb = 0, lsb = 0;

    do {
      if (!startCondition()) break;
      if (!writeByte((uint8_t)(_address << 1))) break;
      if (!writeByte((uint8_t)0x03)) break;

      if (!startCondition()) break; // repeated START
      if (!writeByte((uint8_t)((_address << 1) | 1))) break;
      if (!readByte(msb, true)) break;
      if (!readByte(lsb, false)) break;
      stopCondition();

      value = ((uint16_t)msb << 6) | ((uint16_t)(lsb & 0xFC) >> 2);
      ok = true;
    } while (0);

    if (!ok) stopCondition();
    noteResult(ok);
    return ok;
  }

  uint8_t _address;
  uint16_t _lastRaw = 0;
  int32_t _accum = 0;
  int32_t _zeroOffset = 0;
  bool _initialized = false;
  bool _lastReadOK = false;
  uint8_t _consecutiveFailures = 255;
};

#endif
'''
(root / "MT6701Compat.h").write_text(mt)

ino = root / "brWheel_my.ino"
s = ino.read_text()

# Global safety flag used by HoverboardUART. It is TRUE until valid angle data exists.
anchor = "s32v ffbs; // milos, instance of struct holding 2 axis FFB data"
if "bool mt6701SafeZero" not in s:
    if anchor not in s:
        raise SystemExit("ffbs anchor not found")
    s = s.replace(anchor, anchor + "\nbool mt6701SafeZero = true; // force motor torque off if MT6701 data is not healthy", 1)

# After every X-axis MT6701 read, update the fail-safe state.
readline = "turn.x = ROTATION_MID - as5600x.getCumulativePosition(); // MT6701 steering direction inverted"
replacement = readline + "\n      mt6701SafeZero = !as5600x.healthy();"
if replacement not in s:
    if readline not in s:
        raise SystemExit("inverted MT6701 read marker not found")
    s = s.replace(readline, replacement, 1)

ino.write_text(s)

uart = root / "HoverboardUART.ino"
u = uart.read_text()

if "#define HOVERBOARD_CMD_SLEW_STEP" not in u:
    u = u.replace(
        "#define HOVERBOARD_TORQUE_SCALE_PCT 100",
        "#define HOVERBOARD_TORQUE_SCALE_PCT 100\n"
        "#define HOVERBOARD_CMD_SLEW_STEP 60 // R5-character: faster torque detail while retaining spike limiting\n"
        "extern bool mt6701SafeZero;",
        1
    )

old = """  cmd = constrain(cmd, -HOVERBOARD_CMD_MAX, HOVERBOARD_CMD_MAX);
  if (abs(cmd) <= HOVERBOARD_DEADBAND) cmd = 0;

  // GD32 package uses normal mixer with STEER_COEFFICIENT=0 and SPEED_COEFFICIENT=1.0.
  // Therefore SPEED carries the torque command. Only MOTOR_RIGHT_ENA is enabled.
  HoverboardSend(0, (int16_t)cmd);"""
new = """  cmd = constrain(cmd, -HOVERBOARD_CMD_MAX, HOVERBOARD_CMD_MAX);
  if (abs(cmd) <= HOVERBOARD_DEADBAND) cmd = 0;

  static int16_t lastCmd = 0;

  // Sensor fail-safe: a bad MT6701 bus must never leave stale torque active.
  // UART stays alive and sends an immediate zero command while the sensor recovers.
  if (mt6701SafeZero) {
    lastCmd = 0;
    HoverboardSend(0, 0);
    return;
  }

  // Slew-limit torque command so a sudden centering/FFB step cannot create
  // an unnecessarily sharp current impulse on the hoverboard power stage.
  int32_t hi = (int32_t)lastCmd + HOVERBOARD_CMD_SLEW_STEP;
  int32_t lo = (int32_t)lastCmd - HOVERBOARD_CMD_SLEW_STEP;
  if (cmd > hi) cmd = hi;
  else if (cmd < lo) cmd = lo;
  lastCmd = (int16_t)cmd;

  // GD32 package uses normal mixer with STEER_COEFFICIENT=0 and SPEED_COEFFICIENT=1.0.
  // Therefore SPEED carries the torque command. Only MOTOR_RIGHT_ENA is enabled.
  HoverboardSend(0, lastCmd);"""
if new not in u:
    if old not in u:
        raise SystemExit("HoverboardSendTorque marker not found")
    u = u.replace(old, new, 1)

uart.write_text(u)

# Bump firmware version so saved defaults are refreshed cleanly.
cfg = root / "Config.h"
c = cfg.read_text()
c = c.replace("#define FIRMWARE_VERSION         0xFB", "#define FIRMWARE_VERSION         0xFC", 1)
cfg.write_text(c)

print("Applied stable MT6701 bounded-I2C fail-safe and UART torque slew limiting")
