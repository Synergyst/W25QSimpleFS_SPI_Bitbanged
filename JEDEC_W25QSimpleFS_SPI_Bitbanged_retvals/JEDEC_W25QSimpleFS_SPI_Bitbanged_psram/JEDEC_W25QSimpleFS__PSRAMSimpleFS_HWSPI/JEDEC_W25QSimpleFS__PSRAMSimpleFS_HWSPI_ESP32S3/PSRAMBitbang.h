/*
  PSRAMBitbang.h - Single-header, header-only bit-banged SPI helper for PSRAM
  (unchanged from prior release; included for completeness)
*/

#ifndef PSRAMBITBANG_H
#define PSRAMBITBANG_H

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

#ifndef PSRAM_PIN_CS
#define PSRAM_PIN_CS 9
#endif
#ifndef PSRAM_PIN_MISO
#define PSRAM_PIN_MISO 12
#endif
#ifndef PSRAM_PIN_MOSI
#define PSRAM_PIN_MOSI 11
#endif
#ifndef PSRAM_PIN_SCK
#define PSRAM_PIN_SCK 10
#endif

#ifndef PSRAM_CMD_READ_JEDEC
#define PSRAM_CMD_READ_JEDEC 0x9F
#endif
#ifndef PSRAM_CMD_READ_03
#define PSRAM_CMD_READ_03 0x03
#endif
#ifndef PSRAM_CMD_WRITE_02
#define PSRAM_CMD_WRITE_02 0x02
#endif
#ifndef PSRAM_CMD_WRITE_ENABLE
#define PSRAM_CMD_WRITE_ENABLE 0x06
#endif

class PSRAMBitbang {
public:
  PSRAMBitbang(uint8_t pin_cs = PSRAM_PIN_CS,
               uint8_t pin_miso = PSRAM_PIN_MISO,
               uint8_t pin_mosi = PSRAM_PIN_MOSI,
               uint8_t pin_sck = PSRAM_PIN_SCK)
    : _pinCS(pin_cs),
      _pinMISO(pin_miso),
      _pinMOSI(pin_mosi),
      _pinSCK(pin_sck),
      _pinIO2(255),
      _pinIO3(255),
      _useQuad(false),
      _halfCycleDelayUs(1) {}

  inline void begin() {
    pinMode(_pinCS, OUTPUT);
    pinMode(_pinMOSI, OUTPUT);
    pinMode(_pinSCK, OUTPUT);
    pinMode(_pinMISO, INPUT);
    digitalWrite(_pinCS, HIGH);
    digitalWrite(_pinSCK, LOW);
    digitalWrite(_pinMOSI, LOW);
    if (_pinIO2 != 255) pinMode(_pinIO2, INPUT);
    if (_pinIO3 != 255) pinMode(_pinIO3, INPUT);
  }

  inline void setClockDelayUs(uint8_t halfCycleDelayUs) {
    _halfCycleDelayUs = halfCycleDelayUs;
  }
  inline void setExtraDataPins(uint8_t io2, uint8_t io3) {
    _pinIO2 = io2;
    _pinIO3 = io3;
    if (_pinIO2 != 255) pinMode(_pinIO2, INPUT);
    if (_pinIO3 != 255) pinMode(_pinIO3, INPUT);
  }
  inline void setModeQuad(bool enable) {
    _useQuad = enable;
  }

  inline void csLow() {
    digitalWrite(_pinCS, LOW);
  }
  inline void csHigh() {
    digitalWrite(_pinCS, HIGH);
  }

  inline uint8_t transfer(uint8_t tx) {
    uint8_t rx = 0;
    for (int bit = 7; bit >= 0; --bit) {
      digitalWrite(_pinMOSI, (tx >> bit) & 1);
      if (_halfCycleDelayUs) delayMicroseconds(_halfCycleDelayUs);
      digitalWrite(_pinSCK, HIGH);
      rx <<= 1;
      if (digitalRead(_pinMISO)) rx |= 1;
      if (_halfCycleDelayUs) delayMicroseconds(_halfCycleDelayUs);
      digitalWrite(_pinSCK, LOW);
    }
    return rx;
  }

  inline void transfer(const uint8_t *txbuf, uint8_t *rxbuf, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      uint8_t t = txbuf ? txbuf[i] : 0x00;
      uint8_t r = transfer(t);
      if (rxbuf) rxbuf[i] = r;
    }
  }

  inline void cmdRead(const uint8_t *cmd, size_t cmdLen, uint8_t *resp, size_t respLen) {
    csLow();
    if (cmd && cmdLen) transfer(cmd, nullptr, cmdLen);
    if (respLen) transfer(nullptr, resp, respLen);
    csHigh();
  }

  inline void readJEDEC(uint8_t *out, size_t len) {
    uint8_t cmd = PSRAM_CMD_READ_JEDEC;
    cmdRead(&cmd, 1, out, len);
  }

  inline bool readData03(uint32_t addr, uint8_t *buf, size_t len) {
    uint8_t cmd[4];
    cmd[0] = PSRAM_CMD_READ_03;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    csLow();
    transfer(cmd, nullptr, 4);
    transfer(nullptr, buf, len);
    csHigh();
    return true;
  }

  inline void writeEnable() {
    uint8_t cmd = PSRAM_CMD_WRITE_ENABLE;
    csLow();
    transfer(cmd);
    csHigh();
  }

  inline bool writeData02(uint32_t addr, const uint8_t *buf, size_t len, bool needsWriteEnable = false) {
    if (!buf || len == 0) return true;
    if (needsWriteEnable) writeEnable();
    uint8_t cmd[4];
    cmd[0] = PSRAM_CMD_WRITE_02;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    csLow();
    transfer(cmd, nullptr, 4);
    transfer(buf, nullptr, len);
    csHigh();
    return true;
  }

  inline void rawMisoScan(uint8_t *out, size_t len) {
    csLow();
    for (size_t i = 0; i < len; ++i) out[i] = transfer(0x00);
    csHigh();
  }

  inline bool quadAvailable() const {
    return _useQuad && (_pinIO2 != 255) && (_pinIO3 != 255);
  }

  inline bool quadWriteByte(uint8_t tx) {
    if (!quadAvailable()) return false;
    pinMode(_pinMOSI, OUTPUT);
    pinMode(_pinMISO, OUTPUT);
    pinMode(_pinIO2, OUTPUT);
    pinMode(_pinIO3, OUTPUT);
    uint8_t nibbles[2] = { (uint8_t)(tx >> 4), (uint8_t)(tx & 0x0F) };
    for (int ni = 0; ni < 2; ++ni) {
      uint8_t nib = nibbles[ni];
      for (int b = 3; b >= 0; --b) {
        digitalWrite(_pinMOSI, (nib >> 0) & 1);
        digitalWrite(_pinMISO, (nib >> 1) & 1);
        digitalWrite(_pinIO2, (nib >> 2) & 1);
        digitalWrite(_pinIO3, (nib >> 3) & 1);
        if (_halfCycleDelayUs) delayMicroseconds(_halfCycleDelayUs);
        digitalWrite(_pinSCK, HIGH);
        if (_halfCycleDelayUs) delayMicroseconds(_halfCycleDelayUs);
        digitalWrite(_pinSCK, LOW);
      }
    }
    pinMode(_pinMISO, INPUT);
    return true;
  }

  inline bool quadReadByte(uint8_t &out) {
    if (!quadAvailable()) return false;
    pinMode(_pinMOSI, INPUT);
    pinMode(_pinMISO, INPUT);
    pinMode(_pinIO2, INPUT);
    pinMode(_pinIO3, INPUT);
    uint8_t result = 0;
    for (int ni = 0; ni < 2; ++ni) {
      uint8_t nib = 0;
      for (int b = 3; b >= 0; --b) {
        if (_halfCycleDelayUs) delayMicroseconds(_halfCycleDelayUs);
        digitalWrite(_pinSCK, HIGH);
        uint8_t bit0 = digitalRead(_pinMOSI) ? 1 : 0;
        uint8_t bit1 = digitalRead(_pinMISO) ? 1 : 0;
        uint8_t bit2 = digitalRead(_pinIO2) ? 1 : 0;
        uint8_t bit3 = digitalRead(_pinIO3) ? 1 : 0;
        digitalWrite(_pinSCK, LOW);
        nib |= (bit0 << 0) | (bit1 << 1) | (bit2 << 2) | (bit3 << 3);
      }
      if (ni == 0) result = (uint8_t)(nib << 4);
      else result |= (nib & 0x0F);
    }
    out = result;
    return true;
  }

private:
  uint8_t _pinCS, _pinMISO, _pinMOSI, _pinSCK;
  uint8_t _pinIO2, _pinIO3;
  bool _useQuad;
  volatile uint8_t _halfCycleDelayUs;
};

#endif  // PSRAMBITBANG_H