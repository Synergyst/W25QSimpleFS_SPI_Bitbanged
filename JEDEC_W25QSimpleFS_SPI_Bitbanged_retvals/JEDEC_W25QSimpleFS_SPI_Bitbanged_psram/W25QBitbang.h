#pragma once
/*
  W25QBitbang.h - Hardware or bit-banged SPI driver for Winbond W25Q-series (mode 0).
  If W25Q_USE_HW_SPI is defined, this class uses Arduino RP2040 SPI (Philhower core).
  Otherwise it falls back to the original bit-bang implementation below.
*/

#ifdef W25Q_USE_HW_SPI

#include <Arduino.h>
#include <SPI.h>

// Choose which SPI instance to use for flash (must match PSRAM to share the bus).
#ifndef W25Q_SPI_INSTANCE
#define W25Q_SPI_INSTANCE SPI1
#endif

// Default SPI clock for flash (override if needed)
#ifndef W25Q_SPI_CLOCK_HZ
#define W25Q_SPI_CLOCK_HZ 20000000UL  // 20 MHz is a safe default; many W25Q parts can go higher
#endif

class W25QBitbang {
public:
  // Keep same constructor signature: (miso, cs, sck, mosi)
  W25QBitbang(uint8_t pinMiso, uint8_t pinCs, uint8_t pinSck, uint8_t pinMosi)
    : _miso(pinMiso), _cs(pinCs), _sck(pinSck), _mosi(pinMosi),
      _settings(W25Q_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0) {}

  void begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    // Map pins to this SPI instance and start it
    W25Q_SPI_INSTANCE.setRX(_miso);
    W25Q_SPI_INSTANCE.setTX(_mosi);
    W25Q_SPI_INSTANCE.setSCK(_sck);
    W25Q_SPI_INSTANCE.begin();
  }

  // JEDEC ID (0x9F). Returns total capacity in bytes (2^capCode) or 0 on error.
  uint32_t readJEDEC(uint8_t &mfr, uint8_t &memType, uint8_t &capCode) {
    csLow();
    beginTx();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x9F);
    mfr = W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
    memType = W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
    capCode = W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
    endTx();
    csHigh();

    if (capCode < 32) return (uint32_t)1UL << capCode;
    return 0;
  }

  // Status register-1 (0x05): bit0=WIP, bit1=WEL
  uint8_t readStatus1() {
    csLow();
    beginTx();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x05);
    uint8_t v = W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
    endTx();
    csHigh();
    return v;
  }

  bool isBusy() {
    return (readStatus1() & 0x01) != 0;
  }

  bool waitWhileBusy(uint32_t timeoutMs = 5000) {
    uint32_t t0 = millis();
    while (isBusy()) {
      if ((millis() - t0) > timeoutMs) return false;
      yield();
    }
    return true;
  }

  bool writeEnable(uint32_t confirmTimeoutMs = 50) {
    csLow();
    beginTx();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x06);
    endTx();
    csHigh();

    uint32_t t0 = millis();
    while ((readStatus1() & 0x02) == 0) {
      if ((millis() - t0) > confirmTimeoutMs) return false;
      yield();
    }
    return true;
  }

  // Linear read (0x03)
  size_t readData(uint32_t addr, uint8_t *buf, size_t len) {
    if (!buf || len == 0) return 0;
    csLow();
    beginTx();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x03);
    sendAddr24(addr);
    for (size_t i = 0; i < len; ++i) buf[i] = W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
    endTx();
    csHigh();
    return len;
  }

  // Optional faster read (0x0B) with one dummy byte
  size_t readDataFast0B(uint32_t addr, uint8_t *buf, size_t len) {
    if (!buf || len == 0) return 0;
    csLow();
    beginTx();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x0B);
    sendAddr24(addr);
    (void)W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);  // dummy
    for (size_t i = 0; i < len; ++i) buf[i] = W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
    endTx();
    csHigh();
    return len;
  }

  // Page program (0x02), auto-split at 256-byte boundaries
  bool pageProgram(uint32_t addr, const uint8_t *data, size_t len,
                   uint32_t chunkTimeoutMs = 10) {
    if (!data || len == 0) return true;
    size_t off = 0;
    while (off < len) {
      size_t pageOff = (addr & 0xFF);
      size_t pageSpace = 256 - pageOff;
      size_t chunk = (len - off < pageSpace) ? (len - off) : pageSpace;
      if (!writeEnable()) return false;

      csLow();
      beginTx();
      W25Q_SPI_INSTANCE.transfer((uint8_t)0x02);
      sendAddr24(addr);
      for (size_t i = 0; i < chunk; ++i) W25Q_SPI_INSTANCE.transfer(data[off + i]);
      endTx();
      csHigh();

      if (!waitWhileBusy(chunkTimeoutMs)) return false;
      addr += chunk;
      off += chunk;
    }
    return true;
  }

  bool sectorErase4K(uint32_t addr, uint32_t timeoutMs = 4000) {
    if (!writeEnable()) return false;
    csLow();
    beginTx();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x20);
    sendAddr24(addr);
    endTx();
    csHigh();
    return waitWhileBusy(timeoutMs);
  }

  bool chipErase(uint32_t timeoutMs = 180000) {
    if (!writeEnable()) return false;
    csLow();
    beginTx();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0xC7);  // or 0x60
    endTx();
    csHigh();
    return waitWhileBusy(timeoutMs);
  }

private:
  uint8_t _miso, _cs, _sck, _mosi;
  SPISettings _settings;

  inline void csLow() {
    digitalWrite(_cs, LOW);
  }
  inline void csHigh() {
    digitalWrite(_cs, HIGH);
  }
  inline void beginTx() {
    W25Q_SPI_INSTANCE.beginTransaction(_settings);
  }
  inline void endTx() {
    W25Q_SPI_INSTANCE.endTransaction();
  }

  inline void sendAddr24(uint32_t addr) {
    W25Q_SPI_INSTANCE.transfer((uint8_t)(addr >> 16));
    W25Q_SPI_INSTANCE.transfer((uint8_t)(addr >> 8));
    W25Q_SPI_INSTANCE.transfer((uint8_t)(addr >> 0));
  }
};

#else  // W25Q_USE_HW_SPI

// ------- Original bit-bang implementation follows (unchanged) -------
#include <Arduino.h>
#include <inttypes.h>

#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_GENERIC_RP2040)) && !defined(BB_USE_RP2040_SIO)
#define BB_USE_RP2040_SIO 1
#endif

#ifdef BB_USE_RP2040_SIO
#include "hardware/structs/sio.h"
#endif

// Bit-banged SPI driver for Winbond W25Q-series (mode 0).
class W25QBitbang {
public:
  W25QBitbang(uint8_t pinMiso, uint8_t pinCs, uint8_t pinSck, uint8_t pinMosi)
    : _miso(pinMiso), _cs(pinCs), _sck(pinSck), _mosi(pinMosi) {
#ifdef BB_USE_RP2040_SIO
    _maskMISO = _maskCS = _maskSCK = _maskMOSI = 0;
#endif
  }

  void begin() {
    pinMode(_cs, OUTPUT);
    pinMode(_sck, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    digitalWrite(_cs, HIGH);
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, LOW);

#ifdef BB_USE_RP2040_SIO
    // Precompute masks for fast SIO operations (assumes GPIO numbers < 32)
    _maskMISO = (1u << _miso);
    _maskCS = (1u << _cs);
    _maskSCK = (1u << _sck);
    _maskMOSI = (1u << _mosi);
#endif
  }

  // JEDEC ID (0x9F). Returns total capacity in bytes (2^capCode) or 0 on error.
  uint32_t readJEDEC(uint8_t &mfr, uint8_t &memType, uint8_t &capCode) {
    csLow();
    xfer(0x9F);
    mfr = xfer(0x00);
    memType = xfer(0x00);
    capCode = xfer(0x00);
    csHigh();
    if (capCode < 32) return (uint32_t)1UL << capCode;
    return 0;
  }

  // Status register-1 (0x05): bit0=WIP, bit1=WEL
  uint8_t readStatus1() {
    csLow();
    xfer(0x05);
    uint8_t v = xfer(0x00);
    csHigh();
    return v;
  }

  bool isBusy() {
    return (readStatus1() & 0x01) != 0;
  }

  bool waitWhileBusy(uint32_t timeoutMs = 5000) {
    uint32_t t0 = millis();
    while (isBusy()) {
      if ((millis() - t0) > timeoutMs) return false;
      yield();
    }
    return true;
  }

  bool writeEnable(uint32_t confirmTimeoutMs = 50) {
    csLow();
    xfer(0x06);
    csHigh();
    uint32_t t0 = millis();
    while ((readStatus1() & 0x02) == 0) {
      if ((millis() - t0) > confirmTimeoutMs) return false;
      yield();
    }
    return true;
  }

  // Linear read (0x03)
  size_t readData(uint32_t addr, uint8_t *buf, size_t len) {
    if (!buf || len == 0) return 0;
    csLow();
    xfer(0x03);
    sendAddr24(addr);
    for (size_t i = 0; i < len; ++i) buf[i] = xfer(0x00);
    csHigh();
    return len;
  }

  // Optional faster read (0x0B) with one dummy byte
  size_t readDataFast0B(uint32_t addr, uint8_t *buf, size_t len) {
    if (!buf || len == 0) return 0;
    csLow();
    xfer(0x0B);
    sendAddr24(addr);
    xfer(0x00);  // dummy byte
    for (size_t i = 0; i < len; ++i) buf[i] = xfer(0x00);
    csHigh();
    return len;
  }

  // Page program (0x02), auto-splits at 256-byte boundaries
  bool pageProgram(uint32_t addr, const uint8_t *data, size_t len,
                   uint32_t chunkTimeoutMs = 10) {
    if (!data || len == 0) return true;
    size_t off = 0;
    while (off < len) {
      size_t pageOff = (addr & 0xFF);
      size_t pageSpace = 256 - pageOff;
      size_t chunk = (len - off < pageSpace) ? (len - off) : pageSpace;
      if (!writeEnable()) return false;
      csLow();
      xfer(0x02);
      sendAddr24(addr);
      for (size_t i = 0; i < chunk; ++i) xfer(data[off + i]);
      csHigh();
      if (!waitWhileBusy(chunkTimeoutMs)) return false;
      addr += chunk;
      off += chunk;
    }
    return true;
  }

  bool sectorErase4K(uint32_t addr, uint32_t timeoutMs = 4000) {
    if (!writeEnable()) return false;
    csLow();
    xfer(0x20);
    sendAddr24(addr);
    csHigh();
    return waitWhileBusy(timeoutMs);
  }

  bool chipErase(uint32_t timeoutMs = 180000) {
    if (!writeEnable()) return false;
    csLow();
    xfer(0xC7);
    csHigh();  // or 0x60
    return waitWhileBusy(timeoutMs);
  }

private:
  uint8_t _miso, _cs, _sck, _mosi;

  inline void csLow() {
#ifdef BB_USE_RP2040_SIO
    sio_hw->gpio_clr = _maskCS;
#else
    digitalWrite(_cs, LOW);
#endif
  }

  inline void csHigh() {
#ifdef BB_USE_RP2040_SIO
    sio_hw->gpio_set = _maskCS;
#else
    digitalWrite(_cs, HIGH);
#endif
  }

  // Fast-path transfer using RP2040 SIO if available; otherwise portable digital I/O.
  inline uint8_t xfer(uint8_t outByte) {
#ifdef BB_USE_RP2040_SIO
    uint8_t inByte = 0;
    for (int8_t bit = 7; bit >= 0; --bit) {
      if (outByte & (1u << bit)) sio_hw->gpio_set = _maskMOSI;
      else sio_hw->gpio_clr = _maskMOSI;
      sio_hw->gpio_set = _maskSCK;
      inByte = (uint8_t)((inByte << 1) | ((sio_hw->gpio_in & _maskMISO) ? 1u : 0u));
      sio_hw->gpio_clr = _maskSCK;
    }
    return inByte;
#else
    uint8_t inByte = 0;
    for (int8_t bit = 7; bit >= 0; --bit) {
      digitalWrite(_mosi, (outByte >> bit) & 0x01);
      digitalWrite(_sck, HIGH);
      inByte = (uint8_t)((inByte << 1) | (digitalRead(_miso) & 0x01));
      digitalWrite(_sck, LOW);
    }
    return inByte;
#endif
  }

  inline void sendAddr24(uint32_t addr) {
    xfer((uint8_t)(addr >> 16));
    xfer((uint8_t)(addr >> 8));
    xfer((uint8_t)(addr >> 0));
  }

#ifdef BB_USE_RP2040_SIO
  // Precomputed masks for RP2040 SIO operations (1 << gpio)
  uint32_t _maskMISO = 0, _maskCS = 0, _maskSCK = 0, _maskMOSI = 0;
#endif
};

#endif  // W25Q_USE_HW_SPI