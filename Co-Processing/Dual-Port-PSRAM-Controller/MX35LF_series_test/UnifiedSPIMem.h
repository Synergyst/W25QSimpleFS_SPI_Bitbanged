#pragma once
/*
  UnifiedSPIMem.h
  Combines:
    - MX35LF (SPI-NAND) HW-SPI helper
    - W25QBitbang (NOR) with HW-SPI or bit-bang selectable
    - PSRAMBitbang (bit-bang + RP2040 SIO fast path)
  Unified API:
    - UnifiedSpiMem::Manager scans one or many CS pins and stores results
    - Holds a CS list and forces all CS high before probing (prevents bus contention)
    - Provides reservation-aware open/release:
        * openAuto(): pick first available device by priority (PSRAM, NOR, NAND)
        * openPreferred(type): pick first available device of requested type
        * openByIndex(), openByType() variants
        * release(handle) frees reservation and deletes device handle
    - Device pooling:
        * Create a pool (all devices or by type), open/release from pool
    - PSRAM retention:
        * setPreservePsramContents(true) to avoid PSRAM reset during identification
  Defaults:
    UNIFIED_SPI_INSTANCE = SPI1
    UNIFIED_SPI_CLOCK_HZ = 8 MHz
    UNIFIED_MAX_DETECTED = 16
    UNIFIED_MAX_CS       = 16
  Notes:
    - Single-I/O only (no QSPI/QPI paths).
    - No OTP operations are implemented.
*/
#include <Arduino.h>
#include <SPI.h>
#include <string.h>
#include <initializer_list>
// --------------------------- Common defaults ---------------------------
#ifndef UNIFIED_SPI_INSTANCE
#define UNIFIED_SPI_INSTANCE SPI1
#endif
#ifndef UNIFIED_SPI_CLOCK_HZ
#define UNIFIED_SPI_CLOCK_HZ 104000000UL
#endif
#ifndef UNIFIED_MAX_DETECTED
#define UNIFIED_MAX_DETECTED 16
#endif
#ifndef UNIFIED_MAX_CS
#define UNIFIED_MAX_CS 16
#endif
// Bind W25Q HW-SPI defaults to unified defaults (unless user overrides)
#ifndef W25Q_USE_HW_SPI
#define W25Q_USE_HW_SPI 1
#endif
#ifndef W25Q_SPI_INSTANCE
#define W25Q_SPI_INSTANCE UNIFIED_SPI_INSTANCE
#endif
#ifndef W25Q_SPI_CLOCK_HZ
#define W25Q_SPI_CLOCK_HZ UNIFIED_SPI_CLOCK_HZ
#endif
// Bind MX35 HW-SPI defaults to unified defaults (unless user overrides)
#ifndef MX35_SPI_INSTANCE
#define MX35_SPI_INSTANCE UNIFIED_SPI_INSTANCE
#endif
#ifndef MX35_SPI_CLOCK_HZ
#define MX35_SPI_CLOCK_HZ UNIFIED_SPI_CLOCK_HZ
#endif
// --------------------------- MX35LF (SPI-NAND) ---------------------------
class MX35LF {
public:
  struct IdInfo {
    uint8_t mid;
    uint8_t did1;
    uint8_t did2;
    uint64_t totalBytes;
    uint32_t pageSize;
    uint32_t spareSize;
  };
  MX35LF(uint8_t pinMiso, uint8_t pinCs, uint8_t pinSck, uint8_t pinMosi)
    : _miso(pinMiso), _cs(pinCs), _sck(pinSck), _mosi(pinMosi),
      _settings(MX35_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0) {}
  void begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    MX35_SPI_INSTANCE.setRX(_miso);
    MX35_SPI_INSTANCE.setTX(_mosi);
    MX35_SPI_INSTANCE.setSCK(_sck);
    MX35_SPI_INSTANCE.begin();
  }
  void setClock(uint32_t hz) {
    _settings = SPISettings(hz, MSBFIRST, SPI_MODE0);
  }
  bool reset(uint32_t timeoutMs = 50) {
    csLow();
    beginTx();
    MX35_SPI_INSTANCE.transfer((uint8_t)0xFF);
    endTx();
    csHigh();
    return waitReady(timeoutMs);
  }
  uint8_t getFeature(uint8_t addr) {
    uint8_t v = 0;
    csLow();
    beginTx();
    MX35_SPI_INSTANCE.transfer((uint8_t)0x0F);
    MX35_SPI_INSTANCE.transfer(addr);
    v = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);
    endTx();
    csHigh();
    return v;
  }
  void setFeature(uint8_t addr, uint8_t val) {
    csLow();
    beginTx();
    MX35_SPI_INSTANCE.transfer((uint8_t)0x1F);
    MX35_SPI_INSTANCE.transfer(addr);
    MX35_SPI_INSTANCE.transfer(val);
    endTx();
    csHigh();
  }
  inline bool isBusy() {
    return (getFeature(0xC0) & 0x01) != 0;
  }
  bool waitReady(uint32_t timeoutMs = 100) {
    uint32_t t0 = millis();
    while (isBusy()) {
      if ((millis() - t0) > timeoutMs) return false;
      yield();
    }
    return true;
  }
  // Macronix SPI-NAND requires a dummy byte after 0x9F before the 3 ID bytes
  void readId9F(uint8_t& mid, uint8_t& did1, uint8_t& did2, uint8_t& dummyOut) {
    csLow();
    beginTx();
    MX35_SPI_INSTANCE.transfer((uint8_t)0x9F);
    dummyOut = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);  // dummy
    mid = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);
    did1 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);
    did2 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);
    endTx();
    csHigh();
  }
  void readId90(uint8_t& did1, uint8_t& did2, uint8_t& dummyOut) {
    uint8_t dmy = 0, a0 = 0, d1 = 0, d2 = 0;
    csLow();
    beginTx();
    MX35_SPI_INSTANCE.transfer((uint8_t)0x90);
    dmy = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);
    a0 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);
    d1 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);
    d2 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);
    endTx();
    csHigh();
    (void)a0;
    dummyOut = dmy;
    did1 = d1;
    did2 = d2;
  }
  bool identify(IdInfo& info) {
    memset(&info, 0, sizeof(info));
    // Default to small-page until we know the part
    info.pageSize = 2048;
    info.spareSize = 64;
    uint8_t mid = 0, did1 = 0, did2 = 0, dmy = 0;
    readId9F(mid, did1, did2, dmy);
    if (mid == 0x00 || mid == 0xFF) {
      uint8_t d1 = 0, d2 = 0, dmy2 = 0;
      readId90(d1, d2, dmy2);
      if (d1 != 0x00 && d1 != 0xFF) {
        mid = 0xC2;
        did1 = d1;
        did2 = d2;
      }
    }
    info.mid = mid;
    info.did1 = did1;
    info.did2 = did2;
    if (mid == 0xC2) {
      // Map common densities and geometries
      switch (did1) {
        case 0x12:  // 1Gbit (128 MiB), 2K+64
          info.totalBytes = (uint64_t)128 * 1024 * 1024;
          info.pageSize = 2048;
          info.spareSize = 64;
          return true;
        case 0x22:  // 2Gbit (256 MiB)
        case 0x26:
          info.totalBytes = (uint64_t)256 * 1024 * 1024;
          info.pageSize = 2048;
          info.spareSize = 64;
          return true;
        case 0x2C:  // 4Gbit (512 MiB), 4K+128 (xGE4AD family)
        case 0x37:
          info.totalBytes = (uint64_t)512 * 1024 * 1024;
          info.pageSize = 4096;
          info.spareSize = 128;
          return true;
        default:
          break;
      }
    }
    info.totalBytes = 0;
    return false;
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
    MX35_SPI_INSTANCE.beginTransaction(_settings);
  }
  inline void endTx() {
    MX35_SPI_INSTANCE.endTransaction();
  }
};
// --------------------------- PSRAMBitbang (bit-bang SPI) ---------------------------
#ifndef PSRAMBITBANG_H
#define PSRAMBITBANG_H
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
#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_GENERIC_RP2040)) && !defined(BB_USE_RP2040_SIO)
#define BB_USE_RP2040_SIO 1
#endif
#ifdef BB_USE_RP2040_SIO
#include "hardware/structs/sio.h"
#endif
class PSRAMBitbang {
public:
  PSRAMBitbang(uint8_t pin_cs = PSRAM_PIN_CS,
               uint8_t pin_miso = PSRAM_PIN_MISO,
               uint8_t pin_mosi = PSRAM_PIN_MOSI,
               uint8_t pin_sck = PSRAM_PIN_SCK)
    : _pinCS(pin_cs), _pinMISO(pin_miso), _pinMOSI(pin_mosi),
      _pinSCK(pin_sck), _pinIO2(255), _pinIO3(255),
      _useQuad(false), _halfCycleDelayUs(1) {
#ifdef BB_USE_RP2040_SIO
    _maskCS = _maskMISO = _maskMOSI = _maskSCK = 0;
#endif
  }
  inline void begin() {
    pinMode(_pinCS, OUTPUT);
    pinMode(_pinMOSI, OUTPUT);
    pinMode(_pinSCK, OUTPUT);
    pinMode(_pinMISO, INPUT);
    digitalWrite(_pinCS, HIGH);
    digitalWrite(_pinSCK, LOW);
    digitalWrite(_pinMOSI, LOW);
#ifdef BB_USE_RP2040_SIO
    _maskCS = (1u << _pinCS);
    _maskMISO = (1u << _pinMISO);
    _maskMOSI = (1u << _pinMOSI);
    _maskSCK = (1u << _pinSCK);
#endif
  }
  inline void setClockDelayUs(uint8_t d) {
    _halfCycleDelayUs = d;
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
#ifdef BB_USE_RP2040_SIO
    if (_halfCycleDelayUs == 0) {
      uint8_t rx = 0;
      for (int bit = 7; bit >= 0; --bit) {
        if (tx & (1u << bit)) sio_hw->gpio_set = _maskMOSI;
        else sio_hw->gpio_clr = _maskMOSI;
        sio_hw->gpio_set = _maskSCK;
        rx = (uint8_t)((rx << 1) | ((sio_hw->gpio_in & _maskMISO) ? 1u : 0u));
        sio_hw->gpio_clr = _maskSCK;
      }
      return rx;
    }
#endif
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
  inline void transfer(const uint8_t* txbuf, uint8_t* rxbuf, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      uint8_t t = txbuf ? txbuf[i] : 0x00;
      uint8_t r = transfer(t);
      if (rxbuf) rxbuf[i] = r;
    }
  }
  inline void cmdRead(const uint8_t* cmd, size_t cmdLen, uint8_t* resp, size_t respLen) {
    csLow();
    if (cmd && cmdLen) transfer(cmd, nullptr, cmdLen);
    if (respLen) transfer(nullptr, resp, respLen);
    csHigh();
  }
  inline void readJEDEC(uint8_t* out, size_t len) {
    uint8_t cmd = PSRAM_CMD_READ_JEDEC;
    cmdRead(&cmd, 1, out, len);
  }
  inline bool readData03(uint32_t addr, uint8_t* buf, size_t len) {
    uint8_t cmd[4] = { 0x03, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    csLow();
    transfer(cmd, nullptr, 4);
    transfer(nullptr, buf, len);
    csHigh();
    return true;
  }
  inline void writeEnable() {
    uint8_t cmd = 0x06;
    csLow();
    transfer(cmd);
    csHigh();
  }
  inline bool writeData02(uint32_t addr, const uint8_t* buf, size_t len, bool needsWriteEnable = false) {
    if (!buf || len == 0) return true;
    if (needsWriteEnable) writeEnable();
    uint8_t cmd[4] = { 0x02, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    csLow();
    transfer(cmd, nullptr, 4);
    transfer(buf, nullptr, len);
    csHigh();
    return true;
  }
private:
  uint8_t _pinCS, _pinMISO, _pinMOSI, _pinSCK;
  uint8_t _pinIO2 = 255, _pinIO3 = 255;
  bool _useQuad;
  volatile uint8_t _halfCycleDelayUs;
#ifdef BB_USE_RP2040_SIO
  uint32_t _maskCS = 0, _maskMISO = 0, _maskMOSI = 0, _maskSCK = 0;
#endif
};
#endif  // PSRAMBITBANG_H
// --------------------------- W25QBitbang (NOR) ---------------------------
#ifdef W25Q_USE_HW_SPI
class W25QBitbang {
public:
  W25QBitbang(uint8_t pinMiso, uint8_t pinCs, uint8_t pinSck, uint8_t pinMosi)
    : _miso(pinMiso), _cs(pinCs), _sck(pinSck), _mosi(pinMosi),
      _settings(W25Q_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0) {}
  void begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    W25Q_SPI_INSTANCE.setRX(_miso);
    W25Q_SPI_INSTANCE.setTX(_mosi);
    W25Q_SPI_INSTANCE.setSCK(_sck);
    W25Q_SPI_INSTANCE.begin();
  }
  uint32_t readJEDEC(uint8_t& mfr, uint8_t& memType, uint8_t& capCode) {
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
  size_t readData(uint32_t addr, uint8_t* buf, size_t len) {
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
  bool pageProgram(uint32_t addr, const uint8_t* data, size_t len, uint32_t chunkTimeoutMs = 10) {
    if (!data || len == 0) return true;
    size_t off = 0;
    while (off < len) {
      size_t pageOff = (addr & 0xFF), pageSpace = 256 - pageOff;
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
    W25Q_SPI_INSTANCE.transfer((uint8_t)addr);
  }
};
#else  // W25Q_USE_HW_SPI
#include <inttypes.h>
#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_GENERIC_RP2040)) && !defined(BB_USE_RP2040_SIO)
#define BB_USE_RP2040_SIO 1
#endif
#ifdef BB_USE_RP2040_SIO
#include "hardware/structs/sio.h"
#endif
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
    _maskMISO = (1u << _miso);
    _maskCS = (1u << _cs);
    _maskSCK = (1u << _sck);
    _maskMOSI = (1u << _mosi);
#endif
  }
  uint32_t readJEDEC(uint8_t& mfr, uint8_t& memType, uint8_t& capCode) {
    csLow();
    xfer(0x9F);
    mfr = xfer(0x00);
    memType = xfer(0x00);
    capCode = xfer(0x00);
    csHigh();
    if (capCode < 32) return (uint32_t)1UL << capCode;
    return 0;
  }
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
  size_t readData(uint32_t addr, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return 0;
    csLow();
    xfer(0x03);
    sendAddr24(addr);
    for (size_t i = 0; i < len; ++i) buf[i] = xfer(0x00);
    csHigh();
    return len;
  }
  bool pageProgram(uint32_t addr, const uint8_t* data, size_t len, uint32_t chunkTimeoutMs = 10) {
    if (!data || len == 0) return true;
    size_t off = 0;
    while (off < len) {
      size_t pageOff = (addr & 0xFF), pageSpace = 256 - pageOff;
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
    xfer((uint8_t)addr);
  }
#ifdef BB_USE_RP2040_SIO
  uint32_t _maskMISO = 0, _maskCS = 0, _maskSCK = 0, _maskMOSI = 0;
#endif
};
#endif  // W25Q_USE_HW_SPI
// --------------------------- Unified facade (scan + list + reservation) ---------------------------
namespace UnifiedSpiMem {
enum class DeviceType : uint8_t {
  Unknown = 0,
  NorW25Q,
  SpiNandMX35,
  Psram
};
struct DeviceInfo {
  DeviceType type = DeviceType::Unknown;
  uint8_t cs = 0xFF;
  uint8_t jedec[8] = { 0 };
  uint8_t jedecLen = 0;
  uint8_t vendorId = 0;
  uint64_t capacityBytes = 0;
  uint8_t did1 = 0;
  uint8_t did2 = 0;
  const char* vendorName = "Unknown";
  const char* partHint = nullptr;
};
static inline const char* vendorNameFromMID(uint8_t mfr) {
  switch (mfr) {
    case 0xEF: return "Winbond       ";  // (W25Q)
    case 0xC2: return "Macronix      ";  // (MX25/MX35)
    case 0xC8: return "GigaDevice    ";  // (GD25)
    case 0x20: return "Micron/Numonyx";
    case 0x1F: return "Adesto/Atmel  ";
    case 0x9D: return "ISSI          ";
    case 0x85: return "Puya          ";
    case 0x68: return "BOYA          ";
    case 0x0D: return "AP Memory     ";  // (PSRAM)
    case 0x5E: return "Zbit/Zentel   ";  // (PSRAM)
    case 0x5D: return "Zentel        ";  // (PSRAM)
    default: return "Unknown       ";
  }
}
static inline const char* deviceTypeName(DeviceType t) {
  switch (t) {
    case DeviceType::NorW25Q: return "NOR";       // (W25Q-like)
    case DeviceType::SpiNandMX35: return "NAND";  // (MX35LF)
    case DeviceType::Psram: return "PSRAM";       // (PSRAM)
    default: return "Unknown";
  }
}
static inline bool isLikelyNOR(uint8_t mfr) {
  switch (mfr) {
    case 0xEF:
    case 0xC2:
    case 0xC8:
    case 0x20:
    case 0x1F:
    case 0x9D:
    case 0x85:
    case 0x68:
      return true;
    default: return false;
  }
}
static inline bool isLikelyPSRAMVendor(uint8_t mfr) {
  return (mfr == 0x0D || mfr == 0x5D || mfr == 0x5E);
}
static inline uint32_t psramCapFromKGD(uint8_t kgd) {
  switch (kgd) {
    case 0x5C: return 4u * 1024 * 1024;
    case 0x5D: return 8u * 1024 * 1024;
    case 0x5E: return 16u * 1024 * 1024;
    default: return 0;
  }
}
static inline const char* apmemPartFromKGD(uint8_t kgd) {
  switch (kgd) {
    case 0x5C: return "APM 32 Mbit (4 MiB)";
    case 0x5D: return "APM 64 Mbit (8 MiB)";
    case 0x5E: return "APM 128 Mbit (16 MiB)";
    default: return "APM (unknown density)";
  }
}
static inline bool normalizePSRAMId(const uint8_t raw[8], uint8_t norm[8]) {
  if (!raw || !norm) return false;
  for (int i = 0; i < 8; ++i) norm[i] = raw[i];
  int pos = -1;
  for (int i = 0; i < 4; ++i)
    if (raw[i] == 0x0D || raw[i] == 0x5D || raw[i] == 0x5E) {
      pos = i;
      break;
    }
  if (pos > 0) {
    bool leadingZeros = true;
    for (int i = 0; i < pos; ++i)
      if (raw[i] != 0x00) {
        leadingZeros = false;
        break;
      }
    if (leadingZeros) {
      for (int i = 0; i < 8 - pos; ++i) norm[i] = raw[i + pos];
      for (int i = 8 - pos; i < 8; ++i) norm[i] = 0x00;
      return true;
    }
  }
  return false;
}
// Forward declarations (namespace scope, not nested)
class MemDevice;
class NorMemDevice;
class PsramMemDevice;
class MX35NandMemDevice;
class Manager {
public:
  Manager(uint8_t pinSCK, uint8_t pinMOSI, uint8_t pinMISO, int8_t pinWP = -1, int8_t pinHOLD = -1)
    : _sck(pinSCK), _mosi(pinMOSI), _miso(pinMISO), _wp(pinWP), _hold(pinHOLD),
      _detectedCount(0), _csCount(0), _preservePsram(false) {
    for (size_t i = 0; i < UNIFIED_MAX_DETECTED; ++i) _reserved[i] = false;
  }
  void begin() {
    W25Q_SPI_INSTANCE.setRX(_miso);
    W25Q_SPI_INSTANCE.setTX(_mosi);
    W25Q_SPI_INSTANCE.setSCK(_sck);
    W25Q_SPI_INSTANCE.begin();
    if (_wp >= 0) {
      pinMode(_wp, OUTPUT);
      digitalWrite(_wp, HIGH);
    }
    if (_hold >= 0) {
      pinMode(_hold, OUTPUT);
      digitalWrite(_hold, HIGH);
    }
    ensureAllCsHigh();
  }
  // Option: avoid PSRAM reset during identify to preserve contents
  void setPreservePsramContents(bool enable) {
    _preservePsram = enable;
  }
  bool getPreservePsramContents() const {
    return _preservePsram;
  }
  // CS list management
  void clearCsList() {
    _csCount = 0;
  }
  void addCs(uint8_t cs) {
    if (_csCount >= UNIFIED_MAX_CS) return;
    for (size_t i = 0; i < _csCount; ++i)
      if (_csPins[i] == cs) return;
    _csPins[_csCount++] = cs;
  }
  void setCsList(const uint8_t* csList, size_t count) {
    clearCsList();
    if (!csList || count == 0) return;
    for (size_t i = 0; i < count && i < UNIFIED_MAX_CS; ++i) _csPins[_csCount++] = csList[i];
    ensureAllCsHigh();
  }
  void setCsList(std::initializer_list<uint8_t> list) {
    clearCsList();
    for (auto cs : list) {
      if (_csCount < UNIFIED_MAX_CS) _csPins[_csCount++] = cs;
    }
    ensureAllCsHigh();
  }
  // Scan APIs (store reservations as all free on scan)
  bool scanSingle(uint8_t cs, uint32_t hzForId = UNIFIED_SPI_CLOCK_HZ) {
    setCsList(&cs, 1);
    _detectedCount = 0;
    for (size_t i = 0; i < UNIFIED_MAX_DETECTED; ++i) _reserved[i] = false;
    DeviceInfo info{};
    if (identifyCS(cs, info, hzForId)) {
      if (_detectedCount < UNIFIED_MAX_DETECTED) {
        _detected[_detectedCount] = info;
        _reserved[_detectedCount] = false;
        _detectedCount++;
      }
      return true;
    }
    return false;
  }
  size_t scan(const uint8_t* csList, size_t count, uint32_t hzForId = UNIFIED_SPI_CLOCK_HZ) {
    setCsList(csList, count);
    _detectedCount = 0;
    for (size_t i = 0; i < UNIFIED_MAX_DETECTED; ++i) _reserved[i] = false;
    if (!csList || count == 0) return 0;
    for (size_t i = 0; i < count; ++i) {
      DeviceInfo info{};
      if (identifyCS(csList[i], info, hzForId)) {
        if (_detectedCount < UNIFIED_MAX_DETECTED) {
          _detected[_detectedCount] = info;
          _reserved[_detectedCount] = false;
          _detectedCount++;
        }
      }
    }
    return _detectedCount;
  }
  size_t scan(std::initializer_list<uint8_t> list, uint32_t hzForId = UNIFIED_SPI_CLOCK_HZ) {
    setCsList(list);
    _detectedCount = 0;
    for (size_t i = 0; i < UNIFIED_MAX_DETECTED; ++i) _reserved[i] = false;
    for (auto cs : list) {
      DeviceInfo info{};
      if (identifyCS(cs, info, hzForId)) {
        if (_detectedCount < UNIFIED_MAX_DETECTED) {
          _detected[_detectedCount] = info;
          _reserved[_detectedCount] = false;
          _detectedCount++;
        }
      }
    }
    return _detectedCount;
  }
  // Rescan using current CS list
  size_t rescan(uint32_t hzForId = UNIFIED_SPI_CLOCK_HZ) {
    if (_csCount == 0) return 0;
    return scan(_csPins, _csCount, hzForId);
  }
  // Query detected list
  size_t detectedCount() const {
    return _detectedCount;
  }
  const DeviceInfo* detectedInfo(size_t idx) const {
    if (idx >= _detectedCount) return nullptr;
    return &_detected[idx];
  }
  DeviceType typeAt(size_t idx) const {
    if (idx >= _detectedCount) return DeviceType::Unknown;
    return _detected[idx].type;
  }
  size_t getDetectedTypes(DeviceType* out, size_t maxOut) const {
    if (!out || maxOut == 0) return 0;
    size_t n = (_detectedCount < maxOut) ? _detectedCount : maxOut;
    for (size_t i = 0; i < n; ++i) out[i] = _detected[i].type;
    return n;
  }
  int findIndexByType(DeviceType t, size_t occurrence = 0, bool requireUnreserved = false) const {
    size_t seen = 0;
    for (size_t i = 0; i < _detectedCount; ++i) {
      if (_detected[i].type == t) {
        if (requireUnreserved && _reserved[i]) continue;
        if (seen == occurrence) return (int)i;
        ++seen;
      }
    }
    return -1;
  }
  bool isReserved(size_t idx) const {
    return (idx < _detectedCount) ? _reserved[idx] : false;
  }
  // Reserve/unreserve without opening
  bool reserveIndex(size_t idx) {
    if (idx >= _detectedCount || _reserved[idx]) return false;
    _reserved[idx] = true;
    return true;
  }
  bool unreserveIndex(size_t idx) {
    if (idx >= _detectedCount || !_reserved[idx]) return false;
    _reserved[idx] = false;
    return true;
  }
  // Open device (reservation-aware). Caller must release() to free.
  MemDevice* openByIndex(size_t idx);
  MemDevice* openByType(DeviceType t, size_t occurrence = 0) {
    int idx = findIndexByType(t, occurrence, true);
    if (idx < 0) return nullptr;
    return openByIndex((size_t)idx);
  }
  // Preferred open by type (first available of that type)
  MemDevice* openPreferred(DeviceType t) {
    return openByType(t, 0);
  }
  // Auto-open by priority: PSRAM, NOR, NAND
  MemDevice* openAuto() {
    MemDevice* h = nullptr;
    if (!h) h = openByType(DeviceType::Psram, 0);
    if (!h) h = openByType(DeviceType::NorW25Q, 0);
    if (!h) h = openByType(DeviceType::SpiNandMX35, 0);
    return h;
  }
  // Single-CS open (identify and reserve if in list and free; else ephemeral)
  MemDevice* openSingle(uint8_t cs, DeviceInfo* outInfo = nullptr);
  // Release and delete a device handle, freeing reservation if tracked
  bool release(MemDevice* dev);
  // Identify a single CS (public; re-used internally)
  bool identifyCS(uint8_t cs, DeviceInfo& out, uint32_t spiHzForId = UNIFIED_SPI_CLOCK_HZ) {
    out = DeviceInfo{};
    out.cs = cs;
    ensureAllCsHigh();
    delayMicroseconds(2);
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);

    // TRY MX35 SPI-NAND FIRST to avoid false-positive NOR detections
    {
      if (_wp >= 0) {
        pinMode(_wp, OUTPUT);
        digitalWrite(_wp, HIGH);
      }
      if (_hold >= 0) {
        pinMode(_hold, OUTPUT);
        digitalWrite(_hold, HIGH);
      }
      MX35LF mx(_miso, cs, _sck, _mosi);
      mx.begin();
      mx.setClock(1000000UL);  // safe low rate for ID
      mx.reset();
      delay(2);
      MX35LF::IdInfo mxinfo;
      bool mxok = mx.identify(mxinfo);
      if (mxok && mxinfo.totalBytes && (mxinfo.mid == 0xC2)) {
        out.type = DeviceType::SpiNandMX35;
        out.vendorId = 0xC2;
        out.vendorName = vendorNameFromMID(0xC2);
        out.did1 = mxinfo.did1;
        out.did2 = mxinfo.did2;
        out.capacityBytes = mxinfo.totalBytes;
        out.jedec[0] = mxinfo.mid;
        out.jedec[1] = out.did1;
        out.jedec[2] = out.did2;
        out.jedecLen = 3;
        return true;
      }
    }

    // THEN NOR
    {
      W25QBitbang nor(_miso, cs, _sck, _mosi);
      nor.begin();
      delay(1);
      uint8_t mfr = 0, memType = 0, capCode = 0;
      uint32_t norBytes = nor.readJEDEC(mfr, memType, capCode);
      bool noNOR = ((mfr == 0xFF && memType == 0xFF && capCode == 0xFF) || (mfr == 0x00 && memType == 0x00 && capCode == 0x00));
      if (!noNOR) {
        out.vendorId = mfr;
        out.vendorName = vendorNameFromMID(mfr);
        out.jedec[0] = mfr;
        out.jedec[1] = memType;
        out.jedec[2] = capCode;
        out.jedecLen = 3;
        if (isLikelyNOR(mfr) && norBytes != 0) {
          out.type = DeviceType::NorW25Q;
          out.capacityBytes = norBytes;
          return true;
        }
      }
    }

    // FINALLY PSRAM
    {
      if (!_preservePsram) {
        beginTransaction(spiHzForId / 2);
        csLow(cs);
        W25Q_SPI_INSTANCE.transfer((uint8_t)0xF5);
        csHigh(cs);
        delayMicroseconds(5);
        csLow(cs);
        W25Q_SPI_INSTANCE.transfer((uint8_t)0x66);
        csHigh(cs);
        delayMicroseconds(5);
        csLow(cs);
        W25Q_SPI_INSTANCE.transfer((uint8_t)0x99);
        csHigh(cs);
        delay(1);
        endTransaction();
      }
      uint8_t raw[8] = { 0 }, norm[8] = { 0 };
      beginTransaction(spiHzForId);
      csLow(cs);
      W25Q_SPI_INSTANCE.transfer((uint8_t)0x9F);
      for (int i = 0; i < 8; ++i) raw[i] = W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
      csHigh(cs);
      endTransaction();
      bool normalized = normalizePSRAMId(raw, norm);
      const uint8_t* pid = normalized ? norm : raw;
      out.vendorId = pid[0];
      out.vendorName = vendorNameFromMID(pid[0]);
      memcpy(out.jedec, pid, 8);
      out.jedecLen = 8;
      if (isLikelyPSRAMVendor(pid[0])) {
        out.type = DeviceType::Psram;
        if (pid[0] == 0x0D) {
          out.partHint = apmemPartFromKGD(pid[1]);
          out.capacityBytes = psramCapFromKGD(pid[1]);
        }
        return true;
      }
    }

    out.type = DeviceType::Unknown;
    out.vendorName = "Unknown";
    return false;
  }
  // SPI helpers
  static inline void beginTransaction(uint32_t hz) {
    SPISettings s(hz, MSBFIRST, SPI_MODE0);
    W25Q_SPI_INSTANCE.beginTransaction(s);
  }
  static inline void endTransaction() {
    W25Q_SPI_INSTANCE.endTransaction();
  }
  static inline void csLow(uint8_t cs) {
    digitalWrite(cs, LOW);
  }
  static inline void csHigh(uint8_t cs) {
    digitalWrite(cs, HIGH);
  }
  // Pins accessors (used to construct devices)
  uint8_t pinSCK() const {
    return _sck;
  }
  uint8_t pinMOSI() const {
    return _mosi;
  }
  uint8_t pinMISO() const {
    return _miso;
  }
private:
  void ensureAllCsHigh() const {
    for (size_t i = 0; i < _csCount; ++i) {
      pinMode(_csPins[i], OUTPUT);
      digitalWrite(_csPins[i], HIGH);
    }
  }
  MemDevice* createDevice(const DeviceInfo& info);  // defined after adapters
  uint8_t _sck, _mosi, _miso;
  int8_t _wp, _hold;
  DeviceInfo _detected[UNIFIED_MAX_DETECTED];
  bool _reserved[UNIFIED_MAX_DETECTED];
  size_t _detectedCount;
  uint8_t _csPins[UNIFIED_MAX_CS];
  size_t _csCount;
  bool _preservePsram;
public:
  const DeviceInfo& _getDetected(size_t idx) const {
    return _detected[idx];
  }
  DeviceInfo& _getDetected(size_t idx) {
    return _detected[idx];
  }
};
// --------------------------- Unified access layer (devices) ---------------------------
#ifndef UNIFIEDSPIMEM_LAYER_INCLUDED
#define UNIFIEDSPIMEM_LAYER_INCLUDED
#ifndef MX35_CACHE_READ_ADD_DUMMY
#define MX35_CACHE_READ_ADD_DUMMY 1
#endif
class MemDevice {
public:
  virtual ~MemDevice() {}
  virtual DeviceType type() const = 0;
  virtual uint64_t capacity() const = 0;
  virtual size_t read(uint64_t addr, uint8_t* buf, size_t len) = 0;
  virtual bool write(uint64_t addr, const uint8_t* buf, size_t len) = 0;
  virtual bool eraseRange(uint64_t addr, uint64_t len) = 0;
  virtual uint32_t pageSize() const {
    return 256;
  }
  virtual uint32_t eraseSize() const {
    return 4096;
  }
  uint8_t cs() const {
    return _cs;
  }
  DeviceType _t = DeviceType::Unknown;  // internal marker for release mapping
protected:
  explicit MemDevice(uint8_t cs)
    : _cs(cs) {}
  uint8_t _cs;
};
// NOR adapter
class NorMemDevice : public MemDevice {
public:
  NorMemDevice(uint8_t pinMISO, uint8_t cs, uint8_t pinSCK, uint8_t pinMOSI, uint64_t capacityBytes)
    : MemDevice(cs), _miso(pinMISO), _sck(pinSCK), _mosi(pinMOSI), _capacity(capacityBytes),
      _nor(pinMISO, cs, pinSCK, pinMOSI) {
    _t = DeviceType::NorW25Q;
  }
  bool begin() {
    _nor.begin();
    return true;
  }
  DeviceType type() const override {
    return DeviceType::NorW25Q;
  }
  uint64_t capacity() const override {
    return _capacity;
  }
  uint32_t pageSize() const override {
    return 256;
  }
  uint32_t eraseSize() const override {
    return 4096;
  }
  size_t read(uint64_t addr, uint8_t* buf, size_t len) override {
    if (!buf || len == 0) return 0;
    size_t total = 0;
    uint32_t a = (uint32_t)addr;
    while (total < len) {
      size_t chunk = (len - total > 4096) ? 4096 : (len - total);
      total += _nor.readData(a, buf + total, chunk);
      a += chunk;
    }
    return total;
  }
  bool write(uint64_t addr, const uint8_t* buf, size_t len) override {
    if (!buf || len == 0) return true;
    return _nor.pageProgram((uint32_t)addr, buf, len);
  }
  bool eraseRange(uint64_t addr, uint64_t len) override {
    if (len == 0) return true;
    uint64_t start = addr & ~(uint64_t)(eraseSize() - 1);
    uint64_t end = (addr + len + eraseSize() - 1) & ~(uint64_t)(eraseSize() - 1);
    for (uint64_t a = start; a < end; a += eraseSize())
      if (!_nor.sectorErase4K((uint32_t)a)) return false;
    return true;
  }
private:
  uint8_t _miso, _sck, _mosi;
  uint64_t _capacity;
  W25QBitbang _nor;
};
// PSRAM adapter
class PsramMemDevice : public MemDevice {
public:
  PsramMemDevice(uint8_t cs, uint64_t capacityBytes, uint8_t pinSCK, uint8_t pinMOSI, uint8_t pinMISO)
    : MemDevice(cs), _capacity(capacityBytes), _sck(pinSCK), _mosi(pinMOSI), _miso(pinMISO) {
    _t = DeviceType::Psram;
  }
  bool begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    return true;
  }
  DeviceType type() const override {
    return DeviceType::Psram;
  }
  uint64_t capacity() const override {
    return _capacity;
  }
  uint32_t pageSize() const override {
    return 1024;
  }
  uint32_t eraseSize() const override {
    return 0;
  }
  size_t read(uint64_t addr, uint8_t* buf, size_t len) override {
    if (!buf || len == 0) return 0;
    size_t total = 0;
    while (total < len) {
      size_t chunk = (len - total > 4096) ? 4096 : (len - total);
      beginTx();
      csLow();
      W25Q_SPI_INSTANCE.transfer((uint8_t)0x03);
      sendAddr24((uint32_t)addr);
      for (size_t i = 0; i < chunk; ++i) buf[total + i] = W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
      csHigh();
      endTx();
      addr += chunk;
      total += chunk;
    }
    return total;
  }
  bool write(uint64_t addr, const uint8_t* buf, size_t len) override {
    if (!buf || len == 0) return true;
    size_t total = 0;
    while (total < len) {
      size_t chunk = (len - total > 4096) ? 4096 : (len - total);
      beginTx();
      csLow();
      W25Q_SPI_INSTANCE.transfer((uint8_t)0x02);
      sendAddr24((uint32_t)addr);
      for (size_t i = 0; i < chunk; ++i) W25Q_SPI_INSTANCE.transfer(buf[total + i]);
      csHigh();
      endTx();
      addr += chunk;
      total += chunk;
    }
    return true;
  }
  bool eraseRange(uint64_t, uint64_t) override {
    return false;
  }
private:
  inline void csLow() {
    digitalWrite(_cs, LOW);
  }
  inline void csHigh() {
    digitalWrite(_cs, HIGH);
  }
  inline void beginTx(uint32_t hz = UNIFIED_SPI_CLOCK_HZ) {
    SPISettings s(hz, MSBFIRST, SPI_MODE0);
    W25Q_SPI_INSTANCE.beginTransaction(s);
  }
  inline void endTx() {
    W25Q_SPI_INSTANCE.endTransaction();
  }
  inline void sendAddr24(uint32_t addr) {
    W25Q_SPI_INSTANCE.transfer((uint8_t)(addr >> 16));
    W25Q_SPI_INSTANCE.transfer((uint8_t)(addr >> 8));
    W25Q_SPI_INSTANCE.transfer((uint8_t)addr);
  }
  uint64_t _capacity;
  uint8_t _sck, _mosi, _miso;
};
// SPI‑NAND adapter (raw x1 I/O; no ECC/BBT)
class MX35NandMemDevice : public MemDevice {
public:
  struct Geometry {
    uint32_t pageSize = 2048, spareSize = 64, pagesPerBlock = 64, blocks = 0;
  };
  MX35NandMemDevice(uint8_t pinMISO, uint8_t cs, uint8_t pinSCK, uint8_t pinMOSI, uint64_t capacityBytes)
    : MemDevice(cs), _miso(pinMISO), _sck(pinSCK), _mosi(pinMOSI), _capacity(capacityBytes) {
    _t = DeviceType::SpiNandMX35;
    _geo.blocks = (uint32_t)(_capacity / (uint64_t)(_geo.pageSize * _geo.pagesPerBlock));
  }
  bool begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    // Adjust geometry based on known densities (4Gb = 512 MiB => 4K page)
    if (_capacity >= (uint64_t)512 * 1024 * 1024) {
      _geo.pageSize = 4096;
#if 1
      _geo.spareSize = 128;  // reference demo assumption
#else
      _geo.spareSize = 224;  // some revs use 224 bytes OOB
#endif
      _geo.pagesPerBlock = 64;
      _geo.blocks = (uint32_t)(_capacity / (uint64_t)(_geo.pageSize * _geo.pagesPerBlock));
    }
    // Clear block protection so program/erase will work
    setFeature(0xA0, 0x00);
    return true;
  }
  DeviceType type() const override {
    return DeviceType::SpiNandMX35;
  }
  uint64_t capacity() const override {
    return _capacity;
  }
  uint32_t pageSize() const override {
    return _geo.pageSize;
  }
  uint32_t eraseSize() const override {
    return _geo.pageSize * _geo.pagesPerBlock;
  }
  void setGeometry(const Geometry& g) {
    _geo = g;
    if (_geo.blocks == 0 && _geo.pageSize && _geo.pagesPerBlock) _geo.blocks = (uint32_t)(_capacity / (uint64_t)(_geo.pageSize * _geo.pagesPerBlock));
  }
  // Allow caller to lower/raise SPI clock for NAND independently of unified default
  void setClock(uint32_t hz) {
    _spiHz = hz;
  }
  size_t read(uint64_t addr, uint8_t* buf, size_t len) override {
    if (!buf || len == 0) return 0;
    size_t total = 0;
    while (total < len) {
      uint32_t page = (uint32_t)(addr / _geo.pageSize);
      uint16_t col = (uint16_t)(addr % _geo.pageSize);
      size_t chunk = min<size_t>(len - total, (size_t)(_geo.pageSize - col));
      if (!pageReadToCache(page)) break;
      if (!readFromCache(col, buf + total, chunk)) break;
      addr += chunk;
      total += chunk;
    }
    return total;
  }
  bool write(uint64_t addr, const uint8_t* buf, size_t len) override {
    if (!buf || len == 0) return true;
    while (len > 0) {
      uint32_t page = (uint32_t)(addr / _geo.pageSize);
      uint16_t col = (uint16_t)(addr % _geo.pageSize);
      size_t chunk = min<size_t>(len, (size_t)(_geo.pageSize - col));
      if (!programLoad(col, buf, chunk)) return false;
      if (!programExecute(page)) return false;
      addr += chunk;
      buf += chunk;
      len -= chunk;
    }
    return true;
  }
  bool eraseRange(uint64_t addr, uint64_t len) override {
    if (len == 0) return true;
    uint64_t esize = eraseSize();
    uint64_t start = (addr / esize) * esize;
    uint64_t end = ((addr + len + esize - 1) / esize) * esize;
    for (uint64_t a = start; a < end; a += esize) {
      uint32_t pageRow = (uint32_t)(a / _geo.pageSize);
      if (!blockErase(pageRow)) return false;
    }
    return true;
  }
  // Low-level API (used by helpers)
  bool pageReadToCache(uint32_t row) {
    beginTx();
    csLow();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x13);
    sendRowAddr24(row);
    csHigh();
    endTx();
    return waitReady(2);  // ~2ms timeout
  }
  bool readFromCache(uint16_t col, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return true;
    beginTx();
    csLow();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x03);
    W25Q_SPI_INSTANCE.transfer((uint8_t)(col >> 8));
    W25Q_SPI_INSTANCE.transfer((uint8_t)(col & 0xFF));
#if MX35_CACHE_READ_ADD_DUMMY
    (void)W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
#endif
    for (size_t i = 0; i < len; ++i) buf[i] = W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
    csHigh();
    endTx();
    return true;
  }
  bool programLoad(uint16_t col, const uint8_t* data, size_t len) {
    if (!data || len == 0) return true;
    if (!writeEnable()) return false;
    beginTx();
    csLow();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x02);
    W25Q_SPI_INSTANCE.transfer((uint8_t)(col >> 8));
    W25Q_SPI_INSTANCE.transfer((uint8_t)(col & 0xFF));
    for (size_t i = 0; i < len; ++i) W25Q_SPI_INSTANCE.transfer(data[i]);
    csHigh();
    endTx();
    return true;
  }
  bool programExecute(uint32_t row) {
    beginTx();
    csLow();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x10);
    sendRowAddr24(row);
    csHigh();
    endTx();
    if (!waitReady(6)) return false;  // up to ~6ms
    uint8_t st = getFeature(0xC0);
    if (st & (1u << 3)) return false;  // PFAIL
    return true;
  }
  bool blockErase(uint32_t row) {
    if (!writeEnable()) return false;
    beginTx();
    csLow();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0xD8);
    sendRowAddr24(row);
    csHigh();
    endTx();
    if (!waitReady(120)) return false;  // up to ~120ms
    uint8_t st = getFeature(0xC0);
    if (st & (1u << 2)) return false;  // EFAIL
    return true;
  }
  uint8_t getFeature(uint8_t addr) {
    beginTx();
    csLow();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x0F);
    W25Q_SPI_INSTANCE.transfer(addr);
    uint8_t v = W25Q_SPI_INSTANCE.transfer((uint8_t)0x00);
    csHigh();
    endTx();
    return v;
  }
  void setFeature(uint8_t addr, uint8_t value) {
    beginTx();
    csLow();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x1F);
    W25Q_SPI_INSTANCE.transfer(addr);
    W25Q_SPI_INSTANCE.transfer(value);
    csHigh();
    endTx();
  }
private:
  inline void csLow() {
    digitalWrite(_cs, LOW);
  }
  inline void csHigh() {
    digitalWrite(_cs, HIGH);
  }
  inline void beginTx() {
    SPISettings s(_spiHz, MSBFIRST, SPI_MODE0);
    W25Q_SPI_INSTANCE.beginTransaction(s);
  }
  inline void endTx() {
    W25Q_SPI_INSTANCE.endTransaction();
  }
  inline void sendRowAddr24(uint32_t row) {
    W25Q_SPI_INSTANCE.transfer((uint8_t)(row >> 16));
    W25Q_SPI_INSTANCE.transfer((uint8_t)(row >> 8));
    W25Q_SPI_INSTANCE.transfer((uint8_t)row);
  }
  bool writeEnable() {
    beginTx();
    csLow();
    W25Q_SPI_INSTANCE.transfer((uint8_t)0x06);
    csHigh();
    endTx();
    return true;
  }
  bool waitReady(uint32_t timeoutMs) {
    uint32_t t0 = millis();
    while (true) {
      uint8_t st = getFeature(0xC0);
      if ((st & 0x01) == 0) return true;  // OIP cleared
      if ((millis() - t0) > timeoutMs) return false;
      yield();
    }
  }
  uint8_t _miso, _sck, _mosi;
  uint64_t _capacity;
  Geometry _geo;
  uint32_t _spiHz = 20000000UL;  // safer default for SPI-NAND
};
// Manager: device construction
inline MemDevice* Manager::createDevice(const DeviceInfo& info) {
  switch (info.type) {
    case DeviceType::NorW25Q:
      {
        auto* dev = new NorMemDevice(_miso, info.cs, _sck, _mosi, info.capacityBytes);
        dev->begin();
        return dev;
      }
    case DeviceType::Psram:
      {
        auto* dev = new PsramMemDevice(info.cs, info.capacityBytes, _sck, _mosi, _miso);
        dev->begin();
        return dev;
      }
    case DeviceType::SpiNandMX35:
      {
        auto* dev = new MX35NandMemDevice(_miso, info.cs, _sck, _mosi, info.capacityBytes);
        dev->begin();
        return dev;
      }
    default: return nullptr;
  }
}
inline MemDevice* Manager::openByIndex(size_t idx) {
  if (idx >= _detectedCount) return nullptr;
  if (_reserved[idx]) return nullptr;
  _reserved[idx] = true;
  const DeviceInfo& info = _getDetected(idx);
  return createDevice(info);
}
inline MemDevice* Manager::openSingle(uint8_t cs, DeviceInfo* outInfo) {
  // Prefer tracked entry to respect reservation
  for (size_t i = 0; i < _detectedCount; ++i) {
    if (_detected[i].cs == cs && !_reserved[i]) {
      return openByIndex(i);
    }
  }
  // Not tracked or already reserved: ephemeral open (no reservation)
  DeviceInfo info{};
  if (!identifyCS(cs, info)) return nullptr;
  if (outInfo) *outInfo = info;
  return createDevice(info);
}
inline bool Manager::release(MemDevice* dev) {
  if (!dev) return false;
  // Try to match tracked reservation by CS+type
  for (size_t i = 0; i < _detectedCount; ++i) {
    if (_reserved[i] && _detected[i].cs == dev->cs() && _detected[i].type == dev->type()) {
      _reserved[i] = false;
      delete dev;
      return true;
    }
  }
  // Not tracked: just delete
  delete dev;
  return true;
}
#endif  // UNIFIEDSPIMEM_LAYER_INCLUDED
// --------------------------- Device Pooling ---------------------------
class DevicePool {
public:
  enum class SelectMode : uint8_t { Any = 0,
                                    ByType };
  static DevicePool* createAll(Manager& mgr, bool reserve = true) {
    return createInternal(mgr, SelectMode::Any, DeviceType::Unknown, reserve);
  }
  static DevicePool* createByType(Manager& mgr, DeviceType t, bool reserve = true) {
    return createInternal(mgr, SelectMode::ByType, t, reserve);
  }
  size_t size() const {
    return _count;
  }
  const DeviceInfo* infoAt(size_t i) const {
    return (i < _count) ? &_snap[i] : nullptr;
  }
  MemDevice* openAt(size_t i) {
    if (i >= _count) return nullptr;
    if (_reservedAtCreate[i]) {
      int idx = _mgrIndexFromSnap(i);
      if (idx < 0) return nullptr;
      return _mgr.openByIndex((size_t)idx);
    } else {
      return _mgr.openSingle(_snap[i].cs, nullptr);
    }
  }
  bool poolRelease(size_t i, MemDevice* dev) {
    if (!dev) return false;
    if (i >= _count) {
      delete dev;
      return false;
    }
    if (_reservedAtCreate[i]) return _mgr.release(dev);
    delete dev;
    return true;
  }
  void releaseAllReservations() {
    // Reservations are tied to handles when opened; nothing to release here.
  }
  ~DevicePool() {
    delete[] _snap;
    delete[] _reservedAtCreate;
  }
private:
  DevicePool(Manager& mgr)
    : _mgr(mgr), _snap(nullptr), _reservedAtCreate(nullptr), _count(0), _mode(SelectMode::Any), _type(DeviceType::Unknown) {}
  static DevicePool* createInternal(Manager& mgr, SelectMode mode, DeviceType t, bool reserve) {
    size_t n = mgr.detectedCount();
    DevicePool* pool = new DevicePool(mgr);
    if (n == 0) return pool;
    size_t eligible = 0;
    for (size_t i = 0; i < n; ++i) {
      const DeviceInfo* di = mgr.detectedInfo(i);
      if (!di) continue;
      if (mode == SelectMode::ByType && di->type != t) continue;
      eligible++;
    }
    if (eligible == 0) return pool;
    pool->_snap = new DeviceInfo[eligible];
    pool->_reservedAtCreate = new bool[eligible];
    pool->_count = eligible;
    pool->_mode = mode;
    pool->_type = t;
    size_t k = 0;
    for (size_t i = 0; i < n && k < eligible; ++i) {
      const DeviceInfo* di = mgr.detectedInfo(i);
      if (!di) continue;
      if (mode == SelectMode::ByType && di->type != t) continue;
      pool->_snap[k] = *di;
      bool didReserve = false;
      if (reserve) {
        if (!mgr.isReserved(i)) didReserve = mgr.reserveIndex(i);
      }
      pool->_reservedAtCreate[k] = didReserve;
      k++;
    }
    return pool;
  }
  int _mgrIndexFromSnap(size_t i) const {
    if (i >= _count) return -1;
    const DeviceInfo& di = _snap[i];
    for (size_t m = 0; m < _mgr.detectedCount(); ++m) {
      const DeviceInfo* cur = _mgr.detectedInfo(m);
      if (!cur) continue;
      if (cur->cs == di.cs && cur->type == di.type) return (int)m;
    }
    return -1;
  }
  Manager& _mgr;
  DeviceInfo* _snap;
  bool* _reservedAtCreate;
  size_t _count;
  SelectMode _mode;
  DeviceType _type;
};
}  // namespace UnifiedSpiMem