#pragma once
#include <Arduino.h>
#include <SPI.h>

// HW-SPI driver for Winbond W25Q series (SPI mode 0).
// Exposes the same interface used by W25QSimpleFS:
//   uint32_t readJEDEC(uint8_t& mfr, uint8_t& memType, uint8_t& capCode);
//   size_t   readData(uint32_t addr, uint8_t* buf, size_t len);
//   bool     pageProgram(uint32_t addr, const uint8_t* data, size_t len);
//   bool     sectorErase4K(uint32_t addr);
//   bool     chipErase();
//
// Also provides helpers: readStatus1(), writeEnable(), waitWhileBusy(), isBusy().
class W25QHwSPI {
public:
  W25QHwSPI(SPIClass& spi, uint8_t csPin,
            uint32_t hz = 50000000UL,  // 50 MHz default; adjust for wiring
            uint8_t mode = SPI_MODE0)
    : _spi(&spi), _cs(csPin), _hz(hz), _mode(mode),
      _settings(hz, MSBFIRST, mode) {}

  void begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    // SPI bus must be begun externally (SPI.begin()); this class does transactions per op.
  }

  inline void setClock(uint32_t hz) {
    _hz = hz;
    _settings = SPISettings(_hz, MSBFIRST, _mode);
  }
  inline void setDataMode(uint8_t mode) {
    _mode = mode;
    _settings = SPISettings(_hz, MSBFIRST, _mode);
  }

  // JEDEC ID (0x9F). Returns total capacity in bytes (2^capCode) or 0 on error.
  uint32_t readJEDEC(uint8_t& mfr, uint8_t& memType, uint8_t& capCode) {
    beginTrans_();
    csLow_();
    _spi->transfer(0x9F);
    mfr = _spi->transfer(0x00);
    memType = _spi->transfer(0x00);
    capCode = _spi->transfer(0x00);
    csHigh_();
    endTrans_();
    if (capCode < 32) return (uint32_t)1UL << capCode;
    return 0;
  }

  // Status register-1 (0x05): bit0=WIP, bit1=WEL
  uint8_t readStatus1() {
    uint8_t v;
    beginTrans_();
    csLow_();
    _spi->transfer(0x05);
    v = _spi->transfer(0x00);
    csHigh_();
    endTrans_();
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
    beginTrans_();
    csLow_();
    _spi->transfer(0x06);
    csHigh_();
    endTrans_();
    uint32_t t0 = millis();
    while ((readStatus1() & 0x02) == 0) {
      if ((millis() - t0) > confirmTimeoutMs) return false;
      yield();
    }
    return true;
  }

  // Linear read (0x03)
  size_t readData(uint32_t addr, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return 0;
    beginTrans_();
    csLow_();
    _spi->transfer(0x03);
    sendAddr24_(addr);
    for (size_t i = 0; i < len; ++i) buf[i] = _spi->transfer(0x00);
    csHigh_();
    endTrans_();
    return len;
  }

  // Page program (0x02), auto-splits at 256-byte boundaries
  bool pageProgram(uint32_t addr, const uint8_t* data, size_t len,
                   uint32_t chunkTimeoutMs = 10) {
    if (!data || len == 0) return true;
    size_t off = 0;
    while (off < len) {
      size_t pageOff = (addr & 0xFF);
      size_t pageSpace = 256 - pageOff;
      size_t chunk = (len - off < pageSpace) ? (len - off) : pageSpace;
      if (!writeEnable()) return false;
      beginTrans_();
      csLow_();
      _spi->transfer(0x02);
      sendAddr24_(addr);
      for (size_t i = 0; i < chunk; ++i) _spi->transfer(data[off + i]);
      csHigh_();
      endTrans_();
      if (!waitWhileBusy(chunkTimeoutMs)) return false;
      addr += chunk;
      off += chunk;
      yield();
    }
    return true;
  }

  bool sectorErase4K(uint32_t addr, uint32_t timeoutMs = 4000) {
    if (!writeEnable()) return false;
    beginTrans_();
    csLow_();
    _spi->transfer(0x20);
    sendAddr24_(addr);
    csHigh_();
    endTrans_();
    return waitWhileBusy(timeoutMs);
  }

  bool chipErase(uint32_t timeoutMs = 180000) {
    if (!writeEnable()) return false;
    beginTrans_();
    csLow_();
    _spi->transfer(0xC7);  // or 0x60
    csHigh_();
    endTrans_();
    return waitWhileBusy(timeoutMs);
  }

private:
  SPIClass* _spi;
  uint8_t _cs;
  uint32_t _hz;
  uint8_t _mode;
  SPISettings _settings;

  inline void beginTrans_() {
    _spi->beginTransaction(_settings);
  }
  inline void endTrans_() {
    _spi->endTransaction();
  }
  inline void csLow_() {
    digitalWrite(_cs, LOW);
  }
  inline void csHigh_() {
    digitalWrite(_cs, HIGH);
  }
  inline void sendAddr24_(uint32_t addr) {
    _spi->transfer((uint8_t)(addr >> 16));
    _spi->transfer((uint8_t)(addr >> 8));
    _spi->transfer((uint8_t)(addr >> 0));
  }
};