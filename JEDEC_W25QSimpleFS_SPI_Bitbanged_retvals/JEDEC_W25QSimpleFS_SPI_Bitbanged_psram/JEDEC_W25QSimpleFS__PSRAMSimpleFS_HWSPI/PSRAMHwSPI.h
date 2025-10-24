#pragma once
#include <Arduino.h>
#include <SPI.h>

// HW-SPI driver for PSRAM-like device with 0x03 (read) and 0x02 (write).
// Exposes the interface consumed by PSRAMSimpleFS:
//   void readData03(uint32_t addr, uint8_t* buf, size_t len);
//   bool writeData02(uint32_t addr, const uint8_t* buf, size_t len, bool needsWriteEnable=false);
//
// Also provides readJEDEC() and writeEnable().
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

class PSRAMHwSPI {
public:
  PSRAMHwSPI(SPIClass& spi, uint8_t csPin,
             uint32_t hz = 40000000UL,  // 40 MHz default; adjust as needed
             uint8_t mode = SPI_MODE0)
    : _spi(&spi), _cs(csPin), _hz(hz), _mode(mode),
      _settings(hz, MSBFIRST, mode) {}

  void begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    // SPI bus must be begun externally; this class manages transactions.
  }

  inline void setClock(uint32_t hz) {
    _hz = hz;
    _settings = SPISettings(_hz, MSBFIRST, _mode);
  }
  inline void setDataMode(uint8_t mode) {
    _mode = mode;
    _settings = SPISettings(_hz, MSBFIRST, _mode);
  }

  // Optional utility
  void readJEDEC(uint8_t* out, size_t len) {
    if (!out || len == 0) return;
    beginTrans_();
    csLow_();
    _spi->transfer(PSRAM_CMD_READ_JEDEC);
    for (size_t i = 0; i < len; ++i) out[i] = _spi->transfer(0x00);
    csHigh_();
    endTrans_();
  }

  void writeEnable() {
    beginTrans_();
    csLow_();
    _spi->transfer(PSRAM_CMD_WRITE_ENABLE);
    csHigh_();
    endTrans_();
  }

  // Linear read 0x03
  bool readData03(uint32_t addr, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return true;
    beginTrans_();
    csLow_();
    _spi->transfer(PSRAM_CMD_READ_03);
    sendAddr24_(addr);
    for (size_t i = 0; i < len; ++i) buf[i] = _spi->transfer(0x00);
    csHigh_();
    endTrans_();
    return true;
  }

  // Linear write 0x02 (some devices do not require WREN; parameter allows enabling if needed)
  bool writeData02(uint32_t addr, const uint8_t* buf, size_t len, bool needsWriteEnable = false) {
    if (!buf || len == 0) return true;
    if (needsWriteEnable) writeEnable();
    beginTrans_();
    csLow_();
    _spi->transfer(PSRAM_CMD_WRITE_02);
    sendAddr24_(addr);
    // Chunk large transfers to avoid huge blocking bursts (optional)
    const size_t CHUNK = 1024;
    size_t off = 0;
    while (off < len) {
      size_t n = (len - off > CHUNK) ? CHUNK : (len - off);
      for (size_t i = 0; i < n; ++i) _spi->transfer(buf[off + i]);
      off += n;
      yield();
    }
    csHigh_();
    endTrans_();
    return true;
  }

  // Optional raw read of MISO with clocks
  void rawMisoScan(uint8_t* out, size_t len) {
    if (!out || len == 0) return;
    beginTrans_();
    csLow_();
    for (size_t i = 0; i < len; ++i) out[i] = _spi->transfer(0x00);
    csHigh_();
    endTrans_();
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