// FastSPIAdapter.h
// Adapter that exposes a PSRAMBitbang/W25QBitbang-compatible API
// backed by FastSPI_DMA (RP2040 hardware SPI + DMA).
#pragma once

#ifdef ARDUINO_ARCH_RP2040

#include <Arduino.h>
#include "FastSPI_DMA.h"

// Simple adapter: constructor signature matches PSRAMBitbang(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck)
class FastSPIAdapter {
public:
  // Accepts same parameter ordering as PSRAMBitbang: cs, miso, mosi, sck
  FastSPIAdapter(uint8_t pin_cs = 255, uint8_t pin_miso = 12, uint8_t pin_mosi = 11, uint8_t pin_sck = 10)
    : _cs(pin_cs), _spi(spi0, pin_miso, pin_sck, pin_mosi, pin_cs),
      _pinMISO(pin_miso), _pinMOSI(pin_mosi), _pinSCK(pin_sck) {
    // default clock (can be changed by setClockDelayUs wrapper)
    _halfCycleDelayUs = 1;
    _useQuad = false;
  }

  // Basic lifecycle
  void begin() {
    _spi.begin();
    if (_cs != 255) {
      pinMode(_cs, OUTPUT);
      digitalWrite(_cs, HIGH);
    }
  }

  // Map half-cycle delay to a hardware-friendly clock value:
  // If halfCycleDelayUs == 0 -> set a high clock (you choose safe default), otherwise derive
  // an approximate clock Hz (not perfect mapping but good enough).
  void setClockDelayUs(uint8_t halfCycleDelayUs) {
    _halfCycleDelayUs = halfCycleDelayUs;
    if (halfCycleDelayUs == 0) {
      _spi.setClockHz(40'000'000);  // try 40 MHz (subject to board constraints)
    } else {
      // convert half-cycle micros to approx Hertz: period_us = halfCycle*2, hz ~= 1e6/period_us
      uint32_t periodUs = (uint32_t)halfCycleDelayUs * 2u;
      if (periodUs == 0) periodUs = 1;
      uint32_t hz = (uint32_t)(1000000UL / periodUs);
      if (hz < 100000) hz = 100000;  // floor
      _spi.setClockHz(hz);
    }
  }

  // PSRAM-like extras -- no-ops or stored for compatibility
  void setExtraDataPins(uint8_t io2, uint8_t io3) {
    _io2 = io2;
    _io3 = io3;
  }
  void setModeQuad(bool enable) {
    _useQuad = enable;
  }

  // CS helpers (adapter manages a CS pin if provided, but for 74HC138 flows CS is controlled externally)
  inline void csLow() {
    if (_cs != 255) digitalWrite(_cs, LOW);
  }
  inline void csHigh() {
    if (_cs != 255) digitalWrite(_cs, HIGH);
  }

  // Basic transfers (PSRAMBitbang-compatible API)
  uint8_t transfer(uint8_t tx) {
    uint8_t rx = 0;
    _spi.transfer(&tx, &rx, 1);
    return rx;
  }

  void transfer(const uint8_t *txbuf, uint8_t *rxbuf, size_t len) {
    _spi.transfer(txbuf, rxbuf, len);
  }

  void cmdRead(const uint8_t *cmd, size_t cmdLen, uint8_t *resp, size_t respLen) {
    // cs should be managed by caller (adapter provides cs helpers)
    if (cmd && cmdLen) _spi.transfer(cmd, nullptr, cmdLen);
    if (resp && respLen) _spi.transfer(nullptr, resp, respLen);
  }

  void readJEDEC(uint8_t *out, size_t len) {
    uint8_t cmd = 0x9F;
    csLow();
    _spi.transfer(&cmd, nullptr, 1);
    _spi.transfer(nullptr, out, len);
    csHigh();
  }

  bool readData03(uint32_t addr, uint8_t *buf, size_t len) {
    uint8_t hdr[4] = { 0x03, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    csLow();
    _spi.transfer(hdr, nullptr, 4);
    if (len) _spi.transfer(nullptr, buf, len);
    csHigh();
    return true;
  }

  void writeEnable() {
    uint8_t cmd = 0x06;
    csLow();
    _spi.transfer(&cmd, nullptr, 1);
    csHigh();
  }

  bool writeData02(uint32_t addr, const uint8_t *buf, size_t len, bool /*needsWriteEnable*/ = false) {
    if (!buf || len == 0) return true;
    uint8_t hdr[4] = { 0x02, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    csLow();
    _spi.transfer(hdr, nullptr, 4);
    _spi.transfer(buf, nullptr, len);
    csHigh();
    return true;
  }

  void rawMisoScan(uint8_t *out, size_t len) {
    csLow();
    uint8_t z = 0x00;
    for (size_t i = 0; i < len; ++i) {
      _spi.transfer(&z, &out[i], 1);
    }
    csHigh();
  }

  // Quad helpers: we don't implement real quad mode here (requires wiring + signaling).
  inline bool quadAvailable() const {
    return false;
  }
  inline bool quadWriteByte(uint8_t /*tx*/) {
    return false;
  }
  inline bool quadReadByte(uint8_t & /*out*/) {
    return false;
  }

  // ---- W25Q-style methods (to satisfy W25QSimpleFS use) ----
  // readData: linear 0x03 read
  size_t readData(uint32_t addr, uint8_t *buf, size_t len) {
    if (!buf || len == 0) return 0;
    readData03(addr, buf, len);
    return len;
  }

  // pageProgram: auto-split like W25QBitbang expects.
  bool pageProgram(uint32_t addr, const uint8_t *data, size_t len, uint32_t /*chunkTimeoutMs*/ = 10) {
    if (!data || len == 0) return true;
    size_t off = 0;
    while (off < len) {
      size_t pageOff = (addr & 0xFF);
      size_t pageSpace = 256 - pageOff;
      size_t chunk = (len - off < pageSpace) ? (len - off) : pageSpace;
      // write enable
      writeEnable();
      // program
      uint8_t hdr[4] = { 0x02, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
      csLow();
      _spi.transfer(hdr, nullptr, 4);
      _spi.transfer(data + off, nullptr, chunk);
      csHigh();
      // Wait for WIP clear
      if (!waitWhileBusy(5000)) return false;
      addr += chunk;
      off += chunk;
    }
    return true;
  }

  bool sectorErase4K(uint32_t addr, uint32_t timeoutMs = 4000) {
    writeEnable();
    uint8_t hdr[4] = { 0x20, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    csLow();
    _spi.transfer(hdr, nullptr, 4);
    csHigh();
    return waitWhileBusy(timeoutMs);
  }

  bool chipErase(uint32_t timeoutMs = 180000) {
    writeEnable();
    uint8_t cmd = 0xC7;
    csLow();
    _spi.transfer(&cmd, nullptr, 1);
    csHigh();
    return waitWhileBusy(timeoutMs);
  }

  uint8_t readStatus1() {
    uint8_t cmd = 0x05, s = 0;
    csLow();
    _spi.transfer(&cmd, nullptr, 1);
    _spi.transfer(nullptr, &s, 1);
    csHigh();
    return s;
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

private:
  uint8_t _cs;
  FastSPI_DMA _spi;
  uint8_t _pinMISO, _pinMOSI, _pinSCK;
  uint8_t _io2 = 255, _io3 = 255;
  bool _useQuad;
  volatile uint8_t _halfCycleDelayUs;
};

#else
#error "FastSPIAdapter: RP2040-specific adapter (ARUDINO_ARCH_RP2040 required)."
#endif