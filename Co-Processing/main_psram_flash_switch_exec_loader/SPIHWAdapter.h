#pragma once
#ifdef ARDUINO_ARCH_RP2040
#include <Arduino.h>
#include <SPI.h>

// Select which HW SPI to use (RP2040 has SPI and SPI1)
// For your pins SCK=10, MOSI=12, MISO=11, SPI1 is the right peripheral.
// If you use different pins, just change this macro.
#ifndef PSRAM_SPI_INSTANCE
#define PSRAM_SPI_INSTANCE SPI1
#endif

class SPIHWAdapter {
public:
  // Keep PSRAMBitbang-compatible signature: (cs, miso, mosi, sck)
  // For decoder-based PSRAM, pass cs=255 (unused).
  SPIHWAdapter(uint8_t pin_cs = 255, uint8_t pin_miso = 12, uint8_t pin_mosi = 11, uint8_t pin_sck = 10)
    : _cs(pin_cs), _miso(pin_miso), _mosi(pin_mosi), _sck(pin_sck),
      _clockHz(20000000UL), _useQuad(false), _io2(255), _io3(255) {
    _settings = SPISettings(_clockHz, MSBFIRST, SPI_MODE0);
  }

  void begin() {
    // Pin assignment must be done before begin()
    // Philhower core allows arbitrary valid pin mapping for a given SPI instance
    PSRAM_SPI_INSTANCE.setRX(_miso);
    PSRAM_SPI_INSTANCE.setTX(_mosi);
    PSRAM_SPI_INSTANCE.setSCK(_sck);
    PSRAM_SPI_INSTANCE.begin();

    if (_cs != 255) {
      pinMode(_cs, OUTPUT);
      digitalWrite(_cs, HIGH);
    }
  }

  // Convert half-cycle delay to an approximate SPI clock.
  // If 0 => fast (try 40 MHz, reduce if needed)
  void setClockDelayUs(uint8_t halfCycleDelayUs) {
    if (halfCycleDelayUs == 0) {
      _clockHz = 40000000UL;  // try 40MHz
    } else {
      // period_us = 2 * halfCycleDelayUs; freq ~= 1e6 / period_us
      uint32_t periodUs = (uint32_t)halfCycleDelayUs * 2u;
      if (periodUs == 0) periodUs = 1;
      uint32_t hz = 1000000UL / periodUs;
      if (hz < 100000UL) hz = 100000UL;  // floor to 100kHz
      _clockHz = hz;
    }
    _settings = SPISettings(_clockHz, MSBFIRST, SPI_MODE0);
  }

  // No-ops for compatibility
  void setExtraDataPins(uint8_t io2, uint8_t io3) {
    _io2 = io2;
    _io3 = io3;
  }
  void setModeQuad(bool enable) {
    _useQuad = enable;
  }

  // Single-byte transfer (full-duplex)
  inline uint8_t transfer(uint8_t tx) {
    PSRAM_SPI_INSTANCE.beginTransaction(_settings);
    uint8_t rx = PSRAM_SPI_INSTANCE.transfer(tx);
    PSRAM_SPI_INSTANCE.endTransaction();
    return rx;
  }

  // Full-duplex or half-duplex transfer
  // - If tx && rx: rx[i] = transfer(tx[i])
  // - If tx && !rx: write only
  // - If !tx && rx: read 'len' bytes by clocking out 0x00
  inline void transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    if (!len) return;
    PSRAM_SPI_INSTANCE.beginTransaction(_settings);
    if (tx && rx) {
      for (size_t i = 0; i < len; ++i) rx[i] = PSRAM_SPI_INSTANCE.transfer(tx[i]);
    } else if (tx && !rx) {
      for (size_t i = 0; i < len; ++i) PSRAM_SPI_INSTANCE.transfer(tx[i]);
    } else if (!tx && rx) {
      for (size_t i = 0; i < len; ++i) rx[i] = PSRAM_SPI_INSTANCE.transfer((uint8_t)0x00);
    }
    PSRAM_SPI_INSTANCE.endTransaction();
  }

private:
  uint8_t _cs, _miso, _mosi, _sck;
  uint32_t _clockHz;
  SPISettings _settings;
  bool _useQuad;
  uint8_t _io2, _io3;
};

#else
#error "SPIHWAdapter: RP2040 + Arduino (Philhower) required."
#endif