#pragma once
/*
  MX35LF.h - Minimal HW-SPI helper for Macronix MX35LF-series SPI-NAND
  - Correct ID sequences per datasheet:
      0x9F -> [DUMMY][MID][DID1][DID2]
      0x90 -> [DUMMY][00h][DID1][DID2] (some variants)
  - Decodes common densities; includes DID1=0x37 -> 4 Gbit (512 MiB), DID1=0x26 -> 2 Gbit (256 MiB)
*/

#include <Arduino.h>
#include <SPI.h>
#include <string.h>

#ifndef MX35_SPI_INSTANCE
#define MX35_SPI_INSTANCE SPI1
#endif
#ifndef MX35_SPI_CLOCK_HZ
#define MX35_SPI_CLOCK_HZ 8000000UL  // 8 MHz safe for ID/status
#endif

class MX35LF {
public:
  struct IdInfo {
    uint8_t mid;          // Manufacturer ID (Macronix = 0xC2)
    uint8_t did1;         // Device/density code
    uint8_t did2;         // Additional ID (revision/family)
    uint64_t totalBytes;  // Total capacity in bytes (0 if unknown)
    uint32_t pageSize;    // typical 2048
    uint32_t spareSize;   // typical 64 or 128
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

  // Reset (0xFF), then wait until not busy
  bool reset(uint32_t timeoutMs = 50) {
    csLow();
    beginTx();
    MX35_SPI_INSTANCE.transfer((uint8_t)0xFF);
    endTx();
    csHigh();
    return waitReady(timeoutMs);
  }

  // Get Feature (0x0F), e.g. addr 0xC0 = Status
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

  // Read ID (0x9F): returns 4 bytes: [dummy, MID, DID1, DID2]
  void readId9F(uint8_t &mid, uint8_t &did1, uint8_t &did2, uint8_t &dummyOut) {
    uint8_t b0 = 0, b1 = 0, b2 = 0, b3 = 0;
    csLow();
    beginTx();
    MX35_SPI_INSTANCE.transfer((uint8_t)0x9F);
    b0 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);  // dummy
    b1 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);  // MID
    b2 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);  // DID1
    b3 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);  // DID2
    endTx();
    csHigh();
    dummyOut = b0;
    mid = b1;
    did1 = b2;
    did2 = b3;
  }

  // Legacy Read ID (0x90): returns [DUMMY][00h][DID1][DID2]
  void readId90(uint8_t &did1, uint8_t &did2, uint8_t &dummyOut) {
    uint8_t dmy = 0, a0 = 0, d1 = 0, d2 = 0;
    csLow();
    beginTx();
    MX35_SPI_INSTANCE.transfer((uint8_t)0x90);
    dmy = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);  // dummy
    a0 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);   // fixed 0x00
    d1 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);
    d2 = MX35_SPI_INSTANCE.transfer((uint8_t)0x00);
    endTx();
    csHigh();
    (void)a0;
    dummyOut = dmy;
    did1 = d1;
    did2 = d2;
  }

  // Identify and fill IdInfo (returns true if recognized capacity)
  bool identify(IdInfo &info) {
    memset(&info, 0, sizeof(info));
    info.pageSize = 2048;
    info.spareSize = 64;

    uint8_t mid = 0, did1 = 0, did2 = 0, dmy = 0;
    readId9F(mid, did1, did2, dmy);

    if (mid != 0xC2 || mid == 0x00 || mid == 0xFF) {
      uint8_t d1 = 0, d2 = 0, dmy2 = 0;
      readId90(d1, d2, dmy2);
      if (d1 != 0x00 && d1 != 0xFF) {
        mid = 0xC2;  // Macronix implied
        did1 = d1;
        did2 = d2;
      }
    }

    info.mid = mid;
    info.did1 = did1;
    info.did2 = did2;

    if (mid == 0xC2) {
      switch (did1) {
        case 0x12: info.totalBytes = (uint64_t)128 * 1024 * 1024; return true;  // 1 Gbit
        case 0x22: info.totalBytes = (uint64_t)256 * 1024 * 1024; return true;  // 2 Gbit
        case 0x2C: info.totalBytes = (uint64_t)512 * 1024 * 1024; return true;  // 4 Gbit (older code)
        case 0x26:
          info.totalBytes = (uint64_t)256 * 1024 * 1024;
          return true;  // 2 Gbit (your part)
        case 0x37:
          info.totalBytes = (uint64_t)512 * 1024 * 1024;
          return true;  // 4 Gbit (your part)
        // Add more DID1 codes here when known
        default: break;
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