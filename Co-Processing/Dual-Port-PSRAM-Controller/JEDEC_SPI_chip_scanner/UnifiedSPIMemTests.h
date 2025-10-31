#pragma once
/*
  UnifiedSPIMemTests.h
  Test runner for UnifiedSPIMem with pretty console formatting.

  Principles:
    - Only use UnifiedSpiMem public API (Manager + MemDevice).
    - No direct/vendor-specific device access.
    - Safe suite: non-destructive.
    - Destructive suite: uniform tail-region wipe/write/verify per device type.
    - PSRAM is fully included in both safe and destructive tests.

  You can pass any Stream (Serial, Serial1, etc.) for console output.
*/

#include <Arduino.h>
#include "UnifiedSPIMem.h"

#ifndef UNIFIEDSPIMEM_TESTS_INCLUDED
#define UNIFIEDSPIMEM_TESTS_INCLUDED

namespace UnifiedSpiMem {

// Pretty output configuration
struct PrettyConfig {
  bool enable = true;
  int colwCS = 4;       // right-aligned
  int colwType = 18;    // left
  int colwVID = 8;      // left ("0xEF")
  int colwVendor = 22;  // left
  int colwCap = 10;     // right ("512 MiB")
  int colwHint = 24;    // left
};

// Full suite configuration
struct SuiteConfig {
  // SPI pins
  uint8_t pinSCK = 10;
  uint8_t pinMOSI = 11;
  uint8_t pinMISO = 12;
  int8_t pinWP = -1;
  int8_t pinHOLD = -1;

  // CS list to scan
  const uint8_t* csList = nullptr;
  size_t csCount = 0;

  // Identify options
  uint32_t spiHzForIdentify = UNIFIED_SPI_CLOCK_HZ;
  bool preservePsramContents = false;

  // Console (defaults to Serial if null)
  Stream* console = nullptr;

  // Inventory print
  bool printDeviceTable = true;
  PrettyConfig pretty{};

  // Destructive suite tuning (generic, driver-surface only)
  size_t regionBytes = 0;      // 0 = auto: prefer eraseSize(), else pageSize(), else 4096
  uint8_t tryTailRegions = 4;  // how many tail regions to attempt if the first fails
  bool alignToErase = true;    // align region base to eraseSize boundary when available
  bool restoreAfter = true;    // try to restore region (erase if supported, else fill)
  uint8_t restoreFill = 0x00;  // fill byte for devices without erase (e.g., PSRAM)
};

// Pretty printer for aligned table
class PrettyPrinter {
public:
  PrettyPrinter(Stream& out, const PrettyConfig& cfg)
    : _out(out), _cfg(cfg) {}

  void header() {
    divider();
    printHeaderRow();
    divider();
  }

  void row(const DeviceInfo& di) {
    char csBuf[8];
    snprintf(csBuf, sizeof(csBuf), "%u", (unsigned)di.cs);
    char vidBuf[8];
    snprintf(vidBuf, sizeof(vidBuf), "0x%02X", di.vendorId);
    char capBuf[16];
    snprintf(capBuf, sizeof(capBuf), "%lu MiB", (unsigned long)(di.capacityBytes >> 20));

    const char* typeName = deviceTypeName(di.type);
    const char* vendor = di.vendorName ? di.vendorName : "Unknown";
    const char* hint = di.partHint ? di.partHint : "-";

    _out.write('|');
    _out.write(' ');
    pad(csBuf, _cfg.colwCS, true);
    _out.write(' ');
    _out.write('|');
    _out.write(' ');
    pad(typeName, _cfg.colwType, false);
    _out.write(' ');
    _out.write('|');
    _out.write(' ');
    pad(vidBuf, _cfg.colwVID, false);
    _out.write(' ');
    _out.write('|');
    _out.write(' ');
    pad(vendor, _cfg.colwVendor, false);
    _out.write(' ');
    _out.write('|');
    _out.write(' ');
    pad(capBuf, _cfg.colwCap, true);
    _out.write(' ');
    _out.write('|');
    _out.write(' ');
    pad(hint, _cfg.colwHint, false);
    _out.write(' ');
    _out.write('|');
    _out.println();
  }

  void footer() {
    divider();
  }

  void printPassFail(const char* name, bool pass) {
    _out.write('[');
    _out.print(pass ? "PASS" : "FAIL");
    _out.print("] ");
    _out.println(name);
  }

private:
  void divider() {
    const int total = 1 + _cfg.colwCS + 2 + _cfg.colwType + 3 + _cfg.colwVID + 3 + _cfg.colwVendor + 3 + _cfg.colwCap + 3 + _cfg.colwHint + 4;
    for (int i = 0; i < total; ++i) _out.write((i == 0 || i == total - 1) ? '+' : '-');
    _out.println();
  }

  void printHeaderRow() {
    _out.write('|');
    _out.write(' ');
    pad("CS", _cfg.colwCS, true);
    _out.write(' ');
    _out.write('|');
    _out.write(' ');
    pad("Type", _cfg.colwType, false);
    _out.write(' ');
    _out.write('|');
    _out.write(' ');
    pad("VendorID", _cfg.colwVID, false);
    _out.write(' ');
    _out.write('|');
    _out.write(' ');
    pad("Vendor", _cfg.colwVendor, false);
    _out.write(' ');
    _out.write('|');
    _out.write(' ');
    pad("Capacity", _cfg.colwCap, true);
    _out.write(' ');
    _out.write('|');
    _out.write(' ');
    pad("PartHint", _cfg.colwHint, false);
    _out.write(' ');
    _out.write('|');
    _out.println();
  }

  void pad(const char* s, int width, bool rightAlign) {
    if (!s) s = "-";
    int len = (int)strlen(s);
    if (len > width) {
      if (width <= 0) return;
      for (int i = 0; i < width - 1; ++i) _out.write(s[i]);
      _out.write('.');
      return;
    }
    if (rightAlign)
      for (int i = 0; i < width - len; ++i) _out.write(' ');
    _out.print(s);
    if (!rightAlign)
      for (int i = 0; i < width - len; ++i) _out.write(' ');
  }

  Stream& _out;
  const PrettyConfig& _cfg;
};

class SuiteRunner {
public:
  explicit SuiteRunner(const SuiteConfig& cfg)
    : _cfg(cfg),
      _out(cfg.console ? *cfg.console : Serial),
      _mgr(cfg.pinSCK, cfg.pinMOSI, cfg.pinMISO, cfg.pinWP, cfg.pinHOLD),
      _pp(_out, _cfg.pretty) {
    _mgr.begin();
    _mgr.setPreservePsramContents(_cfg.preservePsramContents);
  }

  // Scan; print inventory with pretty formatting if enabled
  size_t scan() {
    if (!_cfg.csList || _cfg.csCount == 0) return 0;
    const size_t found = _mgr.scan(_cfg.csList, _cfg.csCount, _cfg.spiHzForIdentify);
    if (_cfg.printDeviceTable) {
      if (found == 0) {
        _out.println("No devices detected.");
      } else {
        _pp.header();
        for (size_t i = 0; i < found; ++i) {
          const auto* di = _mgr.detectedInfo(i);
          if (di) _pp.row(*di);
        }
        _pp.footer();
      }
    }
    return found;
  }

  // Safe (non-destructive) tests
  bool runSafe() {
    _ran = _pass = 0;
    ensureScanned();

    printResult("openAuto() yields a valid handle", testOpenAuto());

    if (hasType(DeviceType::Psram)) printResult("openPreferred(PSRAM)", testOpenPreferredType(DeviceType::Psram));
    if (hasType(DeviceType::NorW25Q)) printResult("openPreferred(NOR)", testOpenPreferredType(DeviceType::NorW25Q));
    if (hasType(DeviceType::SpiNandMX35)) printResult("openPreferred(SPI-NAND)", testOpenPreferredType(DeviceType::SpiNandMX35));

    if (hasType(DeviceType::Psram)) printResult("PSRAM: read 32 bytes @0x000000", testReadFirstBytes(DeviceType::Psram, 32));
    if (hasType(DeviceType::NorW25Q)) printResult("NOR: read 32 bytes @0x000000", testReadFirstBytes(DeviceType::NorW25Q, 32));
    if (hasType(DeviceType::SpiNandMX35)) printResult("SPI-NAND: read 32 bytes @0x000000", testReadFirstBytes(DeviceType::SpiNandMX35, 32));

    // PSRAM safe write+verify (volatile)
    if (hasType(DeviceType::Psram)) printResult("PSRAM: write 64B + verify", testPsramWriteRead());

    if (hasTwoDifferentTypes()) printResult("Open two different device types simultaneously", testSimultaneousDifferentTypes());

    if (_mgr.detectedCount() > 0) printResult("DevicePool: open/close all devices", testPoolOpenAll());

    printSummary();
    return (_ran == _pass);
  }

  // Destructive tests (generic, via MemDevice only)
  bool runDestructive() {
    _ran = _pass = 0;
    ensureScanned();

    // Apply the same generic destructive process to each present type
    if (hasType(DeviceType::Psram)) printResult("PSRAM (generic destructive): tail region write/verify + restore", testDestructiveGeneric(DeviceType::Psram));
    if (hasType(DeviceType::NorW25Q)) printResult("NOR (generic destructive): tail region erase + write/verify + restore", testDestructiveGeneric(DeviceType::NorW25Q));
    if (hasType(DeviceType::SpiNandMX35)) printResult("SPI-NAND (generic destructive): tail region erase + write/verify + restore", testDestructiveGeneric(DeviceType::SpiNandMX35));

    printSummary();
    return (_ran == _pass);
  }

  // Accessors
  size_t lastRan() const {
    return _ran;
  }
  size_t lastPass() const {
    return _pass;
  }
  Manager& manager() {
    return _mgr;
  }

private:
  // ---------- Common helpers (driver-surface only) ----------
  void ensureScanned() {
    if (_mgr.detectedCount() == 0) scan();
  }

  bool hasType(DeviceType t) {
    return (_mgr.findIndexByType(t, 0, false) >= 0);
  }

  bool hasTwoDifferentTypes() {
    const bool p = hasType(DeviceType::Psram);
    const bool n = hasType(DeviceType::NorW25Q);
    const bool d = hasType(DeviceType::SpiNandMX35);
    return ((p && n) || (p && d) || (n && d));
  }

  void printResult(const char* label, bool ok) {
    _ran++;
    if (ok) _pass++;
    _pp.printPassFail(label, ok);
  }

  void printSummary() {
    _out.println();
    _out.print("Test summary: ");
    _out.print(_pass);
    _out.print("/");
    _out.print(_ran);
    _out.println(" passed.");
    _out.println();
  }

  // Pattern generator (deterministic)
  static inline uint8_t patternByte(uint64_t offset, uint8_t seed = 0x3C) {
    return (uint8_t)(seed ^ ((offset & 0xFF) * 7) ^ ((offset >> 8) & 0xAA));
  }

  bool writePattern(MemDevice* dev, uint64_t base, size_t len, uint8_t seed) {
    if (!dev || len == 0) return false;
    uint8_t buf[512];
    size_t done = 0;
    while (done < len) {
      size_t chunk = (len - done > sizeof(buf)) ? sizeof(buf) : (len - done);
      for (size_t i = 0; i < chunk; ++i)
        buf[i] = patternByte(done + i, seed);
      if (!dev->write(base + done, buf, chunk)) return false;
      done += chunk;
    }
    return true;
  }

  bool verifyPattern(MemDevice* dev, uint64_t base, size_t len, uint8_t seed) {
    if (!dev || len == 0) return false;
    uint8_t buf[512];
    size_t done = 0;
    while (done < len) {
      size_t chunk = (len - done > sizeof(buf)) ? sizeof(buf) : (len - done);
      size_t rd = dev->read(base + done, buf, chunk);
      if (rd != chunk) return false;
      for (size_t i = 0; i < chunk; ++i) {
        if (buf[i] != patternByte(done + i, seed)) return false;
      }
      done += chunk;
    }
    return true;
  }

  bool fillRegion(MemDevice* dev, uint64_t base, size_t len, uint8_t value) {
    if (!dev || len == 0) return false;
    uint8_t buf[512];
    memset(buf, value, sizeof(buf));
    size_t done = 0;
    while (done < len) {
      size_t chunk = (len - done > sizeof(buf)) ? sizeof(buf) : (len - done);
      if (!dev->write(base + done, buf, chunk)) return false;
      done += chunk;
    }
    return true;
  }

  bool verifyFill(MemDevice* dev, uint64_t base, size_t len, uint8_t value) {
    if (!dev || len == 0) return false;
    uint8_t buf[512];
    size_t done = 0;
    while (done < len) {
      size_t chunk = (len - done > sizeof(buf)) ? sizeof(buf) : (len - done);
      size_t rd = dev->read(base + done, buf, chunk);
      if (rd != chunk) return false;
      for (size_t i = 0; i < chunk; ++i)
        if (buf[i] != value) return false;
      done += chunk;
    }
    return true;
  }

  // Pick a generic tail region (driver-surface only)
  void chooseRegion(MemDevice* dev, uint64_t& base, size_t& len) {
    uint64_t cap = dev->capacity();
    uint32_t e = dev->eraseSize();
    uint32_t p = dev->pageSize();

    size_t want = _cfg.regionBytes;
    if (want == 0) {
      if (e != 0) want = e;
      else if (p != 0) want = p;
      else want = 4096;
    }
    if ((uint64_t)want > cap) want = (size_t)cap;

    uint64_t b = cap - (uint64_t)want;

    if (_cfg.alignToErase && e != 0) {
      uint64_t aligned = (cap / e) * (uint64_t)e;  // end-aligned
      if (aligned >= e) aligned -= e;
      b = aligned;
      len = e;  // ensure we operate a full erase unit
    } else {
      len = want;
    }

    base = b;
  }

  // Helpers: page-aligned program/verify (prefer 1 program per page)
  static bool writePatternPaged(UnifiedSpiMem::MemDevice* dev, uint64_t base, size_t len, uint32_t pageSize, uint8_t seed) {
    if (!dev || len == 0) return false;
    if (pageSize == 0) return false;

    const size_t BUF_MAX = 2048;  // large enough for typical NAND page (MX35 = 2 KiB)
    uint8_t buf[BUF_MAX];
    if (pageSize > BUF_MAX) return false;

    size_t written = 0;
    while (written < len) {
      size_t chunk = (len - written >= pageSize) ? pageSize : (len - written);
      // Generate deterministic pattern for this page segment
      for (size_t i = 0; i < chunk; ++i) {
        buf[i] = (uint8_t)(seed ^ (((written + i) & 0xFF) * 7) ^ (((written + i) >> 8) & 0xAA));
      }
      // One write call per page for NAND/NOR
      if (!dev->write(base + written, buf, chunk)) return false;
      written += chunk;
    }
    return true;
  }

  static bool verifyPatternPaged(UnifiedSpiMem::MemDevice* dev, uint64_t base, size_t len, uint32_t pageSize, uint8_t seed) {
    if (!dev || len == 0) return false;
    if (pageSize == 0) return false;

    const size_t BUF_MAX = 2048;
    uint8_t buf[BUF_MAX];
    if (pageSize > BUF_MAX) return false;

    size_t checked = 0;
    while (checked < len) {
      size_t chunk = (len - checked >= pageSize) ? pageSize : (len - checked);
      size_t rd = dev->read(base + checked, buf, chunk);
      if (rd != chunk) return false;
      for (size_t i = 0; i < chunk; ++i) {
        uint8_t expect = (uint8_t)(seed ^ (((checked + i) & 0xFF) * 7) ^ (((checked + i) >> 8) & 0xAA));
        if (buf[i] != expect) return false;
      }
      checked += chunk;
    }
    return true;
  }

  // Helper: verify an erased region by sampling (tolerant, majority-FF)
  static bool verifyErasedRegion(UnifiedSpiMem::MemDevice* dev, uint64_t base, size_t len, uint32_t pageSize) {
    if (!dev || len == 0) return false;

    // Sample up to 3 positions: start/middle/end (bounded)
    const size_t sampleWindows = 3;
    const size_t winSize = (pageSize && pageSize <= 1024) ? pageSize : 1024;
    uint8_t buf[1024];

    uint64_t points[sampleWindows];
    points[0] = base;
    points[1] = base + (len > winSize ? (len - winSize) / 2 : 0);
    points[2] = (len > winSize) ? (base + len - winSize) : base;

    for (size_t s = 0; s < sampleWindows; ++s) {
      uint64_t addr = points[s];
      size_t rd = dev->read(addr, buf, winSize);
      if (rd != winSize) return false;
      size_t ff = 0;
      for (size_t i = 0; i < winSize; ++i)
        if (buf[i] == 0xFF) ff++;
      // Require strong majority of 0xFF to account for spare/metadata quirks
      if (ff < (winSize * 9) / 10) return false;
    }
    return true;
  }

  // Generic destructive: tail region per type, using only MemDevice
  bool testDestructiveGeneric(UnifiedSpiMem::DeviceType t) {
    auto* dev = _mgr.openPreferred(t);
    if (!dev) return false;

    bool okOverall = false;

    // Choose a region near the tail
    uint64_t regionBase = 0;
    size_t regionLen = 0;
    chooseRegion(dev, regionBase, regionLen);
    if (regionLen == 0 || dev->capacity() < regionLen) {
      _mgr.release(dev);
      return false;
    }

    const uint32_t eraseSz = dev->eraseSize();
    const uint32_t pageSz = dev->pageSize();
    const uint64_t step = (eraseSz ? eraseSz : (pageSz ? pageSz : regionLen));

    // Align base to page boundary when possible (helps NAND), while staying within capacity
    if (pageSz) {
      uint64_t aligned = (regionBase / pageSz) * (uint64_t)pageSz;
      if (aligned + regionLen <= dev->capacity()) regionBase = aligned;
    }

    // For NAND/NOR, prefer region length to be a multiple of page size to avoid partial-page programs
    if (pageSz && regionLen >= pageSz) {
      regionLen = (regionLen / pageSz) * pageSz;
      if (regionLen == 0) regionLen = pageSz;  // at least one page if possible
    }

    const uint8_t patternSeed = 0x5A;

    for (uint8_t attempt = 0; attempt < _cfg.tryTailRegions; ++attempt) {
      uint64_t base = (regionBase >= (uint64_t)attempt * step) ? (regionBase - (uint64_t)attempt * step) : 0;
      bool ok = true;

      // If supported, erase and verify erased before programming
      if (eraseSz) {
        ok = dev->eraseRange(base, regionLen);
        if (ok) ok = verifyErasedRegion(dev, base, regionLen, pageSz);
        // If erase verify fails, skip to next region
        if (!ok) continue;
      }

      // Program pattern (prefer page-aligned writes to avoid multiple partial programs)
      if (pageSz && regionLen >= pageSz) {
        ok = writePatternPaged(dev, base, regionLen, pageSz, patternSeed);
        if (ok) ok = verifyPatternPaged(dev, base, regionLen, pageSz, patternSeed);
      } else {
        // Fallback streaming (small devices / unknown page size)
        ok = writePattern(dev, base, regionLen, patternSeed);
        if (ok) ok = verifyPattern(dev, base, regionLen, patternSeed);
      }

      if (!ok) {
        // Try next tail region
        continue;
      }

      // Optional restore
      if (_cfg.restoreAfter) {
        if (eraseSz) {
          ok = dev->eraseRange(base, regionLen);
          if (ok) ok = verifyErasedRegion(dev, base, regionLen, pageSz);
        } else {
          ok = fillRegion(dev, base, regionLen, _cfg.restoreFill);
          if (ok) ok = verifyFill(dev, base, regionLen, _cfg.restoreFill);
        }
      }

      if (ok) {
        okOverall = true;
        break;
      }
    }

    _mgr.release(dev);
    return okOverall;
  }

  // ---------- Safe tests ----------
  bool testOpenAuto() {
    auto dev = _mgr.openAuto();
    if (!dev) return false;
    bool ok = (dev->capacity() > 0);
    _mgr.release(dev);
    return ok;
  }

  bool testOpenPreferredType(DeviceType t) {
    auto dev = _mgr.openPreferred(t);
    if (!dev) return false;
    bool ok = (dev->type() == t) && (dev->capacity() > 0);
    _mgr.release(dev);
    return ok;
  }

  bool testReadFirstBytes(DeviceType t, size_t nBytes) {
    auto dev = _mgr.openPreferred(t);
    if (!dev) return false;
    uint8_t buf[64];
    if (nBytes > sizeof(buf)) nBytes = sizeof(buf);
    size_t rd = dev->read(0, buf, nBytes);
    bool ok = (rd == nBytes);
    _mgr.release(dev);
    return ok;
  }

  bool testPsramWriteRead() {
    auto dev = _mgr.openPreferred(DeviceType::Psram);
    if (!dev) return false;
    const uint32_t addr = 0x000100;
    const size_t len = 64;
    uint8_t tx[len], rx[len];
    for (size_t i = 0; i < len; ++i) tx[i] = (uint8_t)(0xA5 ^ i);
    bool ok = dev->write(addr, tx, len);
    if (ok) {
      memset(rx, 0, sizeof(rx));
      size_t rd = dev->read(addr, rx, len);
      if (rd != len) ok = false;
      if (ok) {
        for (size_t i = 0; i < len; ++i) {
          if (rx[i] != tx[i]) {
            ok = false;
            break;
          }
        }
      }
    }
    _mgr.release(dev);
    return ok;
  }

  bool testSimultaneousDifferentTypes() {
    MemDevice* a = nullptr;
    MemDevice* b = nullptr;
    if (hasType(DeviceType::Psram) && hasType(DeviceType::NorW25Q)) {
      a = _mgr.openPreferred(DeviceType::Psram);
      b = _mgr.openPreferred(DeviceType::NorW25Q);
    } else if (hasType(DeviceType::Psram) && hasType(DeviceType::SpiNandMX35)) {
      a = _mgr.openPreferred(DeviceType::Psram);
      b = _mgr.openPreferred(DeviceType::SpiNandMX35);
    } else if (hasType(DeviceType::NorW25Q) && hasType(DeviceType::SpiNandMX35)) {
      a = _mgr.openPreferred(DeviceType::NorW25Q);
      b = _mgr.openPreferred(DeviceType::SpiNandMX35);
    } else {
      return false;
    }
    bool ok = (a && b && (a->cs() != b->cs()));
    if (a) _mgr.release(a);
    if (b) _mgr.release(b);
    return ok;
  }

  bool testPoolOpenAll() {
    auto* pool = DevicePool::createAll(_mgr, false);
    if (!pool) return false;
    bool ok = (pool->size() == _mgr.detectedCount());
    for (size_t i = 0; i < pool->size(); ++i) {
      auto* dev = pool->openAt(i);
      if (!dev) {
        ok = false;
        break;
      }
      uint8_t b[8];
      size_t rd = dev->read(0, b, sizeof(b));
      if (rd != sizeof(b)) ok = false;
      pool->poolRelease(i, dev);
      if (!ok) break;
    }
    delete pool;
    return ok;
  }

private:
  SuiteConfig _cfg;
  Stream& _out;
  Manager _mgr;
  PrettyPrinter _pp;
  size_t _ran = 0;
  size_t _pass = 0;
};

// Convenience free functions
inline bool RunSafeSuite(const SuiteConfig& cfg) {
  SuiteRunner runner(cfg);
  runner.scan();
  return runner.runSafe();
}

inline bool RunDestructiveSuite(const SuiteConfig& cfg) {
  SuiteRunner runner(cfg);
  runner.scan();
  return runner.runDestructive();
}

}  // namespace UnifiedSpiMem

#endif  // UNIFIEDSPIMEM_TESTS_INCLUDED