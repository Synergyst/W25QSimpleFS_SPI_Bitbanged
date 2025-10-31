/*
  UnifiedSPIMem_demo.ino
  - Scans a list of CS pins for PSRAM, NOR (W25Q), and SPI-NAND (MX35LF).
  - Opens/closes devices in various combinations.
  - Runs small, non-destructive read tests (and a small PSRAM write+verify).
  - Prints PASS/FAIL for each test.

  Wiring defaults assume RP2040 SPI1 as the library does:
    SCK=10, MOSI=11, MISO=12
  Edit CS_PINS to match your connected devices.
*/

#include <Arduino.h>
#include "UnifiedSPIMem.h"

// ----- SPI pins (match your board/wiring) -----
static const uint8_t PIN_SCK = 10;   // default RP2040-SPI1 SCK
static const uint8_t PIN_MOSI = 11;  // default RP2040-SPI1 MOSI
static const uint8_t PIN_MISO = 12;  // default RP2040-SPI1 MISO

// Optional WP/HOLD if used on your hardware; -1 = unused
static const int8_t PIN_WP = -1;
static const int8_t PIN_HOLD = -1;

// ----- CS pins to probe (edit this list to match your wiring) -----
// Include PSRAM CS if populated (default library uses CS=9 for PSRAMBitbang)
static const uint8_t CS_PINS[] = { 9, 14, 15 };
static const size_t CS_COUNT = sizeof(CS_PINS) / sizeof(CS_PINS[0]);

// ----- Simple test-result logger -----
struct TestResult {
  const char* name;
  bool ran;
  bool pass;
};

static const size_t MAX_TESTS = 16;
static TestResult g_results[MAX_TESTS];
static size_t g_resultCount = 0;

void logTest(const char* name, bool ran, bool pass) {
  if (g_resultCount < MAX_TESTS) {
    g_results[g_resultCount++] = { name, ran, pass };
  }
  if (ran) {
    Serial.print("[");
    Serial.print(pass ? "PASS" : "FAIL");
    Serial.print("] ");
    Serial.println(name);
  }
  // "silently skip" means no output when not applicable
}

// ----- Pretty table formatting for device list -----

// Column widths (adjust if you like)
static const int COLW_CS = 4;       // right-aligned
static const int COLW_TYPE = 18;    // left-aligned
static const int COLW_VID = 8;      // left-aligned (e.g. 0xEF)
static const int COLW_VENDOR = 22;  // left-aligned
static const int COLW_CAP = 10;     // right-aligned (e.g. 512 MiB)
static const int COLW_HINT = 24;    // left-aligned

static inline void printSpaces(int n) {
  while (n-- > 0) Serial.write(' ');
}

static void printPad(const char* s, int width, bool rightAlign = false) {
  if (!s) s = "-";
  int len = (int)strlen(s);
  if (len > width) {
    // Truncate with ellipsis if it doesn't fit
    if (width >= 2) {
      for (int i = 0; i < width - 1; ++i) Serial.write(s[i]);
      Serial.write('.');
    } else if (width == 1) {
      Serial.write(s[0]);
    }
    return;
  }
  if (rightAlign) printSpaces(width - len);
  Serial.print(s);
  if (!rightAlign) printSpaces(width - len);
}

static void printDivider() {
  const int total = 1 + COLW_CS + 2 + COLW_TYPE + 3 + COLW_VID + 3 + COLW_VENDOR + 3 + COLW_CAP + 3 + COLW_HINT + 4;
  for (int i = 0; i < total; ++i) Serial.write(i == 0 || i == total - 1 ? '+' : '-');
  Serial.println();
}

void printDeviceTableHeader() {
  printDivider();
  Serial.write('|');
  Serial.write(' ');
  printPad("CS", COLW_CS, true);
  Serial.write(' ');
  Serial.write('|');
  Serial.write(' ');
  printPad("Type", COLW_TYPE);
  Serial.write(' ');
  Serial.write('|');
  Serial.write(' ');
  printPad("VendorID", COLW_VID);
  Serial.write(' ');
  Serial.write('|');
  Serial.write(' ');
  printPad("Vendor", COLW_VENDOR);
  Serial.write(' ');
  Serial.write('|');
  Serial.write(' ');
  printPad("Capacity", COLW_CAP, true);
  Serial.write(' ');
  Serial.write('|');
  Serial.write(' ');
  printPad("PartHint", COLW_HINT);
  Serial.write(' ');
  Serial.write('|');
  Serial.println();
  printDivider();
}

void printDeviceRow(const UnifiedSpiMem::DeviceInfo& di) {
  // Build small fields as strings
  char csBuf[8];
  snprintf(csBuf, sizeof(csBuf), "%u", (unsigned)di.cs);

  char vidBuf[8];
  snprintf(vidBuf, sizeof(vidBuf), "0x%02X", di.vendorId);

  char capBuf[16];
  snprintf(capBuf, sizeof(capBuf), "%lu MiB", (unsigned long)(di.capacityBytes >> 20));

  const char* typeName = UnifiedSpiMem::deviceTypeName(di.type);
  const char* vendor = di.vendorName ? di.vendorName : "Unknown";
  const char* hint = di.partHint ? di.partHint : "-";

  Serial.write('|');
  Serial.write(' ');
  printPad(csBuf, COLW_CS, true);
  Serial.write(' ');
  Serial.write('|');
  Serial.write(' ');
  printPad(typeName, COLW_TYPE);
  Serial.write(' ');
  Serial.write('|');
  Serial.write(' ');
  printPad(vidBuf, COLW_VID);
  Serial.write(' ');
  Serial.write('|');
  Serial.write(' ');
  printPad(vendor, COLW_VENDOR);
  Serial.write(' ');
  Serial.write('|');
  Serial.write(' ');
  printPad(capBuf, COLW_CAP, true);
  Serial.write(' ');
  Serial.write('|');
  Serial.write(' ');
  printPad(hint, COLW_HINT);
  Serial.write(' ');
  Serial.write('|');
  Serial.println();
}

bool hasType(UnifiedSpiMem::Manager& mgr, UnifiedSpiMem::DeviceType t) {
  return (mgr.findIndexByType(t, 0, false) >= 0);
}

UnifiedSpiMem::MemDevice* openPreferredIfPresent(UnifiedSpiMem::Manager& mgr, UnifiedSpiMem::DeviceType t) {
  if (!hasType(mgr, t)) return nullptr;
  return mgr.openPreferred(t);
}

bool testOpenAuto(UnifiedSpiMem::Manager& mgr) {
  auto dev = mgr.openAuto();
  if (!dev) return false;
  bool ok = (dev->capacity() > 0);
  mgr.release(dev);
  return ok;
}

bool testOpenPreferredType(UnifiedSpiMem::Manager& mgr, UnifiedSpiMem::DeviceType t) {
  if (!hasType(mgr, t)) return false;  // not ran (we'll treat as "skip" in caller)
  auto dev = mgr.openPreferred(t);
  if (!dev) return false;
  bool ok = (dev->type() == t) && (dev->capacity() > 0);
  mgr.release(dev);
  return ok;
}

bool testReadFirstBytes(UnifiedSpiMem::Manager& mgr, UnifiedSpiMem::DeviceType t, size_t nBytes = 32) {
  if (!hasType(mgr, t)) return false;  // not ran
  auto dev = mgr.openPreferred(t);
  if (!dev) return false;
  uint8_t buf[64];
  if (nBytes > sizeof(buf)) nBytes = sizeof(buf);
  size_t rd = dev->read(0, buf, nBytes);  // non-destructive
  bool ok = (rd == nBytes);
  mgr.release(dev);
  return ok;
}

bool testPsramWriteRead(UnifiedSpiMem::Manager& mgr) {
  if (!hasType(mgr, UnifiedSpiMem::DeviceType::Psram)) return false;  // not ran
  auto dev = mgr.openPreferred(UnifiedSpiMem::DeviceType::Psram);
  if (!dev) return false;

  const uint32_t addr = 0x000100;  // small offset
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

  mgr.release(dev);
  return ok;
}

bool testSimultaneousDifferentTypes(UnifiedSpiMem::Manager& mgr) {
  // Pick two different types if possible: priority Psram, then NOR, then NAND
  UnifiedSpiMem::MemDevice* a = nullptr;
  UnifiedSpiMem::MemDevice* b = nullptr;

  // Try PSRAM + NOR
  if (hasType(mgr, UnifiedSpiMem::DeviceType::Psram) && hasType(mgr, UnifiedSpiMem::DeviceType::NorW25Q)) {
    a = mgr.openPreferred(UnifiedSpiMem::DeviceType::Psram);
    b = mgr.openPreferred(UnifiedSpiMem::DeviceType::NorW25Q);
  } else if (hasType(mgr, UnifiedSpiMem::DeviceType::Psram) && hasType(mgr, UnifiedSpiMem::DeviceType::SpiNandMX35)) {
    a = mgr.openPreferred(UnifiedSpiMem::DeviceType::Psram);
    b = mgr.openPreferred(UnifiedSpiMem::DeviceType::SpiNandMX35);
  } else if (hasType(mgr, UnifiedSpiMem::DeviceType::NorW25Q) && hasType(mgr, UnifiedSpiMem::DeviceType::SpiNandMX35)) {
    a = mgr.openPreferred(UnifiedSpiMem::DeviceType::NorW25Q);
    b = mgr.openPreferred(UnifiedSpiMem::DeviceType::SpiNandMX35);
  } else {
    return false;  // not ran
  }

  bool ok = (a && b && (a->cs() != b->cs()));
  if (a) mgr.release(a);
  if (b) mgr.release(b);
  return ok;
}

bool testPoolOpenAll(UnifiedSpiMem::Manager& mgr) {
  if (mgr.detectedCount() == 0) return false;  // not ran
  // Don't pre-reserve, since openByIndex() can't open reserved entries
  auto* pool = UnifiedSpiMem::DevicePool::createAll(mgr, false);
  if (!pool) return false;

  bool ok = (pool->size() == mgr.detectedCount());

  // Open each from pool and immediately close
  for (size_t i = 0; i < pool->size(); ++i) {
    auto* dev = pool->openAt(i);
    if (!dev) {
      ok = false;
      break;
    }
    // tiny non-destructive read
    uint8_t b[8];
    size_t rd = dev->read(0, b, sizeof(b));
    if (rd != sizeof(b)) ok = false;
    pool->poolRelease(i, dev);
    if (!ok) break;
  }
  delete pool;
  return ok;
}

void setup() {
  delay(500);
  Serial.begin(115200);
  while (!Serial) { delay(200); }
  delay(200);
  Serial.println();
  Serial.println("UnifiedSpiMem demo: scan + open/close + basic tests");

  // Ensure CS pins start high (avoid bus contention during boot)
  for (size_t i = 0; i < CS_COUNT; ++i) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);
  }

  UnifiedSpiMem::Manager mgr(PIN_SCK, PIN_MOSI, PIN_MISO, PIN_WP, PIN_HOLD);
  mgr.begin();

  size_t found = mgr.scan(CS_PINS, CS_COUNT);
  Serial.print("Scanned CS pins: ");
  for (size_t i = 0; i < CS_COUNT; ++i) {
    Serial.print((int)CS_PINS[i]);
    if (i + 1 < CS_COUNT) Serial.print(", ");
  }
  Serial.println();

  Serial.print("Detected devices: ");
  Serial.println(found);
  if (found > 0) {
    printDeviceTableHeader();
    for (size_t i = 0; i < found; ++i) {
      const auto* di = mgr.detectedInfo(i);
      if (di) printDeviceRow(*di);
    }
    printDivider();
  }

  // ---- Run tests ----
  // Test: openAuto
  {
    bool ok = testOpenAuto(mgr);
    logTest("openAuto() yields a valid handle", true, ok);
  }

  // Test: openPreferred(PSRAM)
  {
    bool applicable = hasType(mgr, UnifiedSpiMem::DeviceType::Psram);
    bool ok = applicable ? testOpenPreferredType(mgr, UnifiedSpiMem::DeviceType::Psram) : false;
    logTest("openPreferred(PSRAM)", applicable, ok);
  }

  // Test: openPreferred(NOR)
  {
    bool applicable = hasType(mgr, UnifiedSpiMem::DeviceType::NorW25Q);
    bool ok = applicable ? testOpenPreferredType(mgr, UnifiedSpiMem::DeviceType::NorW25Q) : false;
    logTest("openPreferred(NOR)", applicable, ok);
  }

  // Test: openPreferred(SPI-NAND)
  {
    bool applicable = hasType(mgr, UnifiedSpiMem::DeviceType::SpiNandMX35);
    bool ok = applicable ? testOpenPreferredType(mgr, UnifiedSpiMem::DeviceType::SpiNandMX35) : false;
    logTest("openPreferred(SPI-NAND)", applicable, ok);
  }

  // Test: basic read from each type (non-destructive)
  {
    bool applicable = hasType(mgr, UnifiedSpiMem::DeviceType::Psram);
    bool ok = applicable ? testReadFirstBytes(mgr, UnifiedSpiMem::DeviceType::Psram, 32) : false;
    logTest("PSRAM: read 32 bytes @0x000000", applicable, ok);
  }
  {
    bool applicable = hasType(mgr, UnifiedSpiMem::DeviceType::NorW25Q);
    bool ok = applicable ? testReadFirstBytes(mgr, UnifiedSpiMem::DeviceType::NorW25Q, 32) : false;
    logTest("NOR: read 32 bytes @0x000000", applicable, ok);
  }
  {
    bool applicable = hasType(mgr, UnifiedSpiMem::DeviceType::SpiNandMX35);
    bool ok = applicable ? testReadFirstBytes(mgr, UnifiedSpiMem::DeviceType::SpiNandMX35, 32) : false;
    logTest("SPI-NAND: read 32 bytes @0x000000", applicable, ok);
  }

  // Test: PSRAM write+readback (safe; volatile)
  {
    bool applicable = hasType(mgr, UnifiedSpiMem::DeviceType::Psram);
    bool ok = applicable ? testPsramWriteRead(mgr) : false;
    logTest("PSRAM: write 64B + verify", applicable, ok);
  }

  // Test: Simultaneous open of two different types
  {
    bool applicable =
      (hasType(mgr, UnifiedSpiMem::DeviceType::Psram) && hasType(mgr, UnifiedSpiMem::DeviceType::NorW25Q)) || (hasType(mgr, UnifiedSpiMem::DeviceType::Psram) && hasType(mgr, UnifiedSpiMem::DeviceType::SpiNandMX35)) || (hasType(mgr, UnifiedSpiMem::DeviceType::NorW25Q) && hasType(mgr, UnifiedSpiMem::DeviceType::SpiNandMX35));
    bool ok = applicable ? testSimultaneousDifferentTypes(mgr) : false;
    logTest("Open two different device types simultaneously", applicable, ok);
  }

  // Test: pool open all (non-destructive tiny read on each)
  {
    bool applicable = (mgr.detectedCount() > 0);
    bool ok = applicable ? testPoolOpenAll(mgr) : false;
    logTest("DevicePool: open/close all devices", applicable, ok);
  }

  // ---- Summary ----
  size_t ran = 0, pass = 0;
  for (size_t i = 0; i < g_resultCount; ++i) {
    if (g_results[i].ran) {
      ran++;
      if (g_results[i].pass) pass++;
    }
  }
  Serial.println();
  Serial.print("Test summary: ");
  Serial.print(pass);
  Serial.print("/");
  Serial.print(ran);
  Serial.println(" passed.");
}

void loop() {
  // Nothing to do
  delay(1000);
}