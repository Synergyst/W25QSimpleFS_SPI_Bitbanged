/*
  main_psram_flash_switch_exec_loader_serial.ino
  DEFAULT (golden) PSRAM CS SCHEME: Single 74HC138 (MC74HC138AN), 4 PSRAM chips
*/
// ------- Shared HW SPI (FLASH + PSRAM) -------
#include "ConsolePrint.h"
#include <SPI.h>
// FLASH on HW-SPI via W25QBitbang's HW path
#define W25Q_USE_HW_SPI 1
#define W25Q_SPI_INSTANCE SPI1
#define W25Q_SPI_CLOCK_HZ 20000000UL
#include "W25QBitbang.h"
#include "W25QSimpleFS.h"
// PSRAM transport
#define USE_PSRAM_HW_SPI 1
#if USE_PSRAM_HW_SPI
#define PSRAM_SPI_INSTANCE SPI1
#include "SPIHWAdapter.h"
#define PSRAM_AGGREGATE_BUS SPIHWAdapter
#else
#include "PSRAMBitbang.h"
#endif
#include "PSRAMMulti.h"
// ------- Co-Processor over Software Serial (framed RPC) -------
#include <SoftwareSerial.h>
#include "CoProcProto.h"
#define COPROCLANG_MAX_LINES 512
#define COPROCLANG_MAX_LABELS 128
#include "CoProcLang.h"
#ifndef COPROC_BAUD
#define COPROC_BAUD 230400
#endif
#ifndef PIN_COPROC_RX
#define PIN_COPROC_RX 0  // GP0 (main RX)
#endif
#ifndef PIN_COPROC_TX
#define PIN_COPROC_TX 1  // GP1 (main TX)
#endif
static SoftwareSerial coprocLink(PIN_COPROC_RX, PIN_COPROC_TX, false);  // RX, TX, non-inverted
// Store ASCII scripts as raw multi-line strings, expose pointer + length.
#ifndef DECLARE_ASCII_SCRIPT
#define DECLARE_ASCII_SCRIPT(name, literal) \
  static const char name##_ascii[] = literal; \
  const uint8_t* name = reinterpret_cast<const uint8_t*>(name##_ascii); \
  const unsigned int name##_len = (unsigned int)(sizeof(name##_ascii) - 1)
#endif
#include "blob_mailbox_config.h"
#include "blob_ret42.h"
#define FILE_RET42 "ret42"
#include "blob_add2.h"
#define FILE_ADD2 "add2"
#include "blob_pwmc.h"
#define FILE_PWMC "pwmc"
#include "blob_pwmc2350.h"
#define FILE_PWMC2350 "pwmc2350"
#include "blob_retmin.h"
#define FILE_RETMIN "retmin"
#include "scripts.h"
#define FILE_BLINKSCRIPT "blinkscript"
#define FILE_ONSCRIPT "onscript"
struct BlobReg;               // Forward declaration
static ConsolePrint Console;  // Console wrapper
// ------- MC Compiiler -------
#include "MCCompiler.h"
// ------- Text Editor (legacy header; not used directly now) -------
#include "TextEditor.h"

// ========== Static buffer-related compile-time constant ==========
#define FS_SECTOR_SIZE 4096

// ========== bitbang pin definitions (flash) ==========
const uint8_t PIN_FLASH_MISO = 12;  // GP12
const uint8_t PIN_FLASH_CS = 28;    // GP28
const uint8_t PIN_FLASH_SCK = 10;   // GP10
const uint8_t PIN_FLASH_MOSI = 11;  // GP11

// ========== bitbang pin definitions (psram) ==========
const bool PSRAM_ENABLE_QPI = false;     // Should we enable the PSRAM QPI-mode?
const uint8_t PSRAM_CLOCK_DELAY_US = 0;  // Small delay helps ensure decoder/address settle before EN
const uint8_t PIN_PSRAM_MISO = 12;       // GP12
const uint8_t PIN_PSRAM_MOSI = 11;       // GP11
const uint8_t PIN_PSRAM_SCK = 10;        // GP10
// 74HC138 pins (for 138-only default)
const uint8_t PIN_138_A0 = 8;     // 74HC138 pin 1 (A0)
const uint8_t PIN_138_A1 = 7;     // 74HC138 pin 2 (A1)
const uint8_t PIN_138_EN = 9;     // 74HC138 pins 4+5 (G2A/G2B tied), 10k pull-up to 3.3V
const uint8_t PIN_138_A2 = 255;   // 255 means "not used"; tie 74HC138 pin 3 (A2) to GND
const uint8_t PIN_595_LATCH = 4;  // 595.RCLK (unused in 138-only mode)

// ========== Flash/PSRAM instances (needed before bindActiveFs) ==========
const size_t PERSIST_LEN = 32;
enum class StorageBackend {
  Flash,
  PSRAM_BACKEND
};
static StorageBackend g_storage = StorageBackend::Flash;
constexpr uint8_t PSRAM_NUM_CHIPS = 4;
constexpr uint32_t PER_CHIP_CAP_BYTES = 8UL * 1024UL * 1024UL;
constexpr uint32_t AGG_CAPACITY_BYTES = PER_CHIP_CAP_BYTES * PSRAM_NUM_CHIPS;
W25QBitbang flashBB(PIN_FLASH_MISO, PIN_FLASH_CS, PIN_FLASH_SCK, PIN_FLASH_MOSI);
W25QSimpleFS fsFlash(flashBB);
PSRAMAggregateDevice psramBB(PIN_PSRAM_MISO, PIN_PSRAM_MOSI, PIN_PSRAM_SCK, PER_CHIP_CAP_BYTES);
PSRAMSimpleFS_Multi fsPSRAM(psramBB, AGG_CAPACITY_BYTES);
static bool g_dev_mode = true;

// ========== ActiveFS structure ==========
struct ActiveFS {
  bool (*mount)(bool) = nullptr;
  bool (*format)() = nullptr;
  bool (*wipeChip)() = nullptr;
  bool (*exists)(const char*) = nullptr;
  bool (*createFileSlot)(const char*, uint32_t, const uint8_t*, uint32_t) = nullptr;
  bool (*writeFile)(const char*, const uint8_t*, uint32_t, int /*mode*/) = nullptr;
  bool (*writeFileInPlace)(const char*, const uint8_t*, uint32_t, bool) = nullptr;
  uint32_t (*readFile)(const char*, uint8_t*, uint32_t) = nullptr;
  uint32_t (*readFileRange)(const char*, uint32_t, uint8_t*, uint32_t) = nullptr;
  bool (*getFileSize)(const char*, uint32_t&) = nullptr;
  bool (*getFileInfo)(const char*, uint32_t&, uint32_t&, uint32_t&) = nullptr;
  bool (*deleteFile)(const char*) = nullptr;
  void (*listFilesToSerial)() = nullptr;
  uint32_t (*nextDataAddr)() = nullptr;
  uint32_t (*capacity)() = nullptr;
  uint32_t (*dataRegionStart)() = nullptr;
  static constexpr uint32_t SECTOR_SIZE = FS_SECTOR_SIZE;
  static constexpr uint32_t PAGE_SIZE = 256;
  static constexpr size_t MAX_NAME = 32;
} activeFs;

static void bindActiveFs(StorageBackend backend) {
  if (backend == StorageBackend::Flash) {
    activeFs.mount = [](bool b) {
      return fsFlash.mount(b);
    };
    activeFs.format = []() {
      return fsFlash.format();
    };
    activeFs.wipeChip = []() {
      return fsFlash.wipeChip();
    };
    activeFs.exists = [](const char* n) {
      return fsFlash.exists(n);
    };
    activeFs.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return fsFlash.createFileSlot(n, r, d, s);
    };
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsFlash.writeFile(n, d, s, static_cast<W25QSimpleFS::WriteMode>(m));
    };
    activeFs.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return fsFlash.writeFileInPlace(n, d, s, a);
    };
    activeFs.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return fsFlash.readFile(n, b, sz);
    };
    activeFs.readFileRange = [](const char* n, uint32_t off, uint8_t* b, uint32_t l) {
      return fsFlash.readFileRange(n, off, b, l);
    };
    activeFs.getFileSize = [](const char* n, uint32_t& s) {
      return fsFlash.getFileSize(n, s);
    };
    activeFs.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return fsFlash.getFileInfo(n, a, s, c);
    };
    activeFs.deleteFile = [](const char* n) {
      return fsFlash.deleteFile(n);
    };
    activeFs.listFilesToSerial = []() {
      fsFlash.listFilesToSerial();
    };
    activeFs.nextDataAddr = []() {
      return fsFlash.nextDataAddr();
    };
    activeFs.capacity = []() {
      return fsFlash.capacity();
    };
    activeFs.dataRegionStart = []() {
      return fsFlash.dataRegionStart();
    };
  } else {
    activeFs.mount = [](bool b) {
      return fsPSRAM.mount(b);
    };
    activeFs.format = []() {
      return fsPSRAM.format();
    };
    activeFs.wipeChip = []() {
      return fsPSRAM.wipeChip();
    };
    activeFs.exists = [](const char* n) {
      return fsPSRAM.exists(n);
    };
    activeFs.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return fsPSRAM.createFileSlot(n, r, d, s);
    };
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsPSRAM.writeFile(n, d, s, m);
    };
    activeFs.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return fsPSRAM.writeFileInPlace(n, d, s, a);
    };
    activeFs.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return fsPSRAM.readFile(n, b, sz);
    };
    activeFs.readFileRange = [](const char* n, uint32_t off, uint8_t* b, uint32_t l) {
      return fsPSRAM.readFileRange(n, off, b, l);
    };
    activeFs.getFileSize = [](const char* n, uint32_t& s) {
      return fsPSRAM.getFileSize(n, s);
    };
    activeFs.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return fsPSRAM.getFileInfo(n, a, s, c);
    };
    activeFs.deleteFile = [](const char* n) {
      return fsPSRAM.deleteFile(n);
    };
    activeFs.listFilesToSerial = []() {
      fsPSRAM.listFilesToSerial();
    };
    activeFs.nextDataAddr = []() {
      return fsPSRAM.nextDataAddr();
    };
    activeFs.capacity = []() {
      return fsPSRAM.capacity();
    };
    activeFs.dataRegionStart = []() {
      return fsPSRAM.dataRegionStart();
    };
  }
}

// ========== Blob registry ==========
struct BlobReg {
  const char* id;
  const uint8_t* data;
  unsigned int len;
};
static const BlobReg g_blobs[] = {
  { FILE_RET42, blob_ret42, blob_ret42_len },
  { FILE_ADD2, blob_add2, blob_add2_len },
  { FILE_PWMC, blob_pwmc, blob_pwmc_len },
  { FILE_PWMC2350, blob_pwmc, blob_pwmc2350_len },
  { FILE_RETMIN, blob_retmin, blob_retmin_len },
  { FILE_BLINKSCRIPT, blob_blinkscript, blob_blinkscript_len },
  { FILE_ONSCRIPT, blob_onscript, blob_onscript_len },
};
static const size_t g_blobs_count = sizeof(g_blobs) / sizeof(g_blobs[0]);

// ========== helpers using Console ==========
static void printHexByte(uint8_t b) {
  if (b < 0x10) Console.print('0');
  Console.print(b, HEX);
}

// ========== Return mailbox reservation (Scratch) ==========
extern "C" __scratch_x("blob_mailbox") __attribute__((aligned(4)))
int8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX] = { 0 };

// ================= New: ExecHost header =================
#include "ExecHost.h"
static ExecHost Exec;

// Helper to supply Exec with current FS function pointers
static void updateExecFsTable() {
  ExecFSTable t{};
  t.exists = activeFs.exists;
  t.getFileSize = activeFs.getFileSize;
  t.readFile = activeFs.readFile;
  t.readFileRange = activeFs.readFileRange;
  t.createFileSlot = activeFs.createFileSlot;
  t.writeFile = activeFs.writeFile;
  t.writeFileInPlace = activeFs.writeFileInPlace;
  t.getFileInfo = activeFs.getFileInfo;
  t.deleteFile = activeFs.deleteFile;
  Exec.attachFS(t);
}

// ========== FS helpers and console (unchanged) ==========
static bool checkNameLen(const char* name) {
  size_t n = strlen(name);
  if (n == 0 || n > ActiveFS::MAX_NAME) {
    Console.print("Error: filename length ");
    Console.print(n);
    Console.print(" exceeds max ");
    Console.print(ActiveFS::MAX_NAME);
    Console.println(". Use a shorter name.");
    return false;
  }
  return true;
}
static void listBlobs() {
  Console.println("Available blobs:");
  for (size_t i = 0; i < g_blobs_count; ++i) {
    Console.printf(" - %s  \t(%d bytes)\n", g_blobs[i].id, g_blobs[i].len);
  }
}
static const BlobReg* findBlob(const char* id) {
  for (size_t i = 0; i < g_blobs_count; ++i)
    if (strcmp(g_blobs[i].id, id) == 0) return &g_blobs[i];
  return nullptr;
}
static void dumpFileHead(const char* fname, uint32_t count) {
  uint32_t sz = 0;
  if (!activeFs.getFileSize(fname, sz) || sz == 0) {
    Console.println("dump: missing/empty");
    return;
  }
  if (count > sz) count = sz;
  const size_t CHUNK = 32;
  uint8_t buf[CHUNK];
  uint32_t off = 0;
  Console.print(fname);
  Console.print(" size=");
  Console.println(sz);
  while (off < count) {
    size_t n = (count - off > CHUNK) ? CHUNK : (count - off);
    uint32_t got = activeFs.readFileRange(fname, off, buf, n);
    if (got != n) {
      Console.println("  read error");
      break;
    }
    Console.print("  ");
    for (size_t i = 0; i < n; ++i) {
      if (i) Console.print(' ');
      printHexByte(buf[i]);
    }
    Console.println();
    off += n;
  }
}
static bool ensureBlobFile(const char* fname, const uint8_t* data, uint32_t len, uint32_t reserve = ActiveFS::SECTOR_SIZE) {
  if (!checkNameLen(fname)) return false;
  if (!activeFs.exists(fname)) {
    Console.print("Creating slot ");
    Console.print(fname);
    Console.print(" (");
    Console.print(reserve);
    Console.println(" bytes)...");
    if (activeFs.createFileSlot(fname, reserve, data, len)) {
      Console.println("Created and wrote blob");
      return true;
    }
    Console.println("Failed to create slot");
    return false;
  }
  uint32_t addr, size, cap;
  if (!activeFs.getFileInfo(fname, addr, size, cap)) {
    Console.println("getFileInfo failed");
    return false;
  }
  bool same = (size == len);
  if (same) {
    const size_t CHUNK = 64;
    uint8_t buf[CHUNK];
    uint32_t off = 0;
    while (off < size) {
      size_t n = (size - off > CHUNK) ? CHUNK : (size - off);
      activeFs.readFileRange(fname, off, buf, n);
      for (size_t i = 0; i < n; ++i) {
        if (buf[i] != data[off + i]) {
          same = false;
          break;
        }
      }
      if (!same) break;
      off += n;
      yield();
    }
  }
  if (same) {
    Console.println("Blob already up to date");
    return true;
  }
  if (cap >= len && activeFs.writeFileInPlace(fname, data, len, false)) {
    Console.println("Updated in place");
    return true;
  }
  if (activeFs.writeFile(fname, data, len, static_cast<int>(W25QSimpleFS::WriteMode::ReplaceIfExists))) {
    Console.println("Updated by allocating new space");
    return true;
  }
  Console.println("Failed to update file");
  return false;
}
static bool ensureBlobIfMissing(const char* fname, const uint8_t* data, uint32_t len, uint32_t reserve = ActiveFS::SECTOR_SIZE) {
  if (!checkNameLen(fname)) return false;
  if (activeFs.exists(fname)) {
    Console.printf("Skipping: %s\n", fname);
    return true;
  }
  Console.printf("Auto-creating %s (%d bytes)...\n", fname, reserve);
  if (activeFs.createFileSlot(fname, reserve, data, len)) {
    Console.println("Created and wrote blob");
    return true;
  }
  Console.println("Auto-create failed");
  return false;
}
static void autogenBlobWrites() {
  bool allOk = true;
  allOk &= ensureBlobIfMissing(FILE_RET42, blob_ret42, blob_ret42_len);
  allOk &= ensureBlobIfMissing(FILE_ADD2, blob_add2, blob_add2_len);
  allOk &= ensureBlobIfMissing(FILE_PWMC, blob_pwmc, blob_pwmc_len);
  allOk &= ensureBlobIfMissing(FILE_PWMC2350, blob_pwmc2350, blob_pwmc2350_len);
  allOk &= ensureBlobIfMissing(FILE_RETMIN, blob_retmin, blob_retmin_len);
  allOk &= ensureBlobIfMissing(FILE_BLINKSCRIPT, blob_blinkscript, blob_blinkscript_len);
  allOk &= ensureBlobIfMissing(FILE_ONSCRIPT, blob_onscript, blob_onscript_len);
  Console.print("Autogen:  ");
  Console.println(allOk ? "OK" : "some failures");
}

// ========== PSRAM smoke test (unchanged) ==========
static bool psramSafeSmokeTest() {
  if (g_storage != StorageBackend::PSRAM_BACKEND) {
    Console.println("psramSafeSmokeTest: active storage is not PSRAM — aborting");
    return false;
  }
  const uint32_t UPDATE_STEP_PERCENT = 5;
  const uint32_t MAX_SLOTS = 2044;
  const uint32_t PAGE = 256;
  if (UPDATE_STEP_PERCENT == 0 || (100 % UPDATE_STEP_PERCENT) != 0) {
    Console.println("psramSafeSmokeTest: UPDATE_STEP_PERCENT must divide 100 evenly");
    return false;
  }
  const uint8_t chips = psramBB.chipCount();
  if (chips == 0) {
    Console.println("\nPSRAM: no PSRAM chips detected");
    return false;
  }
  const uint32_t perChip = psramBB.perChipCapacity();
  const uint32_t totalCap = psramBB.capacity();
  const uint32_t SECT = ActiveFS::SECTOR_SIZE;
  uint32_t fsNext = 0, fsDataStart = 0;
  if (activeFs.nextDataAddr) fsNext = activeFs.nextDataAddr();
  if (activeFs.dataRegionStart) fsDataStart = activeFs.dataRegionStart();
  auto alignUp = [](uint32_t v, uint32_t a) -> uint32_t {
    return (v + (a - 1)) & ~(a - 1);
  };
  const uint32_t fsAligned = alignUp(fsNext, SECT);
  Console.printf("\nPSRAM multi-slot smoke: chips=%u per_chip=%u total=%u fsNext=%u fsAligned=%u\n",
                 (unsigned)chips, (unsigned)perChip, (unsigned)totalCap, (unsigned)fsNext, (unsigned)fsAligned);
  const uint32_t approxFree = (totalCap > fsAligned) ? (totalCap - fsAligned) : 0;
  Console.printf("Plan: allocate approx %u bytes of PSRAM free space\n", (unsigned)approxFree);
  if (approxFree == 0) {
    Console.println("PSRAM: no free space beyond FS; smoke test aborted");
    return false;
  }
  uint32_t stepsDesired = 100u / UPDATE_STEP_PERCENT;
  if (stepsDesired > MAX_SLOTS) {
    Console.printf("NOTE: stepsDesired=%u > MAX_SLOTS=%u, capping steps to %u\n",
                   (unsigned)stepsDesired, (unsigned)MAX_SLOTS, (unsigned)MAX_SLOTS);
    stepsDesired = MAX_SLOTS;
  }
  uint32_t numSlots = stepsDesired;
  uint32_t baseReserve = (approxFree + numSlots - 1) / numSlots;
  baseReserve = ((baseReserve + (SECT - 1)) & ~(SECT - 1));
  while ((uint64_t)baseReserve * numSlots > approxFree && numSlots > 1) {
    --numSlots;
    baseReserve = (approxFree + numSlots - 1) / numSlots;
    baseReserve = ((baseReserve + (SECT - 1)) & ~(SECT - 1));
  }
  if ((uint64_t)baseReserve * numSlots > approxFree) {
    Console.printf("Not enough free space for %u slots (need %u bytes, have %u). Aborting.\n",
                   (unsigned)numSlots, (unsigned)((uint64_t)baseReserve * numSlots), (unsigned)approxFree);
    return false;
  }
  Console.printf("Allocating %u slots of ~%u bytes each (aligned to %u)\n", (unsigned)numSlots, (unsigned)baseReserve, (unsigned)SECT);
  char slotName[32];
  uint32_t allocatedBytes = 0;
  uint32_t percentNextAlloc = UPDATE_STEP_PERCENT;
  uint32_t createdSlots = 0;
  Console.println("Stage: Allocate slots");
  for (uint32_t i = 0; i < numSlots; ++i) {
    snprintf(slotName, sizeof(slotName), ".span_part_%03u", (unsigned)i);
    if (activeFs.exists && activeFs.exists(slotName)) {
      if (activeFs.deleteFile) activeFs.deleteFile(slotName);
    }
    uint32_t reserve = baseReserve;
    uint32_t remainNeeded = (allocatedBytes >= approxFree) ? 0 : (approxFree - allocatedBytes);
    if (reserve > remainNeeded && i == numSlots - 1) {
      reserve = ((remainNeeded ? remainNeeded : SECT) + (SECT - 1)) & ~(SECT - 1);
    }
    bool ok = activeFs.createFileSlot(slotName, reserve, nullptr, 0);
    if (!ok) {
      Console.printf("\ncreateFileSlot failed for %s (reserve=%u). Aborting.\n", slotName, (unsigned)reserve);
      return false;
    }
    allocatedBytes += reserve;
    ++createdSlots;
    uint32_t allocPct = (uint32_t)(((uint64_t)allocatedBytes * 100u) / (uint64_t)approxFree);
    if (allocPct > 100) allocPct = 100;
    while (allocPct >= percentNextAlloc && percentNextAlloc <= 100) {
      Console.printf("Alloc progress: %u%%\n", (unsigned)percentNextAlloc);
      percentNextAlloc += UPDATE_STEP_PERCENT;
    }
    if ((i & 7) == 0) yield();
  }
  if (percentNextAlloc <= 100) {
    while (percentNextAlloc <= 100) {
      Console.printf("Alloc progress: %u%%\n", (unsigned)percentNextAlloc);
      percentNextAlloc += UPDATE_STEP_PERCENT;
    }
  }
  Console.println("Allocation complete");
  uint8_t page[PAGE];
  uint8_t readbuf[PAGE];
  uint64_t totalToWrite = approxFree;
  uint64_t totalWritten = 0;
  uint32_t globalNextPct = UPDATE_STEP_PERCENT;
  Console.println("Stage: Writing data into slots");
  for (uint32_t i = 0; i < createdSlots; ++i) {
    snprintf(slotName, sizeof(slotName), ".span_part_%03u", (unsigned)i);
    uint32_t addr = 0, sz = 0, cap = 0;
    if (!activeFs.getFileInfo(slotName, addr, sz, cap)) {
      Console.printf("getFileInfo failed for slot %s\n", slotName);
      return false;
    }
    uint32_t toWrite = cap;
    if (toWrite > approxFree - totalWritten) toWrite = (uint32_t)(approxFree - totalWritten);
    if (toWrite == 0) {
      totalWritten = approxFree;
      break;
    }
    Console.printf("Slot %u: addr=0x%08X cap=%u write=%u\n", (unsigned)i, (unsigned)addr, (unsigned)cap, (unsigned)toWrite);
    uint32_t slotWritten = 0;
    while (slotWritten < toWrite) {
      uint32_t n = (toWrite - slotWritten) >= PAGE ? PAGE : (toWrite - slotWritten);
      for (uint32_t k = 0; k < n; ++k) {
        page[k] = (uint8_t)(0xA5 ^ (uint8_t)i ^ (uint8_t)((slotWritten + k) & 0xFF));
      }
      if (!psramBB.writeData02(addr + slotWritten, page, n)) {
        Console.printf("  write failed for slot %u at offset %u\n", (unsigned)i, (unsigned)slotWritten);
        return false;
      }
      memset(readbuf, 0, sizeof(readbuf));
      if (!psramBB.readData03(addr + slotWritten, readbuf, n)) {
        Console.printf("  readback failed for slot %u at offset %u\n", (unsigned)i, (unsigned)slotWritten);
        return false;
      }
      if (memcmp(readbuf, page, n) != 0) {
        Console.printf("  verify mismatch slot %u offset %u\n", (unsigned)i, (unsigned)slotWritten);
        return false;
      }
      slotWritten += n;
      totalWritten += n;
      uint32_t slotPct = (uint32_t)(((uint64_t)slotWritten * 100u) / (uint64_t)toWrite);
      if (slotPct > 100) slotPct = 100;
      uint32_t globalPct = (uint32_t)(((uint64_t)totalWritten * 100u) / (uint64_t)totalToWrite);
      if (globalPct > 100) globalPct = 100;
      Console.printf("Global: %u%%, Slot %03u: %u%%, offset=%u, written=%u\n",
                     (unsigned)globalPct, (unsigned)i, (unsigned)slotPct, (unsigned)slotWritten, (unsigned)slotWritten);
      if ((slotWritten % SECT) == 0) yield();
    }
    Console.printf("Slot %03u complete: written=%u\n", (unsigned)i, (unsigned)toWrite);
  }
  while (globalNextPct <= 100) {
    Console.printf("Global: %u%%\n", (unsigned)globalNextPct);
    globalNextPct += UPDATE_STEP_PERCENT;
  }
  Console.println("PSRAM multi-slot smoke test complete.");
  Console.printf("Total bytes written (approx): %llu\n", (unsigned long long)totalWritten);
  return true;
}

// ========== Binary upload helpers (single-line puthex/putb64) ==========
static inline int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
static bool decodeHexString(const char* hex, uint8_t*& out, uint32_t& outLen) {
  out = nullptr;
  outLen = 0;
  if (!hex) return false;
  size_t n = strlen(hex);
  if (n == 0) return false;
  if (n & 1) {
    Console.println("puthex: error: hex string length is odd");
    return false;
  }
  uint32_t bytes = (uint32_t)(n / 2);
  uint8_t* buf = (uint8_t*)malloc(bytes);
  if (!buf) {
    Console.println("puthex: malloc failed");
    return false;
  }
  for (uint32_t i = 0; i < bytes; ++i) {
    int hi = hexVal(hex[2 * i + 0]), lo = hexVal(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) {
      Console.println("puthex: invalid hex character");
      free(buf);
      return false;
    }
    buf[i] = (uint8_t)((hi << 4) | lo);
  }
  out = buf;
  outLen = bytes;
  return true;
}
static int8_t b64Map[256];
static void initB64MapOnce() {
  static bool inited = false;
  if (inited) return;
  for (int i = 0; i < 256; ++i) b64Map[i] = -1;
  for (char c = 'A'; c <= 'Z'; ++c) b64Map[(uint8_t)c] = (int8_t)(c - 'A');
  for (char c = 'a'; c <= 'z'; c++) b64Map[(uint8_t)c] = (int8_t)(26 + (c - 'a'));
  for (char c = '0'; c <= '9'; c++) b64Map[(uint8_t)c] = (int8_t)(52 + (c - '0'));
  b64Map[(uint8_t)'+'] = 62;
  b64Map[(uint8_t)'/'] = 63;
}
static bool decodeBase64String(const char* s, uint8_t*& out, uint32_t& outLen) {
  out = nullptr;
  outLen = 0;
  if (!s) return false;
  initB64MapOnce();
  size_t inLen = strlen(s);
  uint32_t outCap = (uint32_t)(((inLen + 3) / 4) * 3);
  uint8_t* buf = (uint8_t*)malloc(outCap ? outCap : 1);
  if (!buf) {
    Console.println("putb64: malloc failed");
    return false;
  }
  uint32_t o = 0;
  int vals[4];
  int vCount = 0;
  int pad = 0;
  for (size_t i = 0; i < inLen; ++i) {
    unsigned char c = (unsigned char)s[i];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
    if (c == '=') {
      vals[vCount++] = 0;
      pad++;
    } else {
      int8_t v = b64Map[c];
      if (v < 0) {
        Console.println("putb64: invalid base64 character");
        free(buf);
        return false;
      }
      vals[vCount++] = v;
    }
    if (vCount == 4) {
      uint32_t v0 = (uint32_t)vals[0], v1 = (uint32_t)vals[1], v2 = (uint32_t)vals[2], v3 = (uint32_t)vals[3];
      uint8_t b0 = (uint8_t)((v0 << 2) | (v1 >> 4));
      uint8_t b1 = (uint8_t)(((v1 & 0x0F) << 4) | (v2 >> 2));
      uint8_t b2 = (uint8_t)(((v2 & 0x03) << 6) | v3);
      if (pad == 0) {
        buf[o++] = b0;
        buf[o++] = b1;
        buf[o++] = b2;
      } else if (pad == 1) {
        buf[o++] = b0;
        buf[o++] = b1;
      } else if (pad == 2) {
        buf[o++] = b0;
      } else {
        free(buf);
        Console.println("putb64: invalid padding");
        return false;
      }
      vCount = 0;
      pad = 0;
    }
  }
  if (vCount != 0) {
    free(buf);
    Console.println("putb64: truncated input");
    return false;
  }
  out = buf;
  outLen = o;
  return true;
}
static bool writeBinaryToFS(const char* fname, const uint8_t* data, uint32_t len) {
  if (!checkNameLen(fname)) return false;
  bool ok = activeFs.writeFile(fname, data, len, static_cast<int>(W25QSimpleFS::WriteMode::ReplaceIfExists));
  return ok;
}

// ========== New: compile Tiny-C source file -> raw Thumb binary on FS ==========
static void printCompileErrorContext(const char* src, size_t srcLen, size_t pos) {
  const size_t CONTEXT = 40;
  if (!src || srcLen == 0) return;
  size_t start = (pos > CONTEXT) ? (pos - CONTEXT) : 0;
  size_t end = (pos + CONTEXT < srcLen) ? (pos + CONTEXT) : srcLen;
  Console.println("----- context -----");
  for (size_t i = start; i < end; ++i) {
    char c = src[i];
    if (c == '\r') c = ' ';
    Console.print(c);
  }
  Console.println();
  size_t caret = pos - start;
  for (size_t i = 0; i < caret; ++i) Console.print(' ');
  Console.println("^");
  Console.println("-------------------");
}
static bool compileTinyCFileToFile(const char* srcName, const char* dstName) {
  if (!checkNameLen(srcName) || !checkNameLen(dstName)) return false;
  if (!activeFs.exists(srcName)) {
    Console.print("compile: source not found: ");
    Console.println(srcName);
    return false;
  }
  uint32_t srcSize = 0;
  if (!activeFs.getFileSize(srcName, srcSize) || srcSize == 0) {
    Console.println("compile: getFileSize failed or empty source");
    return false;
  }
  // Read source
  char* srcBuf = (char*)malloc(srcSize + 1);
  if (!srcBuf) {
    Console.println("compile: malloc src failed");
    return false;
  }
  uint32_t got = activeFs.readFile(srcName, (uint8_t*)srcBuf, srcSize);
  if (got != srcSize) {
    Console.println("compile: readFile failed");
    free(srcBuf);
    return false;
  }
  srcBuf[srcSize] = 0;
  // Prepare output buffer
  uint32_t outCap = (srcSize * 12u) + 256u;
  if (outCap < 512u) outCap = 512u;
  uint8_t* outBuf = (uint8_t*)malloc(outCap);
  if (!outBuf) {
    Console.println("compile: malloc out failed");
    free(srcBuf);
    return false;
  }
  MCCompiler comp;
  size_t outSize = 0;
  MCCompiler::Result r = comp.compile(srcBuf, srcSize, outBuf, outCap, &outSize);
  if (!r.ok) {
    Console.print("compile: error at pos ");
    Console.print((uint32_t)r.errorPos);
    Console.print(": ");
    Console.println(r.errorMsg ? r.errorMsg : "unknown");
    printCompileErrorContext(srcBuf, srcSize, r.errorPos);
    free(outBuf);
    free(srcBuf);
    return false;
  }
  // Write to destination file
  bool ok = writeBinaryToFS(dstName, outBuf, (uint32_t)outSize);
  if (ok) {
    Console.print("compile: OK -> ");
    Console.print(dstName);
    Console.print(" (");
    Console.print((uint32_t)outSize);
    Console.println(" bytes)");
    if (outSize & 1u) {
      Console.println("note: odd-sized output; for Thumb execution, even size is recommended.");
    }
  } else {
    Console.println("compile: write failed");
  }
  free(outBuf);
  free(srcBuf);
  return ok;
}

// ===================== Co-Processor RPC helpers (for CMD_FUNC) =====================
static uint32_t g_coproc_seq = 1;
static bool coprocWriteAll(const uint8_t* src, size_t n, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t off = 0;
  while (off < n) {
    size_t w = coprocLink.write(src + off, n - off);
    if (w > 0) {
      off += w;
      continue;
    }
    if ((millis() - start) > timeoutMs) return false;
    yield();
  }
  coprocLink.flush();
  return true;
}
static bool coprocReadByte(uint8_t& b, uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) <= timeoutMs) {
    int a = coprocLink.available();
    if (a > 0) {
      int v = coprocLink.read();
      if (v >= 0) {
        b = (uint8_t)v;
        return true;
      }
    }
    tight_loop_contents();
    yield();
  }
  return false;
}
static bool coprocReadExact(uint8_t* dst, size_t n, uint32_t timeoutPerByteMs) {
  for (size_t i = 0; i < n; ++i) {
    if (!coprocReadByte(dst[i], timeoutPerByteMs)) return false;
  }
  return true;
}
// Send CMD_FUNC with classic int32 argv[argc]. On success returns true and sets outResult.
static bool coprocCallFunc(const char* name, const int32_t* argv, uint32_t argc, int32_t& outResult) {
  if (!name) return false;
  uint32_t nameLen = (uint32_t)strlen(name);
  if (nameLen == 0 || nameLen > 128) {
    Console.println("coproc func: name length invalid (1..128)");
    return false;
  }
  if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
  // Build payload: [uint32 nameLen][bytes name][uint32 argc][int32 argv[argc]]
  const uint32_t payloadLen = 4 + nameLen + 4 + 4 * argc;
  uint8_t payload[4 + 128 + 4 + 4 * MAX_EXEC_ARGS];
  size_t off = 0;
  CoProc::writePOD(payload, sizeof(payload), off, nameLen);
  CoProc::writeBytes(payload, sizeof(payload), off, name, nameLen);
  CoProc::writePOD(payload, sizeof(payload), off, argc);
  for (uint32_t i = 0; i < argc; ++i) {
    CoProc::writePOD(payload, sizeof(payload), off, argv[i]);
  }
  // Build request header
  CoProc::Frame req{};
  req.magic = CoProc::MAGIC;
  req.version = CoProc::VERSION;
  req.cmd = CoProc::CMD_FUNC;
  req.seq = g_coproc_seq++;
  req.len = payloadLen;
  req.crc32 = (payloadLen ? CoProc::crc32_ieee(payload, payloadLen) : 0);
  // Send request
  if (!coprocWriteAll(reinterpret_cast<const uint8_t*>(&req), sizeof(req), 2000)) {
    Console.println("coproc func: write header failed");
    return false;
  }
  if (payloadLen) {
    if (!coprocWriteAll(payload, payloadLen, 10000)) {
      Console.println("coproc func: write payload failed");
      return false;
    }
  }
  // Read response: scan for magic
  const uint8_t magicBytes[4] = { (uint8_t)('C'), (uint8_t)('P'), (uint8_t)('R'), (uint8_t)('0') };
  uint8_t w[4] = { 0, 0, 0, 0 };
  for (;;) {
    uint8_t b = 0;
    if (!coprocReadByte(b, 5000)) {
      Console.println("coproc func: response timeout");
      return false;
    }
    w[0] = w[1];
    w[1] = w[2];
    w[2] = w[3];
    w[3] = b;
    if (w[0] == magicBytes[0] && w[1] == magicBytes[1] && w[2] == magicBytes[2] && w[3] == magicBytes[3]) {
      // We have magic, read rest of header
      union {
        CoProc::Frame f;
        uint8_t bytes[sizeof(CoProc::Frame)];
      } u;
      memcpy(u.bytes, w, 4);
      if (!coprocReadExact(u.bytes + 4, sizeof(CoProc::Frame) - 4, 200)) {
        Console.println("coproc func: resp header tail timeout");
        return false;
      }
      CoProc::Frame resp = u.f;
      if ((resp.magic != CoProc::MAGIC) || (resp.version != CoProc::VERSION)) {
        Console.println("coproc func: resp bad magic/version");
        return false;
      }
      if (resp.cmd != (uint16_t)(CoProc::CMD_FUNC | 0x80)) {
        Console.print("coproc func: unexpected resp cmd=0x");
        Console.println(resp.cmd, HEX);
        return false;
      }
      // Read payload if any
      uint8_t buf[16];
      if (resp.len > sizeof(buf)) {
        Console.println("coproc func: resp too large");
        uint32_t left = resp.len;
        uint8_t sink[32];
        while (left) {
          uint32_t chunk = (left > sizeof(sink)) ? sizeof(sink) : left;
          if (!coprocReadExact(sink, chunk, 200)) break;
          left -= chunk;
        }
        return false;
      }
      if (resp.len) {
        if (!coprocReadExact(buf, resp.len, 200)) {
          Console.println("coproc func: resp payload timeout");
          return false;
        }
        uint32_t crc = CoProc::crc32_ieee(buf, resp.len);
        if (crc != resp.crc32) {
          Console.print("coproc func: resp CRC mismatch exp=0x");
          Console.print(resp.crc32, HEX);
          Console.print(" got=0x");
          Console.println(crc, HEX);
          return false;
        }
      }
      // Parse response: int32 status, int32 result
      size_t rp = 0;
      int32_t st = CoProc::ST_BAD_CMD;
      int32_t rv = 0;
      if (resp.len >= sizeof(int32_t)) {
        CoProc::readPOD(buf, resp.len, rp, st);
      }
      if (resp.len >= 2 * sizeof(int32_t)) {
        CoProc::readPOD(buf, resp.len, rp, rv);
      }
      if (st == CoProc::ST_OK) {
        outResult = rv;
        return true;
      } else {
        Console.print("coproc func: remote error status=");
        Console.println(st);
        return false;
      }
    }
  }
}

// ========== Serial console / command handling ==========

// Cooked console line input with echo, backspace handling, and CR/LF normalization.
// This fixes lack of echo in terminal emulators and supports Windows CRLF.
static char lineBuf[FS_SECTOR_SIZE];
static bool readLine() {
  static size_t pos = 0;
  while (Serial.available()) {
    int ic = Serial.read();
    if (ic < 0) break;
    char c = (char)ic;

    // Handle Windows CRLF and Unix LF
    if (c == '\r' || c == '\n') {
      // Swallow paired LF after CR for Windows
      if (c == '\r') {
        if (Serial.available()) {
          int p = Serial.peek();
          if (p == '\n') Serial.read();
        }
      }
      // Echo newline to terminal
      Serial.write('\r');
      Serial.write('\n');
      lineBuf[pos] = 0;
      pos = 0;
      return true;
    }

    // Backspace / Delete
    if ((c == 0x08) || (c == 0x7F)) {
      if (pos > 0) {
        pos--;
        // Erase char visually: backspace, space, backspace
        Serial.write('\b');
        Serial.write(' ');
        Serial.write('\b');
      }
      continue;
    }

    // Printable ASCII
    if (c >= 32 && c <= 126) {
      if (pos + 1 < sizeof(lineBuf)) {
        lineBuf[pos++] = c;
        Serial.write(c);  // echo
      } else {
        // Optional: bell on overflow
        Serial.write('\a');
      }
      continue;
    }

    // Ignore other control bytes
  }
  return false;
}

static int nextToken(char*& p, char*& tok) {
  while (*p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
  if (!*p) {
    tok = nullptr;
    return 0;
  }
  tok = p;
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  if (*p) {
    *p = 0;
    ++p;
  }
  return 1;
}

// ----------------- Nano-like editor integration -----------------
namespace Term {
static inline void write(const char* s) {
  if (s) Serial.print(s);
}
static inline void writeChar(char c) {
  Serial.write((uint8_t)c);
}
static inline void clear() {
  write("\x1b[2J\x1b[H");
}
static inline void hideCursor() {
  write("\x1b[?25l");
}
static inline void showCursor() {
  write("\x1b[?25h");
}
static inline void move(int row1, int col1) {
  char buf[24];
  snprintf(buf, sizeof(buf), "\x1b[%d;%dH", row1, col1);
  write(buf);
}
static inline void clrEol() {
  write("\x1b[K");
}
static inline void invertOn() {
  write("\x1b[7m");
}
static inline void invertOff() {
  write("\x1b[0m");
}
}

struct NanoEditor {
  static const int MAX_LINES = 512;
  static const int MAX_COLS = 256;    // allow longer lines than console view
  static const int SCREEN_ROWS = 24;  // assume 80x24 terminal
  static const int SCREEN_COLS = 80;

  char lines[MAX_LINES][MAX_COLS];
  int lineCount = 0;
  int cx = 0;       // cursor x in current line (column, 0-based)
  int cy = 0;       // cursor y (line index, 0-based)
  int rowOff = 0;   // topmost visible line
  int colOff = 0;   // leftmost visible column
  int prefCol = 0;  // preferred column for up/down
  bool modified = false;
  char filename[ActiveFS::MAX_NAME + 1];

  // Load file (UTF-8 treated as bytes)
  bool load(const char* path) {
    filename[0] = 0;
    if (path) {
      strncpy(filename, path, sizeof(filename) - 1);
      filename[sizeof(filename) - 1] = 0;
    }
    lineCount = 0;
    if (!path || !activeFs.exists(path)) {
      lineCount = 1;
      lines[0][0] = 0;
      cx = cy = rowOff = colOff = prefCol = 0;
      modified = false;
      return true;
    }
    uint32_t sz = 0;
    if (!activeFs.getFileSize(path, sz)) return false;
    char* tmp = (char*)malloc(sz + 1);
    if (!tmp) return false;
    uint32_t got = activeFs.readFile(path, (uint8_t*)tmp, sz);
    if (got != sz) {
      free(tmp);
      return false;
    }
    tmp[sz] = 0;
    char* p = tmp;
    while (*p && lineCount < MAX_LINES) {
      char* e = p;
      while (*e && *e != '\n' && *e != '\r') ++e;
      int L = (int)(e - p);
      if (L >= MAX_COLS) L = MAX_COLS - 1;
      memcpy(lines[lineCount], p, L);
      lines[lineCount][L] = 0;
      lineCount++;
      while (*e == '\n' || *e == '\r') ++e;
      p = e;
    }
    if (lineCount == 0) {
      lineCount = 1;
      lines[0][0] = 0;
    }
    free(tmp);
    cx = cy = rowOff = colOff = prefCol = 0;
    modified = false;
    return true;
  }

  // Save to ActiveFS
  bool saveAs(const char* path) {
    if (!path || !checkNameLen(path)) return false;
    // join lines
    uint32_t total = 0;
    for (int i = 0; i < lineCount; ++i) total += (uint32_t)strlen(lines[i]) + 1;
    uint8_t* out = (uint8_t*)malloc(total ? total : 1);
    if (!out) return false;
    uint32_t off = 0;
    for (int i = 0; i < lineCount; ++i) {
      size_t L = strlen(lines[i]);
      memcpy(out + off, lines[i], L);
      off += (uint32_t)L;
      out[off++] = '\n';
    }
    bool ok = activeFs.writeFile(path, out, off, static_cast<int>(W25QSimpleFS::WriteMode::ReplaceIfExists));
    free(out);
    if (ok) {
      strncpy(filename, path, sizeof(filename) - 1);
      filename[sizeof(filename) - 1] = 0;
      modified = false;
    }
    return ok;
  }

  // UI
  void drawStatus(const char* msg) {
    Term::move(SCREEN_ROWS - 1, 1);
    Term::invertOn();
    // status line
    char status[128];
    snprintf(status, sizeof(status), " %s %s  Ln %d, Col %d  %s",
             "Nano-like Editor",
             filename[0] ? filename : "[No Name]",
             cy + 1, cx + 1,
             modified ? "(modified)" : "");
    Serial.print(status);
    Term::clrEol();
    Term::invertOff();
    // help bar
    Term::move(SCREEN_ROWS, 1);
    Serial.print("^X Exit  ^C Pos  Arrows Move  Backspace/Delete Edit  Enter Newline");
    Term::clrEol();
    // message (overlay above help bar if provided)
    if (msg && *msg) {
      Term::move(SCREEN_ROWS - 2, 1);
      Serial.print(msg);
      Term::clrEol();
    } else {
      Term::move(SCREEN_ROWS - 2, 1);
      Term::clrEol();
    }
  }
  void drawRows() {
    // Draw from row 1 to SCREEN_ROWS-2 (reserve bottom 2 rows for bars)
    int usable = SCREEN_ROWS - 2;
    for (int i = 0; i < usable; ++i) {
      int fileRow = rowOff + i;
      Term::move(1 + i, 1);
      Term::clrEol();
      if (fileRow >= lineCount) {
        Serial.print("~");
      } else {
        const char* s = lines[fileRow];
        int len = (int)strlen(s);
        if (len > colOff) {
          const char* p = s + colOff;
          int toShow = len - colOff;
          if (toShow < 0) toShow = 0;
          if (toShow > SCREEN_COLS) toShow = SCREEN_COLS;
          Serial.write((const uint8_t*)p, toShow);
        }
      }
    }
  }
  void refresh(const char* msg = nullptr) {
    Term::hideCursor();
    Term::move(1, 1);
    drawRows();
    drawStatus(msg);
    // place cursor
    int crow = cy - rowOff + 1;
    int ccol = cx - colOff + 1;
    if (crow < 1) crow = 1;
    if (ccol < 1) ccol = 1;
    if (crow > SCREEN_ROWS - 2) crow = SCREEN_ROWS - 2;
    if (ccol > SCREEN_COLS) ccol = SCREEN_COLS;
    Term::move(crow, ccol);
    Term::showCursor();
  }

  // Editing actions
  void moveLeft() {
    if (cx > 0) {
      cx--;
      prefCol = cx;
      return;
    }
    if (cy > 0) {
      cy--;
      cx = (int)strlen(lines[cy]);
      prefCol = cx;
      if (rowOff > cy) rowOff = cy;
      ensureCursorVisible();
    }
  }
  void moveRight() {
    int L = (int)strlen(lines[cy]);
    if (cx < L) {
      cx++;
      prefCol = cx;
      ensureCursorVisible();
      return;
    }
    if (cy + 1 < lineCount) {
      cy++;
      cx = 0;
      prefCol = cx;
      ensureCursorVisible();
    }
  }
  void moveUp() {
    if (cy > 0) cy--;
    int L = (int)strlen(lines[cy]);
    if (prefCol > L) cx = L;
    else cx = prefCol;
    ensureCursorVisible();
  }
  void moveDown() {
    if (cy + 1 < lineCount) cy++;
    int L = (int)strlen(lines[cy]);
    if (prefCol > L) cx = L;
    else cx = prefCol;
    ensureCursorVisible();
  }
  void ensureCursorVisible() {
    // Horizontal
    if (cx < colOff) colOff = cx;
    if (cx >= colOff + SCREEN_COLS) colOff = cx - SCREEN_COLS + 1;
    // Vertical
    if (cy < rowOff) rowOff = cy;
    if (cy >= rowOff + (SCREEN_ROWS - 2)) rowOff = cy - (SCREEN_ROWS - 2) + 1;
  }
  void insertChar(char c) {
    char* line = lines[cy];
    int L = (int)strlen(line);
    if (L >= MAX_COLS - 1) return;  // full, ignore
    if (cx > L) cx = L;
    // shift right
    for (int i = L; i >= cx; --i) line[i + 1] = line[i];
    line[cx] = c;
    cx++;
    prefCol = cx;
    modified = true;
    ensureCursorVisible();
  }
  void backspace() {
    if (cx > 0) {
      char* line = lines[cy];
      int L = (int)strlen(line);
      for (int i = cx - 1; i < L; ++i) line[i] = line[i + 1];
      cx--;
      prefCol = cx;
      modified = true;
      return;
    }
    // at start of line: join with previous if any
    if (cy > 0) {
      int prevL = (int)strlen(lines[cy - 1]);
      int curL = (int)strlen(lines[cy]);
      int canCopy = min(MAX_COLS - 1 - prevL, curL);
      // append as much as fits
      memcpy(lines[cy - 1] + prevL, lines[cy], canCopy);
      lines[cy - 1][prevL + canCopy] = 0;
      // remove this line
      for (int i = cy; i < lineCount - 1; ++i) {
        strcpy(lines[i], lines[i + 1]);
      }
      lineCount--;
      cy--;
      cx = prevL;
      prefCol = cx;
      modified = true;
      ensureCursorVisible();
    }
  }
  void delChar() {
    char* line = lines[cy];
    int L = (int)strlen(line);
    if (cx < L) {
      for (int i = cx; i < L; ++i) line[i] = line[i + 1];
      modified = true;
      return;
    }
    // at end of line: join with next line
    if (cy + 1 < lineCount) {
      int curL = (int)strlen(lines[cy]);
      int nextL = (int)strlen(lines[cy + 1]);
      int canCopy = min(MAX_COLS - 1 - curL, nextL);
      memcpy(lines[cy] + curL, lines[cy + 1], canCopy);
      lines[cy][curL + canCopy] = 0;
      // shift lines up
      for (int i = cy + 1; i < lineCount - 1; ++i) {
        strcpy(lines[i], lines[i + 1]);
      }
      lineCount--;
      modified = true;
    }
  }
  void newline() {
    if (lineCount >= MAX_LINES) return;
    char* line = lines[cy];
    int L = (int)strlen(line);
    // split at cx
    char tail[MAX_COLS];
    int tailLen = (cx < L) ? (L - cx) : 0;
    if (tailLen > 0) {
      memcpy(tail, line + cx, tailLen);
    }
    tail[tailLen] = 0;
    line[cx] = 0;  // truncate current
    // shift lines down to insert
    for (int i = lineCount; i > cy + 1; --i) {
      strcpy(lines[i], lines[i - 1]);
    }
    strcpy(lines[cy + 1], tail);
    lineCount++;
    cy++;
    cx = 0;
    prefCol = 0;
    modified = true;
    ensureCursorVisible();
  }

  // read a key, parsing ESC sequences for arrows and delete
  int readKey() {
    for (;;) {
      if (Serial.available() > 0) {
        int ch = Serial.read();
        if (ch == '\r') {
          // swallow optional '\n'
          if (Serial.available() && Serial.peek() == '\n') Serial.read();
          return '\n';
        }
        if (ch == 0x1B) {  // ESC
          // parse CSI
          while (!Serial.available()) {
            Exec.pollBackground();
            tight_loop_contents();
            yield();
          }
          int ch1 = Serial.read();
          if (ch1 == '[') {
            while (!Serial.available()) {
              Exec.pollBackground();
              tight_loop_contents();
              yield();
            }
            int ch2 = Serial.read();
            if (ch2 == 'A') return 1000;  // up
            if (ch2 == 'B') return 1001;  // down
            if (ch2 == 'C') return 1002;  // right
            if (ch2 == 'D') return 1003;  // left
            if (ch2 == '3') {
              // expect ~
              while (!Serial.available()) {
                Exec.pollBackground();
                tight_loop_contents();
                yield();
              }
              int tilde = Serial.read();
              if (tilde == '~') return 1004;  // delete
            }
            // ignore other CSI
            return -1;
          } else {
            // lone ESC or other sequences: ignore
            return -1;
          }
        }
        return ch;
      }
      Exec.pollBackground();
      tight_loop_contents();
      yield();
    }
  }

  // prompt Yes/No, returns 1 for yes, 0 for no, -1 for cancel (Esc)
  int promptYesNo(const char* q) {
    refresh(q);
    for (;;) {
      int k = readKey();
      if (k == 'y' || k == 'Y') return 1;
      if (k == 'n' || k == 'N') return 0;
      if (k == 27) return -1;
    }
  }
  // prompt for filename (default shown), return true if got a name in out
  bool promptFilename(char* out, size_t outCap) {
    char buf[ActiveFS::MAX_NAME + 1];
    buf[0] = 0;
    if (filename[0]) strncpy(buf, filename, sizeof(buf) - 1);
    size_t len = strlen(buf);
    char msg[96];
    snprintf(msg, sizeof(msg), "File Name to Write: %s", buf);
    refresh(msg);
    for (;;) {
      int k = readKey();
      if (k == '\n') {
        if (len == 0) return false;
        strncpy(out, buf, outCap - 1);
        out[outCap - 1] = 0;
        return true;
      } else if (k == 27) {
        return false;
      } else if (k == 0x08 || k == 0x7F) {
        if (len > 0) {
          buf[--len] = 0;
        }
      } else if (k >= 32 && k <= 126) {
        if (len + 1 < sizeof(buf)) {
          buf[len++] = (char)k;
          buf[len] = 0;
        }
      }
      snprintf(msg, sizeof(msg), "File Name to Write: %s", buf);
      refresh(msg);
    }
  }

  bool run() {
    Term::clear();
    refresh("Welcome to Nano-like editor. ^X exit, ^C cursor pos");
    for (;;) {
      int k = readKey();
      if (k == 3) {  // Ctrl-C -> show pos
        char m[64];
        snprintf(m, sizeof(m), "Cursor position: Ln %d, Col %d", cy + 1, cx + 1);
        refresh(m);
        continue;
      }
      if (k == 24) {  // Ctrl-X -> prompt save then exit
        int ans = modified ? promptYesNo("Save modified buffer? (Y/N)") : 0;
        if (ans == 1) {
          char outname[ActiveFS::MAX_NAME + 1];
          if (!promptFilename(outname, sizeof(outname))) {
            refresh("Save canceled.");
            continue;  // cancel save, stay
          }
          if (!checkNameLen(outname)) {
            refresh("Error: filename too long.");
            continue;
          }
          if (saveAs(outname)) {
            refresh("Wrote file. Exiting.");
          } else {
            refresh("Write failed! Press any key to continue.");
            (void)readKey();
          }
        }
        // if ans == 0 (No) or not modified, exit without saving
        Term::clear();
        Term::showCursor();
        return true;
      }
      if (k == '\n') {
        newline();
        refresh(nullptr);
        continue;
      }
      if (k == 1000) {
        moveUp();
        refresh(nullptr);
        continue;
      }
      if (k == 1001) {
        moveDown();
        refresh(nullptr);
        continue;
      }
      if (k == 1002) {
        moveRight();
        refresh(nullptr);
        continue;
      }
      if (k == 1003) {
        moveLeft();
        refresh(nullptr);
        continue;
      }
      if (k == 1004) {
        delChar();
        refresh(nullptr);
        continue;
      }
      if (k == 0x08 || k == 0x7F) {
        backspace();
        refresh(nullptr);
        continue;
      }
      if (k >= 32 && k <= 126) {
        insertChar((char)k);
        refresh(nullptr);
        continue;
      }
      // ignore other keys
    }
  }
};

static bool runNanoEditor(const char* path) {
  NanoEditor ed;
  if (!ed.load(path)) {
    Console.println("nano: load failed");
    return false;
  }
  bool ok = ed.run();
  return ok;
}
// ----------------- End Nano-like editor -----------------

static void printHelp() {
  Console.println("Commands (filename max 32 chars):");
  Console.println("  help                         - this help");
  Console.println("  storage                      - show active storage");
  Console.println("  storage flash|psram          - switch storage backend");
  Console.println("  mode                         - show mode (dev/prod)");
  Console.println("  mode dev|prod                - set dev or prod mode");
  Console.println("  persist read                 - read persist file from active storage");
  Console.println("  persist write                - write persist file to active storage");
  Console.println("  blobs                        - list compiled-in blobs");
  Console.println("  autogen                      - auto-create enabled blobs if missing");
  Console.println("  files                        - list files in FS");
  Console.println("  info <file>                  - show file addr/size/cap");
  Console.println("  dump <file> <nbytes>         - hex dump head of file");
  Console.println("  mkSlot <file> <reserve>      - create sector-aligned slot");
  Console.println("  writeblob <file> <blobId>    - create/update file from blob");
  Console.printf("  exec <file> [a0..aN] [&]     - execute blob with 0..%d int args on core1; use '&' to background\n", (int)MAX_EXEC_ARGS);
  Console.println("                                (foreground blocks until completion; background returns immediately)");
  Console.println("  del <file>                   - delete a file");
  Console.println("  format                       - format active FS");
  Console.println("  wipe                         - erase active chip (DANGEROUS to FS)");
  Console.println("  wipereboot                   - erase chip then reboot (DANGEROUS to FS)");
  Console.println("  wipebootloader               - erase chip then reboot to bootloader (DANGEROUS to FS)");
  Console.println("  meminfo                      - show heap/stack info");
  Console.println("  psramsmoketest               - safe, non-destructive PSRAM test (prefers FS-free region)");
  Console.println("  bg [status|query]            - query background job status");
  Console.println("  bg kill|cancel [force]       - cancel background job; 'force' may reset core1 (platform-dependent)");
  Console.println("  reboot                       - reboot the MCU");
  Console.println("  timeout [ms]                 - show or set core1 timeout override (0=defaults)");
  Console.println("  puthex <file> <hex>          - upload binary as hex string (single line)");
  Console.println("  putb64 <file> <base64>       - upload binary as base64 (single line)");
  Console.println("  cc <src> <dst>               - compile Tiny-C source file to raw ARM Thumb-1 binary");
  Console.println();
  Console.println("Editor (Nano-style) commands:");
  Console.println("  nano <file>                  - open Nano-like editor for <file> (ActiveFS-backed)");
  Console.println("  edit <file>                  - alias for 'nano'");
  Console.println("    Keys in editor:");
  Console.println("      Arrows: move   Backspace/Delete: edit   Enter: newline");
  Console.println("      Ctrl+C: show cursor position   Ctrl+X: prompt to save and exit");
  Console.println();
  Console.println("Co-Processor (serial RPC) commands:");
  Console.println("  coproc ping                  - HELLO (version/features)");
  Console.println("  coproc info                  - INFO (flags, blob_len, mailbox_max)");
  Console.println("  coproc exec <file> [a0..aN]  - Load <file> from active FS to co-proc and EXEC with args");
  Console.println("                                 Timeout is taken from 'timeout' setting on the master");
  Console.println("  coproc sexec <file> [a0..aN] - Load a script file to co-proc and execute once");
  Console.println("                                 Uses master 'timeout' setting; script may use DELAY/DELAY_US, PINMODE, DWRITE, etc.");
  Console.println("  coproc func <name> [a0..aN]  - Call a named function registered on the co-processor");
  Console.println("  coproc status                - STATUS (exec_state)");
  Console.println("  coproc mbox [n]              - MAILBOX_RD (read up to n bytes; default 128)");
  Console.println("  coproc cancel                - CANCEL (sets cancel flag)");
  Console.println("  coproc reset                 - RESET (remote reboot)");
  Console.println("  coproc isp enter|exit        - request co-processor to enter or exit ISP mode (device-specific)");
  Console.println();
  Console.println("Linux hints:");
  Console.println("  # Base64 (single line, no wraps)  base64 -w0 your.bin");
  Console.println("  # Hex (single line, no spaces)    xxd -p -c 999999 your.bin | tr -d '\\n'");
  Console.println("Example:");
  Console.println("  putb64 myprog <paste_output_of_base64>");
  Console.println("  puthex myprog <paste_output_of_xxd>");
  Console.println("  cc myprog.c myprog.bin");
  Console.println("Notes:");
  Console.println("  - For blobs you intend to exec, even byte length is recommended (Thumb).");
  Console.println("  - Background jobs: only one background job supported at a time. Use 'bg' to query or cancel.");
  Console.println();
}

static void handleCommand(char* line) {
  char* p = line;
  char* t0;
  if (!nextToken(p, t0)) return;

  if (!strcmp(t0, "help")) {
    printHelp();

  } else if (!strcmp(t0, "nano") || !strcmp(t0, "edit")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Console.println("usage: nano <file>");
      return;
    }
    if (!checkNameLen(fn)) return;
    Console.printf("Opening editor for '%s'...\n", fn);
    runNanoEditor(fn);
    Console.println("Returned to console.");

  } else if (!strcmp(t0, "storage")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Console.print("Active storage: ");
      Console.println(g_storage == StorageBackend::Flash ? "flash" : "psram");
      return;
    }
    if (!strcmp(tok, "flash")) {
      g_storage = StorageBackend::Flash;
      bindActiveFs(g_storage);
      updateExecFsTable();
      bool ok = activeFs.mount(true);
      Console.println("Switched active storage to FLASH");
      Console.println(ok ? "Mounted FLASH (auto-format if empty)" : "Mount failed (FLASH)");
    } else if (!strcmp(tok, "psram")) {
      g_storage = StorageBackend::PSRAM_BACKEND;
      bindActiveFs(g_storage);
      updateExecFsTable();
      bool ok = activeFs.mount(false);
      Console.println("Switched active storage to PSRAM");
      Console.println(ok ? "Mounted PSRAM (no auto-format)" : "Mount failed (PSRAM)");
    } else {
      Console.println("usage: storage [flash|psram]");
    }

  } else if (!strcmp(t0, "mode")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Console.print("Mode: ");
      Console.println(g_dev_mode ? "dev" : "prod");
      return;
    }
    if (!strcmp(tok, "dev")) {
      g_dev_mode = true;
      Console.println("mode=dev");
    } else if (!strcmp(tok, "prod")) {
      g_dev_mode = false;
      Console.println("mode=prod");
    } else Console.println("usage: mode [dev|prod]");

  } else if (!strcmp(t0, "persist")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Console.println("usage: persist [read|write]");
      return;
    }
    if (!strcmp(tok, "read")) {
      const char* pf = ".persist";
      if (!activeFs.exists(pf)) {
        Console.println("persist read: not found");
        return;
      }
      uint32_t sz = 0;
      activeFs.getFileSize(pf, sz);
      uint8_t buf[PERSIST_LEN];
      memset(buf, 0, PERSIST_LEN);
      uint32_t got = activeFs.readFile(pf, buf, PERSIST_LEN);
      Console.printf("persist read: %d bytes: ", got);
      for (size_t i = 0; i < got; ++i) {
        if (i) Console.print(' ');
        printHexByte(buf[i]);
      }
      Console.println();
    } else if (!strcmp(tok, "write")) {
      const char* pf = ".persist";
      uint8_t buf[PERSIST_LEN];
      for (size_t i = 0; i < PERSIST_LEN; ++i) buf[i] = (uint8_t)(i ^ (g_dev_mode ? 0xA5 : 0x5A));
      if (!activeFs.exists(pf)) {
        if (!activeFs.createFileSlot(pf, ActiveFS::SECTOR_SIZE, buf, PERSIST_LEN)) {
          Console.println("persist write: create failed");
          return;
        }
      } else {
        if (!activeFs.writeFileInPlace(pf, buf, PERSIST_LEN, true)) {
          if (!activeFs.writeFile(pf, buf, PERSIST_LEN, static_cast<int>(W25QSimpleFS::WriteMode::ReplaceIfExists))) {
            Console.println("persist write: write failed");
            return;
          }
        }
      }
      Console.println("persist write: OK");
    } else {
      Console.println("usage: persist [read|write]");
    }

  } else if (!strcmp(t0, "autogen")) {
    autogenBlobWrites();

  } else if (!strcmp(t0, "files")) {
    activeFs.listFilesToSerial();

  } else if (!strcmp(t0, "info")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Console.println("usage: info <file>");
      return;
    }
    uint32_t a, s, c;
    if (activeFs.getFileInfo(fn, a, s, c)) {
      Console.print(fn);
      Console.print(": addr=0x");
      Console.print(a, HEX);
      Console.print(" size=");
      Console.print(s);
      Console.print(" cap=");
      Console.println(c);
    } else Console.println("not found");

  } else if (!strcmp(t0, "dump")) {
    char* fn;
    char* nstr;
    if (!nextToken(p, fn) || !nextToken(p, nstr)) {
      Console.println("usage: dump <file> <nbytes>");
      return;
    }
    dumpFileHead(fn, (uint32_t)strtoul(nstr, nullptr, 0));

  } else if (!strcmp(t0, "mkSlot")) {
    char* fn;
    char* nstr;
    if (!nextToken(p, fn) || !nextToken(p, nstr)) {
      Console.println("usage: mkSlot <file> <reserve>");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint32_t res = (uint32_t)strtoul(nstr, nullptr, 0);
    if (activeFs.createFileSlot(fn, res, nullptr, 0)) Console.println("slot created");
    else Console.println("mkSlot failed");

  } else if (!strcmp(t0, "writeblob")) {
    char* fn;
    char* bid;
    if (!nextToken(p, fn) || !nextToken(p, bid)) {
      Console.println("usage: writeblob <file> <blobId>");
      return;
    }
    if (!checkNameLen(fn)) return;
    const BlobReg* br = findBlob(bid);
    if (!br) {
      Console.println("unknown blobId; use 'blobs'");
      return;
    }
    if (ensureBlobFile(fn, br->data, br->len)) Console.println("writeblob OK");
    else Console.println("writeblob failed");

  } else if (!strcmp(t0, "exec")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Console.println("usage: exec <file> [a0 ... aN] [&]");
      return;
    }
    auto strip_trailing_amp = [](char* s) -> bool {
      if (!s) return false;
      size_t L = strlen(s);
      if (L == 0) return false;
      if (s[L - 1] == '&') {
        s[L - 1] = '\0';
        return true;
      }
      return false;
    };
    char* toks[MAX_EXEC_ARGS + 2];
    int tcount = 0;
    char* tok = nullptr;
    while (tcount < (int)MAX_EXEC_ARGS + 1 && nextToken(p, tok)) toks[tcount++] = tok;
    bool background = false;
    if (tcount > 0 && strcmp(toks[tcount - 1], "&") == 0) {
      background = true;
      --tcount;
    } else if (tcount == 0) {
      if (strip_trailing_amp(fn)) background = true;
    } else {
      if (strip_trailing_amp(toks[tcount - 1])) {
        background = true;
        if (toks[tcount - 1][0] == '\0') --tcount;
      }
    }
    static int32_t argvN[MAX_EXEC_ARGS];
    int argc = 0;
    for (int i = 0; i < tcount && argc < (int)MAX_EXEC_ARGS; ++i) argvN[argc++] = (int32_t)strtol(toks[i], nullptr, 0);
    if (!background) {
      int rv = 0;
      if (!Exec.execBlobForeground(fn, argc, argvN, rv)) { Console.println("exec failed"); }
    } else {
      void* raw = nullptr;
      uint8_t* buf = nullptr;
      uint32_t sz = 0;
      if (!Exec.loadFileToExecBuf(fn, raw, buf, sz)) {
        Console.println("exec: load failed");
        return;
      }
      uint32_t timeout = Exec.timeout(100000);
      if (!Exec.submitBackground(fn, raw, buf, sz, (uint32_t)argc, (const int32_t*)argvN, timeout)) {
        free(raw);
      }
    }

  } else if (!strcmp(t0, "bg")) {
    char* tok;
    if (!nextToken(p, tok)) {
      if (!Exec.bgActive()) Console.println("No active background job");
      else {
        uint32_t elapsed = Exec.bgElapsedMs();
        Console.printf("BG job: '%s'\n", Exec.bgName());
        Console.printf("  elapsed=%u ms  timeout=%u ms  job_flag=%u\n", (unsigned)elapsed, (unsigned)Exec.bgTimeoutMs(), (unsigned)Exec.core1JobFlag());
      }
      return;
    }
    if (!strcmp(tok, "status") || !strcmp(tok, "query")) {
      if (!Exec.bgActive()) Console.println("No active background job");
      else {
        uint32_t elapsed = Exec.bgElapsedMs();
        Console.printf("BG job: '%s'  elapsed=%u ms  timeout=%u ms  job_flag=%u\n",
                       Exec.bgName(), (unsigned)elapsed, (unsigned)Exec.bgTimeoutMs(), (unsigned)Exec.core1JobFlag());
      }
    } else if (!strcmp(tok, "kill") || !strcmp(tok, "cancel")) {
      char* sub = nullptr;
      nextToken(p, sub);
      if (!Exec.bgActive()) {
        if (Exec.core1JobFlag() == 1u) {
          if (Exec.cancelQueuedBackgroundIfAny()) {
            Console.println("Cleared queued core1 job");
          } else {
            Console.println("No active background job to cancel");
          }
        } else {
          Console.println("No active background job to cancel");
        }
        return;
      }
      if (Exec.core1JobFlag() == 1u) {
        if (Exec.cancelQueuedBackgroundIfAny()) Console.println("Background job canceled (was queued)");
        else Console.println("No queued job to cancel");
        return;
      }
      if (Exec.core1JobFlag() == 2u) {
        if (Exec.requestCancelRunningBackground()) {
          Console.println("Cancellation requested (mailbox flag set). If blob cooperates, it will stop shortly.");
        } else {
          Console.println("No running BG job to cancel");
        }
        return;
      }
      Console.println("Cancellation requested (best-effort).");
    } else {
      Console.println("bg: usage: bg [status|query] | bg kill|cancel [force]");
    }

  } else if (!strcmp(t0, "coproc")) {
    char* sub;
    if (!nextToken(p, sub)) {
      Console.println("coproc cmds:");
      Console.println("  coproc ping");
      Console.println("  coproc info");
      Console.println("  coproc exec <file> [a0..aN]");
      Console.println("  coproc sexec <file> [a0..aN]");
      Console.println("  coproc func <name> [a0..aN]");
      Console.println("  coproc status");
      Console.println("  coproc mbox [n]");
      Console.println("  coproc cancel");
      Console.println("  coproc reset");
      Console.println("  coproc isp enter|exit         - request co-processor to enter or exit ISP mode (device-specific)");
      return;
    }
    if (!strcmp(sub, "ping")) {
      if (!Exec.coprocHello()) Console.println("coproc ping failed");
    } else if (!strcmp(sub, "info")) {
      if (!Exec.coprocInfo()) Console.println("coproc info failed");
    } else if (!strcmp(sub, "sexec")) {
      char* fname = nullptr;
      if (!nextToken(p, fname)) {
        Console.println("usage: coproc sexec <file> [a0..aN]");
        return;
      }
      const char* tokens[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) tokens[argc++] = tok;
      if (!Exec.coprocScriptLoadFile(fname)) {
        Console.println("coproc sexec failed: load failed");
        return;
      }
      if (!Exec.coprocScriptExecTokens(tokens, argc)) Console.println("coproc sexec failed");
    } else if (!strcmp(sub, "exec")) {
      char* fname = nullptr;
      if (!nextToken(p, fname)) {
        Console.println("usage: coproc exec <file> [a0..aN]");
        return;
      }
      const char* tokens[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) tokens[argc++] = tok;
      if (!Exec.coprocLoadFile(fname)) {
        Console.println("coproc exec failed: load failed");
        return;
      }
      if (!Exec.coprocExecTokens(tokens, argc)) Console.println("coproc exec failed");
    } else if (!strcmp(sub, "func")) {
      char* name = nullptr;
      if (!nextToken(p, name)) {
        Console.println("usage: coproc func <name> [a0..aN]");
        return;
      }
      static int32_t argvN[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) {
        argvN[argc++] = (int32_t)strtol(tok, nullptr, 0);
      }
      int32_t result = 0;
      if (coprocCallFunc(name, argvN, argc, result)) {
        Console.print("coproc func OK, result=");
        Console.println(result);
      } else {
        Console.println("coproc func failed");
      }
    } else if (!strcmp(sub, "status")) {
      if (!Exec.coprocStatus()) Console.println("coproc status failed");
    } else if (!strcmp(sub, "mbox")) {
      char* nstr;
      uint32_t n = 128;
      if (nextToken(p, nstr)) n = (uint32_t)strtoul(nstr, nullptr, 0);
      if (!Exec.coprocMailboxRead(n)) Console.println("coproc mbox failed");
    } else if (!strcmp(sub, "cancel")) {
      if (!Exec.coprocCancel()) Console.println("coproc cancel failed");
    } else if (!strcmp(sub, "reset")) {
      if (!Exec.coprocReset()) Console.println("coproc reset failed (transport)");
    } else if (!strcmp(sub, "isp")) {
      char* arg = nullptr;
      if (!nextToken(p, arg)) {
        Console.println("usage: coproc isp enter|exit");
        return;
      }
      if (!strcmp(arg, "enter")) {
        if (!Exec.coprocIspEnter()) Console.println("coproc isp enter failed");
      } else if (!strcmp(arg, "exit")) {
        if (!Exec.coprocIspExit()) Console.println("coproc isp exit failed");
      } else {
        Console.println("usage: coproc isp enter|exit");
      }
    } else {
      Console.println("usage: coproc ping|info|exec <file> [a0..aN]|sexec <file> [a0..aN]|func <name> [a0..aN]|status|mbox [n]|cancel|reset");
    }

  } else if (!strcmp(t0, "blobs")) {
    listBlobs();

  } else if (!strcmp(t0, "timeout")) {
    char* msStr;
    if (!nextToken(p, msStr)) {
      Console.print("timeout override = ");
      Console.print((uint32_t)Exec.getTimeoutOverride());
      Console.println(" ms (0 = use per-call defaults)");
    } else {
      uint32_t ms = (uint32_t)strtoul(msStr, nullptr, 0);
      Exec.setTimeoutOverride(ms);
      Console.print("timeout override set to ");
      Console.print(ms);
      Console.println(" ms");
    }

  } else if (!strcmp(t0, "meminfo")) {
    Console.println();
    Console.printf("Total Heap:        %d bytes\n", rp2040.getTotalHeap());
    Console.printf("Free Heap:         %d bytes\n", rp2040.getFreeHeap());
    Console.printf("Used Heap:         %d bytes\n", rp2040.getUsedHeap());
    Console.printf("Total PSRAM Heap:  %d bytes\n", rp2040.getTotalPSRAMHeap());
    Console.printf("Free PSRAM Heap:   %d bytes\n", rp2040.getFreePSRAMHeap());
    Console.printf("Used PSRAM Heap:   %d bytes\n", rp2040.getUsedPSRAMHeap());
    Console.printf("Free Stack:        %d bytes\n", rp2040.getFreeStack());
    psramBB.printCapacityReport();

  } else if (!strcmp(t0, "psramsmoketest")) {
    psramBB.printCapacityReport();
    psramSafeSmokeTest();

  } else if (!strcmp(t0, "reboot")) {
    Console.printf("Rebooting..\n");
    delay(20);
    yield();
    rp2040.reboot();

  } else if (!strcmp(t0, "del")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Console.println("usage: del <file>");
      return;
    }
    if (activeFs.deleteFile && activeFs.deleteFile(fn)) Console.println("deleted");
    else Console.println("delete failed");

  } else if (!strcmp(t0, "format")) {
    if (activeFs.format && activeFs.format()) Console.println("FS formatted");
    else Console.println("format failed");

  } else if (!strcmp(t0, "wipebootloader")) {
    Console.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) {
      Console.println("Chip wiped, rebooting to bootloader now..");
      delay(20);
      yield();
      rp2040.rebootToBootloader();
    } else Console.println("wipe failed");

  } else if (!strcmp(t0, "wipereboot")) {
    Console.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) {
      Console.println("Chip wiped, rebooting now..");
      delay(20);
      yield();
      rp2040.reboot();
    } else Console.println("wipe failed");

  } else if (!strcmp(t0, "wipe")) {
    Console.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) Console.println("Chip wiped");
    else Console.println("wipe failed");

  } else if (!strcmp(t0, "puthex")) {
    char* fn;
    char* hex;
    if (!nextToken(p, fn) || !nextToken(p, hex)) {
      Console.println("usage: puthex <file> <hex>");
      Console.println("tip: xxd -p -c 999999 your.bin | tr -d '\\n'");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint8_t* bin = nullptr;
    uint32_t binLen = 0;
    if (!decodeHexString(hex, bin, binLen)) {
      Console.println("puthex: decode failed");
      return;
    }
    bool ok = writeBinaryToFS(fn, bin, binLen);
    free(bin);
    if (ok) {
      Console.print("puthex: wrote ");
      Console.print(binLen);
      Console.print(" bytes to ");
      Console.println(fn);
      if (binLen & 1u) Console.println("note: odd-sized file; if used as Thumb blob, exec will reject (needs even bytes).");
    } else Console.println("puthex: write failed");

  } else if (!strcmp(t0, "putb64")) {
    char* fn;
    char* b64;
    if (!nextToken(p, fn) || !nextToken(p, b64)) {
      Console.println("usage: putb64 <file> <base64>");
      Console.println("tip: base64 -w0 your.bin");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint8_t* bin = nullptr;
    uint32_t binLen = 0;
    if (!decodeBase64String(b64, bin, binLen)) {
      Console.println("putb64: decode failed");
      return;
    }
    bool ok = writeBinaryToFS(fn, bin, binLen);
    free(bin);
    if (ok) {
      Console.print("putb64: wrote ");
      Console.print(binLen);
      Console.print(" bytes to ");
      Console.println(fn);
      if (binLen & 1u) Console.println("note: odd-sized file; if used as Thumb blob, exec will reject (needs even bytes).");
    } else Console.println("putb64: write failed");

  } else if (!strcmp(t0, "cc")) {
    char* src;
    char* dst;
    if (!nextToken(p, src) || !nextToken(p, dst)) {
      Console.println("usage: cc <src> <dst>");
      Console.println("example: cc myprog.c myprog.bin");
      return;
    }
    if (!compileTinyCFileToFile(src, dst)) {
      Console.println("cc: failed");
    }

  } else {
    Console.println("Unknown command. Type 'help'.");
  }
}

// ========== Setup and main loops ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(20); }
  delay(20);
  Serial.println("System booting..");

  // Initialize Console
  Console.begin();

  // Initialize flash
  flashBB.begin();

  // PSRAM decoder + bus
  psramBB.configureDecoder138(PSRAM_NUM_CHIPS, PIN_138_EN, PIN_138_A0, PIN_138_A1, PIN_138_A2, /*enActiveLow=*/true);
  psramBB.begin();
  psramBB.setClockDelayUs(PSRAM_CLOCK_DELAY_US);

  // bind FS and mount
  bindActiveFs(g_storage);
  bool mounted = (g_storage == StorageBackend::Flash) ? activeFs.mount(true) : activeFs.mount(false);
  if (!mounted) { Console.println("FS mount failed on active storage"); }

  // Clear mailbox
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) BLOB_MAILBOX[i] = 0;

  // ExecHost wiring
  Exec.attachConsole(&Console);
  updateExecFsTable();
  Exec.attachCoProc(&coprocLink, COPROC_BAUD);

  Console.printf("Controller serial link ready @ %u bps (RX=GP%u, TX=GP%u)\n", (unsigned)COPROC_BAUD, (unsigned)PIN_COPROC_RX, (unsigned)PIN_COPROC_TX);
  Console.printf("System ready. Type 'help'\n> ");
}

void setup1() {
  Exec.core1Setup();
}
void loop1() {
  Exec.core1Poll();
  tight_loop_contents();
}

void loop() {
  Exec.pollBackground();  // handle any background job completions/timeouts
  if (readLine()) {
    Console.printf("\n> %s\n", lineBuf);
    handleCommand(lineBuf);
    Console.print("> ");
  }
}