/*
  main_psram_flash_switch.ino
  DEFAULT (golden) PSRAM CS SCHEME: Single 74HC138 (MC74HC138AN), 4 PSRAM chips
  -------------------------------------------------------------------------------
  - RP2040 pins (per your wiring):
      GP8 -> 74HC138 pin 1 (A0)
      GP7 -> 74HC138 pin 2 (A1)
      GP9 -> 74HC138 pins 4 and 5 (G2A/G2B tied together). 10k pull-up to 3.3V.
      74HC138 pin 3 (A2) -> GND
      74HC138 pin 6 (G1) -> 3.3V
      74HC138 pin 16 (VCC) -> 3.3V, pin 8 (GND) -> GND
      Decoupling: 0.1 µF across VCC/GND near the chip.
  - Outputs (ACTIVE-LOW):
      74HC138 Y0..Y3 -> PSRAM CS0..CS3 respectively, each with its own 10k pull-up to 3.3V.
  - SPI bus shared (bit-banged): MISO=GP11, MOSI=GP12, SCK=GP10
  Selection sequence (handled in library):
    EN (G2A/G2B) HIGH -> set A0/A1 -> EN LOW -> transfer -> EN HIGH
  Notes:
  - If you later want Direct CS or 74HC138+74HC595, keep this file as-is and switch in code later.
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
// Default PSRAM_AGGREGATE_BUS remains PSRAMBitbang via PSRAMMulti.h
#endif

#include "PSRAMMulti.h"

#include "blob_mailbox_config.h"
#include "blob_ret42.h"
#define FILE_RET42 "ret42"
#include "blob_pwmc.h"
#define FILE_PWMC "pwmc"
struct BlobReg;               // Forward declaration so findBlob can be placed earlier if needed
static ConsolePrint Console;  // Console wrapper

// ========== Static buffer-related compile-time constant (used previously as fs.SECTOR_SIZE) ==========
#define FS_SECTOR_SIZE 4096

// ========== bitbang pin definitions (flash) ==========
const uint8_t PIN_FLASH_MISO = 12;  // GP12
const uint8_t PIN_FLASH_CS = 28;    // GP28
const uint8_t PIN_FLASH_SCK = 10;   // GP10
const uint8_t PIN_FLASH_MOSI = 11;  // GP11

// ========== bitbang pin definitions (psram) ==========
const bool PSRAM_ENABLE_QPI = false;     // Should we enable the PSRAM QPI-mode?
const uint8_t PSRAM_CLOCK_DELAY_US = 0;  // Small delay helps ensure decoder/address settle before asserting EN
const uint8_t PIN_PSRAM_MISO = 12;       // GP12
const uint8_t PIN_PSRAM_MOSI = 11;       // GP11
const uint8_t PIN_PSRAM_SCK = 10;        // GP10
//const uint8_t PIN_PSRAM_IO2 = ;  // only used if QPI enabled
//const uint8_t PIN_PSRAM_IO3 = ;  // only used if QPI enabled

// 74HC138 pins (for 138-only default)
const uint8_t PIN_138_A0 = 8;  // 74HC138 pin 1 (A0)
const uint8_t PIN_138_A1 = 7;  // 74HC138 pin 2 (A1)
const uint8_t PIN_138_EN = 9;  // 74HC138 pins 4+5 (G2A/G2B tied), 10k pull-up to 3.3V
// If using up to 8 chips with A2 controlled by MCU, set a pin here; else keep 255 and tie A2 to GND.
const uint8_t PIN_138_A2 = 255;  // 255 means "not used"; tie 74HC138 pin 3 (A2) to GND
// OPTIONAL: 74HC595 latch pin for method 138+595 (SER=MOSI, SRCLK=SCK are reused)
const uint8_t PIN_595_LATCH = 4;  // 595.RCLK (unused in 138-only mode)

// ========== Flash/PSRAM instances (needed before bindActiveFs) ==========
const size_t PERSIST_LEN = 32;
enum class StorageBackend {
  // Choose active storage at runtime
  Flash,
  PSRAM_BACKEND
};
static StorageBackend g_storage = StorageBackend::Flash;

// Flash: restore original bitbang flash driver so W25QSimpleFS works unchanged
/*W25QBitbang flashBB(PIN_FLASH_MISO, PIN_FLASH_CS, PIN_FLASH_SCK, PIN_FLASH_MOSI);
W25QSimpleFS fsFlash(flashBB);*/

// Capacity
constexpr uint8_t PSRAM_NUM_CHIPS = 4;
constexpr uint32_t PER_CHIP_CAP_BYTES = 8UL * 1024UL * 1024UL;  // Example: 8MB per chip.
constexpr uint32_t AGG_CAPACITY_BYTES = PER_CHIP_CAP_BYTES * PSRAM_NUM_CHIPS;

// PSRAMAggregateDevice: use concrete class (macro selected underlying bus inside PSRAMMulti.h)
/*PSRAMAggregateDevice psramBB(PIN_PSRAM_MISO, PIN_PSRAM_MOSI, PIN_PSRAM_SCK, PER_CHIP_CAP_BYTES);
PSRAMSimpleFS_Multi fsPSRAM(psramBB, AGG_CAPACITY_BYTES);*/

// Flash (bit-banged)
W25QBitbang flashBB(PIN_FLASH_MISO, PIN_FLASH_CS, PIN_FLASH_SCK, PIN_FLASH_MOSI);
W25QSimpleFS fsFlash(flashBB);

// PSRAM aggregate using PSRAMBitbang (PSRAM_AGGREGATE_BUS defaults to PSRAMBitbang)
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
  bool (*deleteFile)(const char*) = nullptr;  // used by 'del'
  void (*listFilesToSerial)() = nullptr;
  uint32_t (*nextDataAddr)() = nullptr;
  uint32_t (*capacity)() = nullptr;
  uint32_t (*dataRegionStart)() = nullptr;
  static constexpr uint32_t SECTOR_SIZE = FS_SECTOR_SIZE;
  static constexpr uint32_t PAGE_SIZE = 256;
  static constexpr size_t MAX_NAME = 32;
} activeFs;

static void bindActiveFs(StorageBackend backend) {
  // Helper to bind either flash or psram into activeFs at runtime
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

// ========== Blob registry from your original file ==========
struct BlobReg {
  const char* id;
  const uint8_t* data;
  unsigned int len;
};
static const BlobReg g_blobs[] = {
  { FILE_RET42, blob_ret42, blob_ret42_len },
  { FILE_PWMC, blob_pwmc, blob_pwmc_len },
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

// ========== Mailbox helpers ==========
static inline void mailboxClearFirstByte() {
  volatile uint8_t* mb = (volatile uint8_t*)(uintptr_t)BLOB_MAILBOX_ADDR;
  mb[0] = 0;  // sentinel
}
static void mailboxPrintIfAny() {
  const volatile char* p = (const volatile char*)(uintptr_t)BLOB_MAILBOX_ADDR;
  if (p[0] == '\0') return;
  Console.print(" Info=\"");
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) {
    char c = p[i];
    if (!c) break;
    Console.print(c);
  }
  Console.println("\"");
}

// ========== Core1 execution plumbing (mailbox) ==========
#ifndef MAX_EXEC_ARGS
#define MAX_EXEC_ARGS 64
#endif

extern "C" int call_with_args_thumb(void* entryThumb, uint32_t argc, const int32_t* args) {
  if (!entryThumb) return 0;
  constexpr uint32_t N = 64;
  int32_t a64[N] = { 0 };
  uint32_t n = argc;
  if (n > N) n = N;
  if (args && n) {
    for (uint32_t i = 0; i < n; ++i) a64[i] = args[i];
  }
  using fvar_t = int (*)(...);  // AAPCS: r0..r3 then stack
  fvar_t f = reinterpret_cast<fvar_t>(entryThumb);
  return f(
    a64[0], a64[1], a64[2], a64[3], a64[4], a64[5], a64[6], a64[7],
    a64[8], a64[9], a64[10], a64[11], a64[12], a64[13], a64[14], a64[15],
    a64[16], a64[17], a64[18], a64[19], a64[20], a64[21], a64[22], a64[23],
    a64[24], a64[25], a64[26], a64[27], a64[28], a64[29], a64[30], a64[31],
    a64[32], a64[33], a64[34], a64[35], a64[36], a64[37], a64[38], a64[39],
    a64[40], a64[41], a64[42], a64[43], a64[44], a64[45], a64[46], a64[47],
    a64[48], a64[49], a64[50], a64[51], a64[52], a64[53], a64[54], a64[55],
    a64[56], a64[57], a64[58], a64[59], a64[60], a64[61], a64[62], a64[63]);
}

struct ExecJob {
  uintptr_t code;               // raw aligned code address (LSB 0)
  uint32_t size;                // code size (bytes, even)
  uint32_t argc;                // number of int args (0..MAX_EXEC_ARGS)
  int32_t args[MAX_EXEC_ARGS];  // argument values
};

static volatile ExecJob g_job;
static volatile int32_t g_result = 0;
static volatile int32_t g_status = 0;
static volatile uint32_t g_job_flag = 0;  // 0=idle, 1=ready, 2=running, 3=done

static void core1WorkerPoll() {
  if (g_job_flag != 1u) return;
  __asm volatile("dsb" ::
                   : "memory");
  __asm volatile("isb" ::
                   : "memory");
  g_job_flag = 2u;
  ExecJob job;
  job.code = g_job.code;
  job.size = g_job.size;
  job.argc = g_job.argc;
  if (job.argc > MAX_EXEC_ARGS) job.argc = MAX_EXEC_ARGS;
  for (uint32_t i = 0; i < job.argc; ++i) job.args[i] = g_job.args[i];
  int32_t rv = 0;
  int32_t st = 0;
  if (job.code == 0 || (job.size & 1u)) {
    st = -1;  // invalid entry
  } else {
    void* entryThumb = (void*)(job.code | 1u);  // set Thumb bit
    rv = call_with_args_thumb(entryThumb, job.argc, job.args);
  }
  g_result = rv;
  g_status = st;
  __asm volatile("dsb" ::
                   : "memory");
  __asm volatile("isb" ::
                   : "memory");
  g_job_flag = 3u;
}

static volatile uint32_t g_timeout_override_ms = 0;
static inline uint32_t getTimeout(uint32_t defMs) {
  return g_timeout_override_ms ? g_timeout_override_ms : defMs;
}

static bool loadFileToExecBuf(const char* fname, void*& rawOut, uint8_t*& alignedBuf, uint32_t& szOut);  // Forward declaration

static bool runOnCore1(uintptr_t codeAligned, uint32_t sz, uint32_t argc, const int32_t* argv, int& retVal, uint32_t timeoutMs = 100) {
  if (g_job_flag != 0u) {
    Console.println("core1 busy");
    return false;
  }
  if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
  g_job.code = codeAligned;
  g_job.size = sz;
  g_job.argc = argc;
  for (uint32_t i = 0; i < argc; ++i) g_job.args[i] = argv[i];
  g_status = 0;
  g_result = 0;
  __asm volatile("dsb" ::
                   : "memory");
  __asm volatile("isb" ::
                   : "memory");
  g_job_flag = 1u;
  uint32_t start = millis();
  while (g_job_flag != 3u) {
    tight_loop_contents();
    if ((millis() - start) > timeoutMs) {
      if (g_job_flag == 3u) break;
      Console.println("core1 timeout");
      g_job_flag = 0u;
      return false;
    }
  }
  if (g_status != 0) {
    Console.print("core1 error status=");
    Console.println((int)g_status);
    g_job_flag = 0u;
    return false;
  }
  retVal = (int)g_result;
  g_job_flag = 0u;
  return true;
}

static bool execBlobGeneric(const char* fname, int argc, const int argv[], int& retVal) {
  if (argc < 0) argc = 0;
  if (argc > (int)MAX_EXEC_ARGS) argc = (int)MAX_EXEC_ARGS;
  void* raw = nullptr;
  uint8_t* buf = nullptr;
  uint32_t sz = 0;
  if (!loadFileToExecBuf(fname, raw, buf, sz)) return false;
  mailboxClearFirstByte();
  uintptr_t code = (uintptr_t)buf;
  Console.print("Calling entry on core1 at 0x");
  Console.println((uintptr_t)buf, HEX);
  bool ok = runOnCore1(code, sz, (uint32_t)argc, (const int32_t*)argv, retVal, getTimeout(100000));
  if (!ok) {
    Console.println("exec: core1 run failed");
    free(raw);
    return false;
  }
  Console.print("Return=");
  Console.println(retVal);
  mailboxPrintIfAny();
  free(raw);
  return true;
}

// Background execution support (one background job at a time)
static volatile void* g_bg_raw = nullptr;
static volatile uint8_t* g_bg_buf = nullptr;
static volatile uint32_t g_bg_sz = 0;
static volatile bool g_bg_active = false;
static volatile uint32_t g_bg_start_ms = 0;
static volatile uint32_t g_bg_timeout_ms = 0;
static char g_bg_fname[ActiveFS::MAX_NAME + 1] = { 0 };
static volatile bool g_bg_cancel_requested = false;

static bool submitJobBackground(const char* fname, void* raw, uint8_t* alignedBuf, uint32_t sz,
                                uint32_t argc, const int32_t* argv, uint32_t timeoutMs) {
  if (!fname || !raw || !alignedBuf || sz == 0) return false;
  if (g_job_flag != 0u) {
    Console.println("core1 busy");
    return false;
  }
  g_bg_cancel_requested = false;
  g_bg_raw = raw;
  g_bg_buf = alignedBuf;
  g_bg_sz = sz;
  g_bg_active = true;
  g_bg_start_ms = millis();
  g_bg_timeout_ms = timeoutMs;
  strncpy(g_bg_fname, fname, sizeof(g_bg_fname) - 1);
  g_bg_fname[sizeof(g_bg_fname) - 1] = '\0';
  g_job.code = (uintptr_t)alignedBuf;
  g_job.size = sz;
  g_job.argc = (argc > MAX_EXEC_ARGS) ? MAX_EXEC_ARGS : argc;
  for (uint32_t i = 0; i < (uint32_t)g_job.argc; ++i) g_job.args[i] = (int32_t)argv[i];
  mailboxClearCancelFlag();
  __asm volatile("dsb" ::
                   : "memory");
  __asm volatile("isb" ::
                   : "memory");
  g_job_flag = 1u;
  Console.printf("Background job '%s' started on core1 at 0x%08X (sz=%u)\n",
                 g_bg_fname, (unsigned)g_job.code, (unsigned)g_job.size);
  return true;
}

static void pollBackgroundExec() {
  if (!g_bg_active) {
    if (g_job_flag == 3u) {
      __asm volatile("dsb" ::
                       : "memory");
      __asm volatile("isb" ::
                       : "memory");
      g_job_flag = 0u;
    }
    return;
  }
  if (g_job_flag == 3u) {
    int32_t result = (int32_t)g_result;
    int32_t status = (int32_t)g_status;
    if (g_bg_cancel_requested) {
      Console.printf("Background job '%s' finished on core1 but was canceled — result ignored\n", g_bg_fname);
    } else {
      Console.printf("Background job '%s' completed: Return=%d\n", g_bg_fname, (int)result);
      if (status != 0) Console.printf(" Background job status=%d\n", (int)status);
      mailboxPrintIfAny();
    }
    mailboxClearCancelFlag();
    if (g_bg_raw) free((void*)g_bg_raw);
    g_bg_raw = nullptr;
    g_bg_buf = nullptr;
    g_bg_sz = 0;
    g_bg_active = false;
    g_bg_start_ms = 0;
    g_bg_timeout_ms = 0;
    g_bg_fname[0] = '\0';
    g_bg_cancel_requested = false;
    __asm volatile("dsb" ::
                     : "memory");
    __asm volatile("isb" ::
                     : "memory");
    g_job_flag = 0u;
    return;
  }
  uint32_t timeoutMs = g_bg_timeout_ms ? g_bg_timeout_ms : 0;
  if (g_bg_active && timeoutMs != 0) {
    uint32_t elapsed = (uint32_t)(millis() - g_bg_start_ms);
    if (elapsed > timeoutMs) {
      Console.println("core1 timeout (background job)");
      g_bg_cancel_requested = true;
      mailboxSetCancelFlag(1);
      const uint32_t graceMs = 50;
      uint32_t waited = 0;
      while (waited < graceMs) {
        if (g_job_flag == 3u) {
          return;
        }
        delay(1);
        ++waited;
      }
      if (g_bg_raw) free((void*)g_bg_raw);
      g_bg_raw = nullptr;
      g_bg_buf = nullptr;
      g_bg_sz = 0;
      g_bg_active = false;
      g_bg_start_ms = 0;
      g_bg_timeout_ms = 0;
      g_bg_fname[0] = '\0';
      __asm volatile("dsb" ::
                       : "memory");
      __asm volatile("isb" ::
                       : "memory");
      g_job_flag = 0u;
      return;
    }
  }
}

static inline void mailboxSetCancelFlag(uint8_t v) {
  volatile uint8_t* mb = (volatile uint8_t*)(uintptr_t)BLOB_MAILBOX_ADDR;
  mb[0] = v;
}
static inline void mailboxClearCancelFlag() {
  volatile uint8_t* mb = (volatile uint8_t*)(uintptr_t)BLOB_MAILBOX_ADDR;
  mb[0] = 0;
}
static inline uint8_t mailboxGetCancelFlag() {
  volatile uint8_t* mb = (volatile uint8_t*)(uintptr_t)BLOB_MAILBOX_ADDR;
  return mb[0];
}

// ========== FS helpers and console ==========
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

static bool loadFileToExecBuf(const char* fname, void*& rawOut, uint8_t*& alignedBuf, uint32_t& szOut) {
  rawOut = nullptr;
  alignedBuf = nullptr;
  szOut = 0;
  uint32_t sz = 0;
  if (!activeFs.getFileSize(fname, sz) || sz == 0) {
    Console.println("load: missing/empty");
    return false;
  }
  if (sz & 1u) {
    Console.println("load: odd-sized blob (Thumb requires 16-bit alignment)");
    return false;
  }
  void* raw = malloc(sz + 4);
  if (!raw) {
    Console.println("load: malloc failed");
    return false;
  }
  uint8_t* buf = (uint8_t*)((((uintptr_t)raw) + 3) & ~((uintptr_t)3));
  if (activeFs.readFile(fname, buf, sz) != sz) {
    Console.println("load: read failed");
    free(raw);
    return false;
  }
  rawOut = raw;
  alignedBuf = buf;
  szOut = sz;
  return true;
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
  allOk &= ensureBlobIfMissing(FILE_PWMC, blob_pwmc, blob_pwmc_len);
  Console.print("Autogen:  ");
  Console.println(allOk ? "OK" : "some failures");
}

static bool psramSafeSmokeTest() {
  // Requirements: PSRAM must be the active FS backend for safe behavior
  if (g_storage != StorageBackend::PSRAM_BACKEND) {
    Console.println("psramSafeSmokeTest: active storage is not PSRAM — aborting");
    return false;
  }
  // Updated: aim to fill as much of PSRAM as possible (across all 4 banks)
  const uint32_t UPDATE_STEP_PERCENT = 5;  // progress update granularity in percent
  const uint32_t MAX_SLOTS = 2044;         // bump to accommodate large fills
  const uint32_t PAGE = 256;               // write/read granularity
  const bool VERIFY_READBACK = true;       // verify after each chunk
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
  // FS head
  uint32_t fsNext = 0, fsDataStart = 0;
  if (activeFs.nextDataAddr) fsNext = activeFs.nextDataAddr();
  if (activeFs.dataRegionStart) fsDataStart = activeFs.dataRegionStart();
  auto alignUp = [](uint32_t v, uint32_t a) -> uint32_t {
    return (v + (a - 1)) & ~(a - 1);
  };
  const uint32_t fsAligned = alignUp(fsNext, SECT);
  Console.printf("\nPSRAM multi-slot smoke: chips=%u per_chip=%u total=%u fsNext=%u fsAligned=%u\n",
                 (unsigned)chips, (unsigned)perChip, (unsigned)totalCap, (unsigned)fsNext, (unsigned)fsAligned);
  // Compute how much space is effectively free beyond FS (to maximize fill)
  const uint32_t approxFree = (totalCap > fsAligned) ? (totalCap - fsAligned) : 0;
  Console.printf("Plan: allocate approx %u bytes of PSRAM free space\n", (unsigned)approxFree);
  if (approxFree == 0) {
    Console.println("PSRAM: no free space beyond FS; smoke test aborted");
    return false;
  }
  // Determine allocation strategy
  uint32_t stepsDesired = 100u / UPDATE_STEP_PERCENT;
  if (stepsDesired > MAX_SLOTS) {
    Console.printf("NOTE: stepsDesired=%u > MAX_SLOTS=%u, capping steps to %u\n",
                   (unsigned)stepsDesired, (unsigned)MAX_SLOTS, (unsigned)MAX_SLOTS);
    stepsDesired = MAX_SLOTS;
  }
  uint32_t numSlots = stepsDesired;
  // base reserve per slot (rounded up to SECT)
  uint32_t baseReserve = (approxFree + numSlots - 1) / numSlots;
  baseReserve = alignUp(baseReserve, SECT);
  // If baseReserve * numSlots exceeds approxFree, lower numSlots
  while ((uint64_t)baseReserve * numSlots > approxFree && numSlots > 1) {
    --numSlots;
    baseReserve = (approxFree + numSlots - 1) / numSlots;
    baseReserve = alignUp(baseReserve, SECT);
  }
  if ((uint64_t)baseReserve * numSlots > approxFree) {
    Console.printf("Not enough free space for %u slots (need %u bytes, have %u). Aborting.\n",
                   (unsigned)numSlots, (unsigned)((uint64_t)baseReserve * numSlots), (unsigned)approxFree);
    return false;
  }
  Console.printf("Allocating %u slots of ~%u bytes each (aligned to %u)\n", (unsigned)numSlots, (unsigned)baseReserve, (unsigned)SECT);
  // Stage A: Allocate slots incrementally and show allocation progress on new lines
  char slotName[32];
  uint32_t allocatedBytes = 0;
  uint32_t percentNextAlloc = UPDATE_STEP_PERCENT;  // next threshold (1..100)
  uint32_t createdSlots = 0;
  // Print initial allocation stage
  Console.println("Stage: Allocate slots");
  for (uint32_t i = 0; i < numSlots; ++i) {
    snprintf(slotName, sizeof(slotName), ".span_part_%03u", (unsigned)i);
    // If slot exists already, remove it first (fresh slot)
    if (activeFs.exists && activeFs.exists(slotName)) {
      if (activeFs.deleteFile) activeFs.deleteFile(slotName);
    }
    // reserve this slot
    uint32_t reserve = baseReserve;
    uint32_t remainNeeded = (allocatedBytes >= approxFree) ? 0 : (approxFree - allocatedBytes);
    if (reserve > remainNeeded && i == numSlots - 1) {
      // ensure last slot size is at least SECT and covers remaining
      reserve = alignUp(remainNeeded ? remainNeeded : SECT, SECT);
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
  // Ensure final 100% printed for allocation stage
  if (percentNextAlloc <= 100) {
    while (percentNextAlloc <= 100) {
      Console.printf("Alloc progress: %u%%\n", (unsigned)percentNextAlloc);
      percentNextAlloc += UPDATE_STEP_PERCENT;
    }
  }
  Console.println("Allocation complete");
  // Stage B: Write data into each slot sequentially
  uint8_t page[PAGE];
  uint8_t readbuf[PAGE];
  uint64_t totalToWrite = approxFree;
  uint64_t totalWritten = 0;
  uint32_t globalNextPct = UPDATE_STEP_PERCENT;
  Console.println("Stage: Writing data into slots");
  for (uint32_t i = 0; i < createdSlots; ++i) {
    snprintf(slotName, sizeof(slotName), ".span_part_%03u", (unsigned)i);
    // Obtain slot address and capacity
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
      // deterministic pattern
      for (uint32_t k = 0; k < n; ++k) {
        page[k] = (uint8_t)(0xA5 ^ (uint8_t)i ^ (uint8_t)((slotWritten + k) & 0xFF));
      }
      if (!psramBB.writeData02(addr + slotWritten, page, n)) {
        Console.printf("  write failed for slot %u at offset %u\n", (unsigned)i, (unsigned)slotWritten);
        return false;
      }
      if (VERIFY_READBACK) {
        memset(readbuf, 0, sizeof(readbuf));
        if (!psramBB.readData03(addr + slotWritten, readbuf, n)) {
          Console.printf("  readback failed for slot %u at offset %u\n", (unsigned)i, (unsigned)slotWritten);
          return false;
        }
        if (memcmp(readbuf, page, n) != 0) {
          Console.printf("  verify mismatch slot %u offset %u\n", (unsigned)i, (unsigned)slotWritten);
          return false;
        }
      }
      slotWritten += n;
      totalWritten += n;
      uint32_t slotPct = (uint32_t)(((uint64_t)slotWritten * 100u) / (uint64_t)toWrite);
      if (slotPct > 100) slotPct = 100;
      uint32_t globalPct = (uint32_t)(((uint64_t)totalWritten * 100u) / (uint64_t)totalToWrite);
      if (globalPct > 100) globalPct = 100;
      // newline progress
      Console.printf("Global: %u%%, Slot %03u: %u%%, offset=%u, written=%u\n",
                     (unsigned)globalPct, (unsigned)i, (unsigned)slotPct, (unsigned)slotWritten, (unsigned)slotWritten);
      if ((slotWritten % SECT) == 0) yield();
    }
    Console.printf("Slot %03u complete: written=%u\n", (unsigned)i, (unsigned)toWrite);
  }
  // Ensure global 100% printed
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

// ========== Serial console / command handling ==========
static char lineBuf[FS_SECTOR_SIZE];
static bool readLine() {
  static size_t pos = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[pos] = 0;
      pos = 0;
      return true;
    }
    if (pos + 1 < sizeof(lineBuf)) lineBuf[pos++] = c;
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
  Console.println();
  Console.println("Linux hints:");
  Console.println("  # Base64 (single line, no wraps)  base64 -w0 your.bin");
  Console.println("  # Hex (single line, no spaces)    xxd -p -c 999999 your.bin | tr -d '\\n'");
  Console.println("Example:");
  Console.println("  putb64 myprog <paste_output_of_base64>");
  Console.println("  puthex myprog <paste_output_of_xxd>");
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
      bool ok = activeFs.mount(true);
      Console.println("Switched active storage to FLASH");
      Console.println(ok ? "Mounted FLASH (auto-format if empty)" : "Mount failed (FLASH)");
    } else if (!strcmp(tok, "psram")) {
      g_storage = StorageBackend::PSRAM_BACKEND;
      bindActiveFs(g_storage);
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
    static int argvN[MAX_EXEC_ARGS];
    int argc = 0;
    for (int i = 0; i < tcount && argc < (int)MAX_EXEC_ARGS; ++i) argvN[argc++] = (int)strtol(toks[i], nullptr, 0);
    if (!background) {
      int rv;
      if (!execBlobGeneric(fn, argc, argvN, rv)) { Console.println("exec failed"); }
    } else {
      void* raw = nullptr;
      uint8_t* buf = nullptr;
      uint32_t sz = 0;
      if (!loadFileToExecBuf(fn, raw, buf, sz)) {
        Console.println("exec: load failed");
        return;
      }
      uint32_t timeout = getTimeout(100000);
      if (!submitJobBackground(fn, raw, buf, sz, (uint32_t)argc, (const int32_t*)argvN, timeout)) {
        free(raw);
      }
    }
  } else if (!strcmp(t0, "bg")) {
    char* tok;
    if (!nextToken(p, tok)) {
      if (!g_bg_active) Console.println("No active background job");
      else {
        uint32_t elapsed = millis() - g_bg_start_ms;
        Console.printf("BG job: '%s'\n", g_bg_fname);
        Console.printf("  elapsed=%u ms  timeout=%u ms  job_flag=%u\n", (unsigned)elapsed, (unsigned)g_bg_timeout_ms, (unsigned)g_job_flag);
      }
      return;
    }
    if (!strcmp(tok, "status") || !strcmp(tok, "query")) {
      if (!g_bg_active) Console.println("No active background job");
      else {
        uint32_t elapsed = millis() - g_bg_start_ms;
        Console.printf("BG job: '%s'  elapsed=%u ms  timeout=%u ms  job_flag=%u\n",
                       g_bg_fname, (unsigned)elapsed, (unsigned)g_bg_timeout_ms, (unsigned)g_job_flag);
      }
    } else if (!strcmp(tok, "kill") || !strcmp(tok, "cancel")) {
      char* sub = nullptr;
      nextToken(p, sub);
      if (!g_bg_active) {
        if (g_job_flag == 1u) {
          __asm volatile("dsb" ::
                           : "memory");
          __asm volatile("isb" ::
                           : "memory");
          g_job_flag = 0u;
          Console.println("Cleared queued core1 job");
        } else {
          Console.println("No active background job to cancel");
        }
        return;
      }
      if (g_job_flag == 1u) {
        __asm volatile("dsb" ::
                         : "memory");
        __asm volatile("isb" ::
                         : "memory");
        g_job_flag = 0u;
        void* raw = (void*)g_bg_raw;
        if (raw) free(raw);
        g_bg_raw = nullptr;
        g_bg_buf = nullptr;
        g_bg_sz = 0;
        g_bg_active = false;
        g_bg_start_ms = 0;
        g_bg_timeout_ms = 0;
        g_bg_fname[0] = '\0';
        g_bg_cancel_requested = false;
        Console.println("Background job canceled (was queued)");
        return;
      }
      if (g_job_flag == 2u) {
        g_bg_cancel_requested = true;
        mailboxSetCancelFlag(1);
        Console.println("Cancellation requested (mailbox flag set). If blob cooperates, it will stop shortly.");
        if (sub && strcmp(sub, "force") == 0) {
#if defined(PICO_ON_DEVICE)
          Console.println("Force kill: resetting core1 (PICO_ON_DEVICE)");
          multicore_reset_core1();
#else
          Console.println("Force kill not supported on this build (no PICO_ON_DEVICE).");
#endif
        }
        return;
      }
      g_bg_cancel_requested = true;
      Console.println("Cancellation requested (best-effort).");
    } else {
      Console.println("bg: usage: bg [status|query] | bg kill|cancel [force]");
    }
  } else if (!strcmp(t0, "blobs")) {
    listBlobs();
  } else if (!strcmp(t0, "timeout")) {
    char* msStr;
    if (!nextToken(p, msStr)) {
      Console.print("timeout override = ");
      Console.print((uint32_t)g_timeout_override_ms);
      Console.println(" ms (0 = use per-call defaults)");
    } else {
      uint32_t ms = (uint32_t)strtoul(msStr, nullptr, 0);
      g_timeout_override_ms = ms;
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
  } else {
    Console.println("Unknown command. Type 'help'.");
  }
}

// ========== Setup and main loop ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(20); }
  delay(20);
  Serial.println("System booting..");

  // Initialize Console
  Console.begin();

  // Initialize flash
  flashBB.begin();

  // ---------- PSRAM CS mode selection ----------
  // 74HC138-only mode (this must be called before psramBB.begin())
  psramBB.configureDecoder138(PSRAM_NUM_CHIPS, PIN_138_EN, PIN_138_A0, PIN_138_A1, PIN_138_A2, /*enActiveLow=*/true);
  // Start PSRAM bus (adapter inside PSRAMAggregateDevice will be used)
  psramBB.begin();
  psramBB.setClockDelayUs(PSRAM_CLOCK_DELAY_US);

  // Avoid configuring QPI IO2/IO3 lines to prevent pin conflict unless you wired them explicitly.
  // if (PSRAM_ENABLE_QPI) {
  //   psramBB.setExtraDataPins(PIN_PSRAM_IO2, PIN_PSRAM_IO3);
  //   psramBB.setModeQuad(true);
  // }

  bindActiveFs(g_storage);
  bool mounted = (g_storage == StorageBackend::Flash) ? activeFs.mount(true) : activeFs.mount(false);
  if (!mounted) { Console.println("FS mount failed on active storage"); }

  // Clear mailbox
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) BLOB_MAILBOX[i] = 0;

  Console.printf("System ready. Type 'help'\n> ");
}

void setup1() {
  g_job_flag = 0u;
}

void loop1() {
  core1WorkerPoll();
  tight_loop_contents();
}

void loop() {
  pollBackgroundExec();  // handle any background job completions/timeouts
  if (readLine()) {
    Console.printf("\n> %s\n", lineBuf);
    handleCommand(lineBuf);
    Console.print("> ");
  }
}