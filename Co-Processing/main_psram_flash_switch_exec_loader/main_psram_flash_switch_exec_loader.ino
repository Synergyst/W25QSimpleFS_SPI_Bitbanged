/*
  main_psram_flash_switch_exec_loader_serial.ino
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
  - SPI bus shared: MISO=GP12, MOSI=GP11, SCK=GP10
  Selection sequence (handled in library):
    EN (G2A/G2B) HIGH -> set A0/A1 -> EN LOW -> transfer -> EN HIGH
  Notes:
  - Co-processor (RP2350) connected over software serial (bitbanged UART):
      Main RP2040: RX=GP0, TX=GP1
      Co-Proc RP2350: RX=GP0, TX=GP1
      Wire TX<->RX cross: RP2040.GP1 -> RP2350.GP0, RP2040.GP0 <- RP2350.GP1
  - Protocol: CoProcProto.h framed RPC (24B header + payload + CRC32(payload)).
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
#include "CoProcProto.h"  // shared single-header protocol
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
constexpr uint8_t PSRAM_NUM_CHIPS = 4;
constexpr uint32_t PER_CHIP_CAP_BYTES = 8UL * 1024UL * 1024UL;  // Example: 8MB per chip.
constexpr uint32_t AGG_CAPACITY_BYTES = PER_CHIP_CAP_BYTES * PSRAM_NUM_CHIPS;
W25QBitbang flashBB(PIN_FLASH_MISO, PIN_FLASH_CS, PIN_FLASH_SCK, PIN_FLASH_MOSI);
W25QSimpleFS fsFlash(flashBB);
PSRAMAggregateDevice psramBB(PIN_PSRAM_MISO, PIN_PSRAM_MOSI, PIN_PSRAM_SCK, PER_CHIP_CAP_BYTES);
PSRAMSimpleFS_Multi fsPSRAM(psramBB, AGG_CAPACITY_BYTES);
static bool g_dev_mode = true;
// Sequence for CoProc protocol
static uint32_t g_coproc_seq = 1;  // Global sequence for protocol
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
// ========== Blob registry ==========
struct BlobReg {
  const char* id;
  const uint8_t* data;
  unsigned int len;
};
static const BlobReg g_blobs[] = {
  { FILE_RET42, blob_ret42, blob_ret42_len },
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
static bool loadFileToExecBuf(const char* fname, void*& rawOut, uint8_t*& alignedBuf, uint32_t& szOut);  // Forward
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
  unsigned long execTime = millis();
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
  allOk &= ensureBlobIfMissing(FILE_PWMC2350, blob_pwmc2350, blob_pwmc2350_len);
  allOk &= ensureBlobIfMissing(FILE_RETMIN, blob_retmin, blob_retmin_len);
  allOk &= ensureBlobIfMissing(FILE_BLINKSCRIPT, blob_blinkscript, blob_blinkscript_len);
  allOk &= ensureBlobIfMissing(FILE_ONSCRIPT, blob_onscript, blob_onscript_len);
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
      if (true) {
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
// ========== Co-Processor RPC over Software Serial ==========
static inline void coprocLinkBegin() {
  coprocLink.begin(COPROC_BAUD);
  coprocLink.listen();
}
static bool linkReadByte(uint8_t& b, uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) <= timeoutMs) {
    if (coprocLink.available() > 0) {
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
static bool linkReadExact(uint8_t* dst, size_t n, uint32_t timeoutPerByteMs) {
  for (size_t i = 0; i < n; ++i) {
    if (!linkReadByte(dst[i], timeoutPerByteMs)) return false;
  }
  return true;
}
static bool linkWriteAll(const uint8_t* src, size_t n, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t off = 0;
  while (off < n) {
    size_t wrote = coprocLink.write(src + off, n - off);
    if (wrote > 0) {
      off += wrote;
      continue;
    }
    if ((millis() - start) > timeoutMs) return false;
    yield();
  }
  coprocLink.flush();
  return true;
}
static bool readResponseHeaderSerial(CoProc::Frame& rh, uint32_t overallTimeoutMs = 5000) {
  // Scan for MAGIC bytes to re-synchronize
  const uint8_t magicBytes[4] = {
    (uint8_t)('C'), (uint8_t)('P'), (uint8_t)('R'), (uint8_t)('0')
  };
  uint8_t w[4] = { 0, 0, 0, 0 };
  uint32_t start = millis();
  while ((millis() - start) <= overallTimeoutMs) {
    uint8_t b;
    if (!linkReadByte(b, 50)) {
      yield();
      continue;
    }
    w[0] = w[1];
    w[1] = w[2];
    w[2] = w[3];
    w[3] = b;
    if (w[0] == magicBytes[0] && w[1] == magicBytes[1] && w[2] == magicBytes[2] && w[3] == magicBytes[3]) {
      // Got magic; read rest of header
      union {
        CoProc::Frame f;
        uint8_t bytes[sizeof(CoProc::Frame)];
      } u;
      memcpy(u.bytes, w, 4);
      if (!linkReadExact(u.bytes + 4, sizeof(CoProc::Frame) - 4, 200)) return false;
      rh = u.f;
      return true;
    }
  }
  return false;
}
static bool coprocRequestSerial(uint16_t cmd,
                                const uint8_t* payload, uint32_t len,
                                CoProc::Frame& respHdr,
                                uint8_t* respBuf, uint32_t respCap, uint32_t& respLen,
                                int32_t* statusOut = nullptr) {
  // Build request frame
  CoProc::Frame req{};
  req.magic = CoProc::MAGIC;
  req.version = CoProc::VERSION;
  req.cmd = cmd;
  req.seq = g_coproc_seq++;
  req.len = len;
  req.crc32 = (len ? CoProc::crc32_ieee(payload, len) : 0);
  // Send header + payload
  if (!linkWriteAll(reinterpret_cast<const uint8_t*>(&req), sizeof(req), 2000)) {
    Console.println("coproc(serial): write header failed");
    return false;
  }
  if (len && payload) {
    if (!linkWriteAll(payload, len, 10000)) {
      Console.println("coproc(serial): write payload failed");
      return false;
    }
  }
  // Read response header (scan for MAGIC)
  memset(&respHdr, 0, sizeof(respHdr));
  if (!readResponseHeaderSerial(respHdr, 10000)) {
    Console.println("coproc(serial): read resp header timeout");
    return false;
  }
  // Validate header
  if (respHdr.magic != CoProc::MAGIC || respHdr.version != CoProc::VERSION) {
    Console.println("coproc(serial): bad resp header (magic/version)");
    return false;
  }
  if (respHdr.cmd != (uint16_t)(cmd | 0x80)) {
    Console.printf("coproc(serial): bad resp cmd 0x%02X (expected 0x%02X)\n",
                   (unsigned)respHdr.cmd, (unsigned)(cmd | 0x80));
    return false;
  }
  // Read response payload (if any)
  respLen = 0;
  if (respHdr.len) {
    if (respHdr.len > respCap) {
      Console.printf("coproc(serial): resp too large %u > cap %u\n",
                     (unsigned)respHdr.len, (unsigned)respCap);
      return false;
    }
    if (!linkReadExact(respBuf, respHdr.len, 200)) {
      Console.println("coproc(serial): resp payload timeout");
      return false;
    }
    respLen = respHdr.len;
    // CRC check
    if (respHdr.crc32) {
      uint32_t c = CoProc::crc32_ieee(respBuf, respLen);
      if (c != respHdr.crc32) {
        Console.printf("coproc(serial): resp CRC mismatch calc=0x%08X hdr=0x%08X\n",
                       (unsigned)c, (unsigned)respHdr.crc32);
        return false;
      }
    }
  }
  // Parse status (first int32 in payload for most commands)
  if (statusOut) {
    if (respLen >= 4) {
      int32_t st = 0;
      memcpy(&st, respBuf, 4);
      *statusOut = st;
    } else {
      *statusOut = (int32_t)CoProc::ST_OK;
    }
  }
  return true;
}
// Convenience wrappers
static bool coprocHello() {
  CoProc::Frame rh;
  uint8_t buf[16];
  uint32_t rl = 0;
  int32_t st = 0;
  if (!coprocRequestSerial(CoProc::CMD_HELLO, nullptr, 0, rh, buf, sizeof(buf), rl, &st)) return false;
  if (st != CoProc::ST_OK || rl < 12) return false;
  int32_t version = 0, features = 0;
  memcpy(&version, buf + 4, 4);
  memcpy(&features, buf + 8, 4);
  Console.printf("CoProc HELLO: version=%d features=0x%08X\n", version, (unsigned)features);
  return true;
}
static bool coprocInfo() {
  CoProc::Frame rh;
  uint8_t buf[32];
  uint32_t rl = 0;
  int32_t st = 0;
  if (!coprocRequestSerial(CoProc::CMD_INFO, nullptr, 0, rh, buf, sizeof(buf), rl, &st)) return false;
  if (st != CoProc::ST_OK || rl < (int32_t)(4 + sizeof(CoProc::Info))) return false;
  CoProc::Info inf{};
  memcpy(&inf, buf + 4, sizeof(inf));
  Console.printf("CoProc INFO: flags=0x%08X blob_len=%u mailbox_max=%u\n",
                 (unsigned)inf.impl_flags, (unsigned)inf.blob_len, (unsigned)inf.mailbox_max);
  return true;
}
// Stream a buffer into the co-processor (LOAD_BEGIN/DATA/END) with rolling CRC (seed 0)
static bool coprocLoadBuffer(const uint8_t* data, uint32_t len) {
  // LOAD_BEGIN
  uint8_t b4[4];
  memcpy(b4, &len, 4);
  CoProc::Frame rh;
  uint8_t rbuf[8];
  uint32_t rl = 0;
  int32_t st = 0;
  if (!coprocRequestSerial(CoProc::CMD_LOAD_BEGIN, b4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
  if (st != CoProc::ST_OK) {
    Console.printf("LOAD_BEGIN failed st=%d\n", st);
    return false;
  }
  // LOAD_DATA chunks + rolling CRC (seed 0)
  const uint32_t CHUNK = 512;
  uint32_t sent = 0;
  uint32_t rollingCrc = 0;  // seed 0
  while (sent < len) {
    uint32_t n = (len - sent > CHUNK) ? CHUNK : (len - sent);
    if (!coprocRequestSerial(CoProc::CMD_LOAD_DATA, data + sent, n, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
    if (st != CoProc::ST_OK && st != CoProc::ST_SIZE) {
      Console.printf("LOAD_DATA st=%d\n", st);
      return false;
    }
    rollingCrc = CoProc::crc32_ieee(data + sent, n, rollingCrc);
    sent += n;
  }
  // LOAD_END with the same rollingCrc (no extra xor here; matches device)
  uint8_t crc4[4];
  memcpy(crc4, &rollingCrc, 4);
  rl = 0;
  if (!coprocRequestSerial(CoProc::CMD_LOAD_END, crc4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
  if (st != CoProc::ST_OK) {
    Console.printf("LOAD_END failed st=%d\n", st);
    return false;
  }
  Console.printf("CoProc LOAD OK (%u bytes)\n", (unsigned)len);
  return true;
}
// Load from file in active FS by name
static bool coprocLoadFile(const char* fname) {
  if (!activeFs.exists(fname)) {
    Console.println("coproc: file not found");
    return false;
  }
  uint32_t size = 0;
  if (!activeFs.getFileSize(fname, size) || size == 0) {
    Console.println("coproc: empty file");
    return false;
  }
  if (size & 1u) {
    Console.println("coproc: odd-sized blob (Thumb needs even)");
    return false;
  }
  // Begin
  uint8_t b4[4];
  memcpy(b4, &size, 4);
  CoProc::Frame rh;
  uint8_t rbuf[8];
  uint32_t rl = 0;
  int32_t st = 0;
  if (!coprocRequestSerial(CoProc::CMD_LOAD_BEGIN, b4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
  if (st != CoProc::ST_OK) {
    Console.printf("LOAD_BEGIN failed st=%d\n", st);
    return false;
  }
  // Stream chunks and compute rolling CRC (seed 0)
  const uint32_t CHUNK = 512;
  uint8_t buf[CHUNK];
  uint32_t sent = 0, offset = 0;
  uint32_t rollingCrc = 0;  // seed 0
  while (offset < size) {
    uint32_t n = (size - offset > CHUNK) ? CHUNK : (size - offset);
    uint32_t got = activeFs.readFileRange(fname, offset, buf, n);
    if (got != n) {
      Console.println("coproc: read error");
      return false;
    }
    rl = 0;
    if (!coprocRequestSerial(CoProc::CMD_LOAD_DATA, buf, n, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
    if (st != CoProc::ST_OK && st != CoProc::ST_SIZE) {
      Console.printf("LOAD_DATA st=%d\n", st);
      return false;
    }
    rollingCrc = CoProc::crc32_ieee(buf, n, rollingCrc);
    offset += n;
    sent += n;
  }
  // End
  uint8_t crc4[4];
  memcpy(crc4, &rollingCrc, 4);
  rl = 0;
  if (!coprocRequestSerial(CoProc::CMD_LOAD_END, crc4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
  if (st != CoProc::ST_OK) {
    Console.printf("LOAD_END failed st=%d\n", st);
    return false;
  }
  Console.printf("CoProc LOAD OK (%u bytes)\n", (unsigned)size);
  return true;
}
// Execute with args; timeout taken from main device's override (getTimeout())
static bool coprocExec(const int32_t* argv, uint32_t argc) {
  if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
  uint32_t timeoutMs = getTimeout(100000);  // use master’s configured timeout (0 => default 100000 here)
  uint8_t payload[4 + MAX_EXEC_ARGS * 4 + 4];
  size_t off = 0;
  // argc
  memcpy(payload + off, &argc, 4);
  off += 4;
  // argv
  for (uint32_t i = 0; i < argc; ++i) {
    memcpy(payload + off, &argv[i], 4);
    off += 4;
  }
  // timeout_ms (from master)
  memcpy(payload + off, &timeoutMs, 4);
  off += 4;
  CoProc::Frame rh;
  uint8_t rbuf[16];
  uint32_t rl = 0;
  int32_t st = 0;
  if (!coprocRequestSerial(CoProc::CMD_EXEC, payload, (uint32_t)off, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
  if (rl < 8) {
    Console.println("coproc: short EXEC response");
    return false;
  }
  int32_t retCode = 0;
  memcpy(&st, rbuf + 0, 4);
  memcpy(&retCode, rbuf + 4, 4);
  Console.printf("CoProc EXEC: status=%d return=%d\n", (int)st, (int)retCode);
  return true;
}
// New: send CMD_EXEC but with ASCII args mode when tokens indicate (0xFFFF FFFF marker)
static bool coprocExecTokens(const char* tokens[], uint32_t argc) {
  if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
  uint32_t timeoutMs = getTimeout(100000);
  // decide whether to use ASCII-arg alternate encoding
  bool needAscii = false;
  for (uint32_t i = 0; i < argc; ++i) {
    const char* s = tokens[i];
    if (!s || !*s) {
      needAscii = true;
      break;
    }
    // if starts with 0x or contains non-digit (besides +-) -> ascii
    if ((s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))) {
      needAscii = true;
      break;
    }
    for (const char* p = s; *p; ++p) {
      char c = *p;
      if ((c >= '0' && c <= '9') || c == '+' || c == '-') continue;
      needAscii = true;
      break;
    }
    if (needAscii) break;
  }
  if (!needAscii) {
    // fallback to classic binary ints
    int32_t argv[MAX_EXEC_ARGS];
    for (uint32_t i = 0; i < argc; ++i) argv[i] = (int32_t)strtol(tokens[i], nullptr, 0);
    return coprocExec(argv, argc);
  }
  // Build ASCII-mode payload: [argc:uint32][marker:0xFFFFFFFF][for each: slen:uint32 + bytes][timeout:uint32]
  // Compute total size
  uint32_t total = 4 + 4 + 4 + 4;  // argc + marker + (at least one arg) + timeout; we'll recompute precisely
  total = 4 + 4;                   // argc + marker
  uint32_t sum = 0;
  for (uint32_t i = 0; i < argc; ++i) {
    uint32_t sl = (uint32_t)strlen(tokens[i]);
    sum += 4 + sl;  // slen + bytes
  }
  total += sum + 4;  // + timeout
  if (total > 8192) {
    Console.println("coproc exec: args too large");
    return false;
  }
  uint8_t* payload = (uint8_t*)malloc(total);
  if (!payload) {
    Console.println("coproc exec: malloc failed");
    return false;
  }
  size_t off = 0;
  memcpy(payload + off, &argc, 4);
  off += 4;
  uint32_t marker = 0xFFFFFFFFu;
  memcpy(payload + off, &marker, 4);
  off += 4;
  for (uint32_t i = 0; i < argc; ++i) {
    uint32_t sl = (uint32_t)strlen(tokens[i]);
    memcpy(payload + off, &sl, 4);
    off += 4;
    memcpy(payload + off, tokens[i], sl);
    off += sl;
  }
  memcpy(payload + off, &timeoutMs, 4);
  off += 4;
  CoProc::Frame rh;
  uint8_t rbuf[16];
  uint32_t rl = 0;
  int32_t st = 0;
  bool ok = coprocRequestSerial(CoProc::CMD_EXEC, payload, (uint32_t)off, rh, rbuf, sizeof(rbuf), rl, &st);
  free(payload);
  if (!ok) return false;
  if (rl < 8) {
    Console.println("coproc: short EXEC response");
    return false;
  }
  int32_t retCode = 0;
  memcpy(&st, rbuf + 0, 4);
  memcpy(&retCode, rbuf + 4, 4);
  Console.printf("CoProc EXEC: status=%d return=%d\n", (int)st, (int)retCode);
  return true;
}
// Load a blob file from active FS onto the co-processor, then EXEC with args.
// Uses master-side timeout (getTimeout()).
static bool coprocExecFile(const char* fname, const int32_t* argv, uint32_t argc) {
  if (!fname || !*fname) {
    Console.println("coproc exec: missing file name");
    return false;
  }
  // Stream the file to the co-processor
  if (!coprocLoadFile(fname)) {
    Console.println("coproc exec: LOAD_* failed");
    return false;
  }
  // Execute with provided args and master timeout
  if (!coprocExec(argv, argc)) {
    Console.println("coproc exec: EXEC failed");
    return false;
  }
  return true;
}
static bool coprocStatus() {
  CoProc::Frame rh;
  uint8_t buf[8];
  uint32_t rl = 0;
  int32_t st = 0;
  if (!coprocRequestSerial(CoProc::CMD_STATUS, nullptr, 0, rh, buf, sizeof(buf), rl, &st)) return false;
  if (st != CoProc::ST_OK || rl < 8) return false;
  uint32_t es = 0;
  memcpy(&es, buf + 4, 4);
  Console.printf("CoProc STATUS: exec_state=%u\n", (unsigned)es);
  return true;
}
static bool coprocCancel() {
  CoProc::Frame rh;
  uint8_t buf[4];
  uint32_t rl = 0;
  int32_t st = 0;
  if (!coprocRequestSerial(CoProc::CMD_CANCEL, nullptr, 0, rh, buf, sizeof(buf), rl, &st)) return false;
  Console.printf("CoProc CANCEL: status=%d\n", (int)st);
  return true;
}
static bool coprocMailboxRead(uint32_t maxBytes) {
  uint8_t in[4];
  memcpy(in, &maxBytes, 4);
  CoProc::Frame rh;
  uint8_t out[512];
  uint32_t rl = 0;
  int32_t st = 0;
  if (!coprocRequestSerial(CoProc::CMD_MAILBOX_RD, in, 4, rh, out, sizeof(out), rl, &st)) return false;
  if (st != CoProc::ST_OK || rl < 8) return false;
  uint32_t n = 0;
  memcpy(&n, out + 4, 4);
  Console.print("CoProc MAILBOX: ");
  for (uint32_t i = 0; i < n; ++i) Console.print((char)out[8 + i]);
  Console.println();
  return true;
}
static bool coprocReset() {
  CoProc::Frame rh;
  uint8_t buf[4];
  uint32_t rl = 0;
  int32_t st = 0;
  bool ok = coprocRequestSerial(CoProc::CMD_RESET, nullptr, 0, rh, buf, sizeof(buf), rl, &st);
  Console.println("CoProc RESET issued");
  return ok;
}
// ========== Co-Processor Script RPC over Software Serial ==========
// Load a script file (text) to the co-processor (SCRIPT_* pipeline)
static bool coprocScriptLoadFile(const char* fname) {
  if (!activeFs.exists(fname)) {
    Console.println("coproc script: file not found");
    return false;
  }
  uint32_t size = 0;
  if (!activeFs.getFileSize(fname, size) || size == 0) {
    Console.println("coproc script: empty file");
    return false;
  }
  // BEGIN
  uint8_t b4[4];
  memcpy(b4, &size, 4);
  CoProc::Frame rh;
  uint8_t rbuf[8];
  uint32_t rl = 0;
  int32_t st = 0;
  if (!coprocRequestSerial(CoProc::CMD_SCRIPT_BEGIN, b4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
  if (st != CoProc::ST_OK) {
    Console.printf("SCRIPT_BEGIN failed st=%d\n", st);
    return false;
  }
  // DATA chunks + rolling CRC (seed 0)
  const uint32_t CHUNK = 512;
  uint8_t buf[CHUNK];
  uint32_t offset = 0;
  uint32_t rollingCrc = 0;
  while (offset < size) {
    uint32_t n = (size - offset > CHUNK) ? CHUNK : (size - offset);
    uint32_t got = activeFs.readFileRange(fname, offset, buf, n);
    if (got != n) {
      Console.println("coproc script: read error");
      return false;
    }
    rl = 0;
    if (!coprocRequestSerial(CoProc::CMD_SCRIPT_DATA, buf, n, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
    if (st != CoProc::ST_OK && st != CoProc::ST_SIZE) {
      Console.printf("SCRIPT_DATA st=%d\n", st);
      return false;
    }
    rollingCrc = CoProc::crc32_ieee(buf, n, rollingCrc);
    offset += n;
  }
  // END (send rolling CRC)
  uint8_t crc4[4];
  memcpy(crc4, &rollingCrc, 4);
  rl = 0;
  if (!coprocRequestSerial(CoProc::CMD_SCRIPT_END, crc4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
  if (st != CoProc::ST_OK) {
    Console.printf("SCRIPT_END failed st=%d\n", st);
    return false;
  }
  Console.printf("CoProc SCRIPT LOAD OK (%u bytes)\n", (unsigned)size);
  return true;
}
// Execute currently loaded script with args (timeout from master setting)
static bool coprocScriptExec(const int32_t* argv, uint32_t argc) {
  if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
  uint32_t timeoutMs = getTimeout(100000);
  uint8_t payload[4 + MAX_EXEC_ARGS * 4 + 4];
  size_t off = 0;
  memcpy(payload + off, &argc, 4);
  off += 4;
  for (uint32_t i = 0; i < argc; ++i) {
    memcpy(payload + off, &argv[i], 4);
    off += 4;
  }
  memcpy(payload + off, &timeoutMs, 4);
  off += 4;
  CoProc::Frame rh;
  uint8_t rbuf[16];
  uint32_t rl = 0;
  int32_t st = 0;
  if (!coprocRequestSerial(CoProc::CMD_SCRIPT_EXEC, payload, (uint32_t)off, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
  if (rl < 8) {
    Console.println("coproc: short SCRIPT_EXEC response");
    return false;
  }
  int32_t retCode = 0;
  memcpy(&st, rbuf + 0, 4);
  memcpy(&retCode, rbuf + 4, 4);
  Console.printf("CoProc SCRIPT EXEC: status=%d return=%d\n", (int)st, (int)retCode);
  return true;
}
// New: send SCRIPT_EXEC with token strings (auto-choose ASCII mode if tokens indicate)
static bool coprocScriptExecTokens(const char* tokens[], uint32_t argc) {
  if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
  uint32_t timeoutMs = getTimeout(100000);
  // decide whether to use ASCII-arg alternate encoding
  bool needAscii = false;
  for (uint32_t i = 0; i < argc; ++i) {
    const char* s = tokens[i];
    if (!s || !*s) {
      needAscii = true;
      break;
    }
    // if starts with 0x or contains non-digit (besides +-) -> ascii
    if ((s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))) {
      needAscii = true;
      break;
    }
    for (const char* p = s; *p; ++p) {
      char c = *p;
      if ((c >= '0' && c <= '9') || c == '+' || c == '-') continue;
      needAscii = true;
      break;
    }
    if (needAscii) break;
  }
  if (!needAscii) {
    // fallback to classic binary ints
    int32_t argv[MAX_EXEC_ARGS];
    for (uint32_t i = 0; i < argc; ++i) argv[i] = (int32_t)strtol(tokens[i], nullptr, 0);
    return coprocScriptExec(argv, argc);
  }
  // Build ASCII-mode payload: [argc:uint32][marker:0xFFFFFFFF][for each: slen:uint32 + bytes][timeout:uint32]
  uint32_t total = 4 + 4;  // argc + marker
  for (uint32_t i = 0; i < argc; ++i) {
    uint32_t sl = (uint32_t)strlen(tokens[i]);
    total += 4 + sl;
  }
  total += 4;  // timeout
  if (total > 8192) {
    Console.println("coproc script exec: args too large");
    return false;
  }
  uint8_t* payload = (uint8_t*)malloc(total);
  if (!payload) {
    Console.println("coproc script exec: malloc failed");
    return false;
  }
  size_t off = 0;
  memcpy(payload + off, &argc, 4);
  off += 4;
  uint32_t marker = 0xFFFFFFFFu;
  memcpy(payload + off, &marker, 4);
  off += 4;
  for (uint32_t i = 0; i < argc; ++i) {
    uint32_t sl = (uint32_t)strlen(tokens[i]);
    memcpy(payload + off, &sl, 4);
    off += 4;
    memcpy(payload + off, tokens[i], sl);
    off += sl;
  }
  memcpy(payload + off, &timeoutMs, 4);
  off += 4;
  CoProc::Frame rh;
  uint8_t rbuf[16];
  uint32_t rl = 0;
  int32_t st = 0;
  bool ok = coprocRequestSerial(CoProc::CMD_SCRIPT_EXEC, payload, (uint32_t)off, rh, rbuf, sizeof(rbuf), rl, &st);
  free(payload);
  if (!ok) return false;
  if (rl < 8) {
    Console.println("coproc: short SCRIPT_EXEC response");
    return false;
  }
  int32_t retCode = 0;
  memcpy(&st, rbuf + 0, 4);
  memcpy(&retCode, rbuf + 4, 4);
  Console.printf("CoProc SCRIPT EXEC: status=%d return=%d\n", (int)st, (int)retCode);
  return true;
}
// One-shot: load script file and exec with args (string tokens)
static bool coprocScriptExecFileTokens(const char* fname, const char* tokens[], uint32_t argc) {
  if (!coprocScriptLoadFile(fname)) return false;
  return coprocScriptExecTokens(tokens, argc);
}
// One-shot: load script file and exec with int args (backward compat)
static bool coprocScriptExecFile(const char* fname, const int32_t* argv, uint32_t argc) {
  if (!coprocScriptLoadFile(fname)) return false;
  return coprocScriptExec(argv, argc);
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
  Console.println("Co-Processor (serial RPC) commands:");
  Console.println("  coproc ping                  - HELLO (version/features)");
  Console.println("  coproc info                  - INFO (flags, blob_len, mailbox_max)");
  Console.println("  coproc exec <file> [a0..aN]  - Load <file> from active FS to co-proc and EXEC with args");
  Console.println("                                 Timeout is taken from 'timeout' setting on the master");
  Console.println("  coproc sexec <file> [a0..aN] - Load a script file to co-proc and execute once");
  Console.println("                                 Uses master 'timeout' setting; script may use DELAY/DELAY_US, PINMODE, DWRITE, etc.");
  Console.println("  coproc status                - STATUS (exec_state)");
  Console.println("  coproc mbox [n]              - MAILBOX_RD (read up to n bytes; default 128)");
  Console.println("  coproc cancel                - CANCEL (sets cancel flag)");
  Console.println("  coproc reset                 - RESET (remote reboot)");
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
#if 0
        if (sub && strcmp(sub, "force") == 0) {
          Console.println("Force kill not implemented in this build.");
        }
#endif
        return;
      }
      g_bg_cancel_requested = true;
      Console.println("Cancellation requested (best-effort).");
    } else {
      Console.println("bg: usage: bg [status|query] | bg kill|cancel [force]");
    }
  } else if (!strcmp(t0, "coproc")) {
    char* sub;
    if (!nextToken(p, sub)) {
      Console.println("coproc cmds: ping|info|exec <file> [a0..aN]|status|mbox [n]|cancel|reset");
      return;
    }
    if (!strcmp(sub, "ping")) {
      if (!coprocHello()) Console.println("coproc ping failed");
    } else if (!strcmp(sub, "info")) {
      if (!coprocInfo()) Console.println("coproc info failed");
    } else if (!strcmp(sub, "sexec")) {
      // Syntax: coproc sexec <file> [a0..aN]
      char* fname = nullptr;
      if (!nextToken(p, fname)) {
        Console.println("usage: coproc sexec <file> [a0..aN]");
        return;
      }
      // Collect raw token strings (do not convert to int here)
      const char* tokens[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) {
        tokens[argc++] = tok;
      }
      if (!coprocScriptLoadFile(fname)) {
        Console.println("coproc sexec failed: load failed");
        return;
      }
      // Use tokens-aware send (will choose ASCII mode if needed)
      if (!coprocScriptExecTokens(tokens, argc)) Console.println("coproc sexec failed");
    } else if (!strcmp(sub, "exec")) {
      // Syntax: coproc exec <file> [a0..aN]
      char* fname = nullptr;
      if (!nextToken(p, fname)) {
        Console.println("usage: coproc exec <file> [a0..aN]");
        return;
      }
      const char* tokens[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) {
        tokens[argc++] = tok;
      }
      // send as tokens-aware (will auto choose ascii vs binary)
      if (!coprocLoadFile(fname)) {
        Console.println("coproc exec failed: load failed");
        return;
      }
      if (!coprocExecTokens(tokens, argc)) Console.println("coproc exec failed");
    } else if (!strcmp(sub, "status")) {
      if (!coprocStatus()) Console.println("coproc status failed");
    } else if (!strcmp(sub, "mbox")) {
      char* nstr;
      uint32_t n = 128;
      if (nextToken(p, nstr)) n = (uint32_t)strtoul(nstr, nullptr, 0);
      if (!coprocMailboxRead(n)) Console.println("coproc mbox failed");
    } else if (!strcmp(sub, "cancel")) {
      if (!coprocCancel()) Console.println("coproc cancel failed");
    } else if (!strcmp(sub, "reset")) {
      if (!coprocReset()) Console.println("coproc reset failed (transport)");
    } else {
      Console.println("usage: coproc ping|info|exec <file> [a0..aN]|status|mbox [n]|cancel|reset");
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
  // Avoid configuring QPI IO2/IO3 lines unless wired
  // if (PSRAM_ENABLE_QPI) {
  //   psramBB.setExtraDataPins(PIN_PSRAM_IO2, PIN_PSRAM_IO3);
  //   psramBB.setModeQuad(true);
  // }
  bindActiveFs(g_storage);
  bool mounted = (g_storage == StorageBackend::Flash) ? activeFs.mount(true) : activeFs.mount(false);
  if (!mounted) { Console.println("FS mount failed on active storage"); }
  // Clear mailbox
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) BLOB_MAILBOX[i] = 0;
  // Initialize co-processor software serial link
  coprocLinkBegin();
  Console.printf("Co-Proc serial link ready @ %u bps (RX=GP%u, TX=GP%u)\n",
                 (unsigned)COPROC_BAUD, (unsigned)PIN_COPROC_RX, (unsigned)PIN_COPROC_TX);
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