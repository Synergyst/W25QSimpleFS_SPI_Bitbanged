// RP2040 + Winbond W25Q128, bit-banged SPI, simple FS + blob loader + console + autogen + core1 exec via mailbox.
//
// Pins:
//   Pin 0 -> MISO
//   Pin 1 -> CS
//   Pin 2 -> SCK
//   Pin 3 -> MOSI
#include "pico/stdlib.h"  // for tight_loop_contents()
#include "W25QBitbang.h"
#include "W25QSimpleFS.h"

// Include your generated blob headers
#include "blob_add2.h"
#include "blob_muladd_a.h"
#include "blob_muladd_b.h"
#include "blob_ret42.h"
#include "blob_square.h"
#include "blob_gpio8blink.h"
#include "blob_gpio_blink1s.h"
#include "blob_gpio_blink_ms.h"
#include "blob_gpio_pwm.h"
#include "blob_gpio_pwm_cycles.h"

// ================== Autowrite configuration ==================
// Toggle which blobs should be auto-written if missing (0=off, 1=on)
#define AUTO_BLOB_RET42 1
#define AUTO_BLOB_ADD2 1
#define AUTO_BLOB_SQUARE 1
#define AUTO_BLOB_MULA 1
#define AUTO_BLOB_MULB 1
#define AUTO_BLOB_GPIO8BLINK 1
#define AUTO_BLOB_GPIO1S 1
#define AUTO_BLOB_GPIOMS 1
#define AUTO_BLOB_GPIOPWM 1
#define AUTO_BLOB_GPIOPWMC 1

// Optional: run autogenBlobWrites() automatically in setup()
#define AUTO_RUN_AUTOGEN_ON_BOOT 1

// Target filenames (must be <= 32 chars; FS name limit)
#define FILE_RET42 "code_ret42.bin"
#define FILE_ADD2 "code_add2.bin"
#define FILE_SQUARE "code_square.bin"
#define FILE_MULA "mulA.bin"
#define FILE_MULB "mulB.bin"
#define FILE_GPIO8BLINK "gpio8.bin"
#define FILE_GPIO1S "gpio1s.bin"
#define FILE_GPIOMS "gpioms.bin"
#define FILE_GPIOPWM "pwm.bin"
#define FILE_GPIOPWMC "pwmc.bin"

// Slot reserve size for newly created blob files (bytes, multiple of 4096 recommended)
#define DEFAULT_SLOT_RESERVE 4096
// =============================================================
const uint8_t PIN_MISO = 0;
const uint8_t PIN_CS = 1;
const uint8_t PIN_SCK = 2;
const uint8_t PIN_MOSI = 3;

W25QBitbang flash(PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI);
W25QSimpleFS fs(flash);

// ================== Blob registry ==================
struct BlobReg {
  const char* id;       // console id (e.g., "ret42")
  const uint8_t* data;  // array from header
  unsigned int len;     // length from header
};
static const BlobReg g_blobs[] = {
  { "ret42", blob_ret42, blob_ret42_len },
  { "add2", blob_add2, blob_add2_len },
  { "square", blob_square, blob_square_len },
  { "muladd_a", blob_muladd_a, blob_muladd_a_len },
  { "muladd_b", blob_muladd_b, blob_muladd_b_len },
  { "gpio8blink", blob_gpio8blink, blob_gpio8blink_len },
  { "gpio_blink1s", blob_gpio_blink1s, blob_gpio_blink1s_len },
  { "gpio_blink_ms", blob_gpio_blink_ms, blob_gpio_blink_ms_len },
  { "gpio_pwm", blob_gpio_pwm, blob_gpio_pwm_len },
  { "gpio_pwm_cycles", blob_gpio_pwm_cycles, blob_gpio_pwm_cycles_len }
};
static const size_t g_blobs_count = sizeof(g_blobs) / sizeof(g_blobs[0]);

static void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

// FS name limit (must match W25QSimpleFS::MAX_NAME = 32)
static const size_t FS_NAME_MAX = 32;

// Timeout override (0 = use per-call defaults)
static volatile uint32_t g_timeout_override_ms = 0;
static inline uint32_t getTimeout(uint32_t defMs) {
  return g_timeout_override_ms ? g_timeout_override_ms : defMs;
}

// ================== Core1 execution plumbing (mailbox) ==================
enum ExecType : uint8_t {
  EX0 = 0,  // int(void)
  EX1 = 1,  // int(int)
  EX2 = 2,  // int(int,int)
  EX3 = 3,  // int(int,int,int)
  EX4 = 4   // int(int,int,int,int)
};
typedef int (*fn0_t)(void);
typedef int (*fn1_t)(int);
typedef int (*fn2_t)(int, int);
typedef int (*fn3_t)(int, int, int);
typedef int (*fn4_t)(int, int, int, int);
struct ExecJob {
  uint8_t type;  // ExecType
  uint8_t rsv0;
  uint8_t rsv1;
  uint8_t rsv2;
  uintptr_t code;  // raw aligned code address (LSB 0)
  uint32_t size;   // code size (bytes, even)
  int32_t a;
  int32_t b;
  int32_t c;
  int32_t d;
};
// Shared mailbox
static volatile ExecJob g_job;
static volatile int32_t g_result = 0;
static volatile int32_t g_status = 0;
// Job flag: 0=idle, 1=ready, 2=running, 3=done
static volatile uint32_t g_job_flag = 0;

// Core1 worker poll (called from loop1())
static void core1WorkerPoll() {
  // Fast path: nothing to do
  if (g_job_flag != 1u) return;
  // Acquire: ensure we see the published job
  __asm__ volatile("dsb 0" ::
                     : "memory");
  __asm__ volatile("isb 0" ::
                     : "memory");
  // Mark running
  g_job_flag = 2u;
  // Snapshot job locally
  ExecJob job;
  job.type = g_job.type;
  job.code = g_job.code;
  job.size = g_job.size;
  job.a = g_job.a;
  job.b = g_job.b;
  job.c = g_job.c;
  job.d = g_job.d;
  int32_t rv = 0;
  int32_t st = 0;
  if (job.code == 0 || (job.size & 1u)) {
    st = -1;  // invalid entry
  } else {
    void* entryThumb = (void*)(job.code | 1u);
    switch (job.type) {
      case EX0:
        {
          fn0_t fn = (fn0_t)entryThumb;
          rv = fn();
        }
        break;
      case EX1:
        {
          fn1_t fn = (fn1_t)entryThumb;
          rv = fn(job.a);
        }
        break;
      case EX2:
        {
          fn2_t fn = (fn2_t)entryThumb;
          rv = fn(job.a, job.b);
        }
        break;
      case EX3:
        {
          fn3_t fn = (fn3_t)entryThumb;
          rv = fn(job.a, job.b, job.c);
        }
        break;
      case EX4:
        {
          fn4_t fn = (fn4_t)entryThumb;
          rv = fn(job.a, job.b, job.c, job.d);
        }
        break;
      default:
        st = -2;
        break;
    }
  }
  g_result = rv;
  g_status = st;
  // Release: publish results then mark done
  __asm__ volatile("dsb 0" ::
                     : "memory");
  __asm__ volatile("isb 0" ::
                     : "memory");
  g_job_flag = 3u;
}

// Core0 helper: run already-loaded RAM code on core1, return result.
// timeoutMs: how long to wait for completion (default 100ms; override in execBlob* helpers).
static bool runOnCore1(ExecType type, uintptr_t codeAligned, uint32_t sz,
                       int32_t a, int32_t b, int32_t c, int32_t d,
                       int& retVal, uint32_t timeoutMs = 100) {
  // If core1 is busy, bail (simple single-job mailbox)
  if (g_job_flag != 0u) {
    Serial.println("core1 busy");
    return false;
  }
  // Publish job
  g_job.type = (uint8_t)type;
  g_job.code = codeAligned;
  g_job.size = sz;
  g_job.a = a;
  g_job.b = b;
  g_job.c = c;
  g_job.d = d;
  g_status = 0;
  g_result = 0;
  __asm__ volatile("dsb 0" ::
                     : "memory");
  __asm__ volatile("isb 0" ::
                     : "memory");
  // Signal ready
  g_job_flag = 1u;
  // Wait for done or timeout
  uint32_t start = millis();
  while (g_job_flag != 3u) {
    // give core1 cycles
    tight_loop_contents();
    if ((millis() - start) > timeoutMs) {
      Serial.println("core1 timeout");
      // Leave job_flag non-zero so we don't race results; user can retry later.
      return false;
    }
  }
  if (g_status != 0) {
    Serial.print("core1 error status=");
    Serial.println((int)g_status);
    g_job_flag = 0u;
    return false;
  }
  retVal = (int)g_result;
  g_job_flag = 0u;  // reset for next job
  return true;
}

// ================== FS helpers and console ==================
static bool checkNameLen(const char* name) {
  size_t n = strlen(name);
  if (n == 0 || n > FS_NAME_MAX) {
    Serial.print("Error: filename length ");
    Serial.print(n);
    Serial.print(" exceeds max ");
    Serial.print(FS_NAME_MAX);
    Serial.println(". Use a shorter name (e.g., mulA.bin, mulB.bin).");
    return false;
  }
  return true;
}
static void listBlobs() {
  Serial.println("Available blobs:");
  for (size_t i = 0; i < g_blobs_count; ++i) {
    Serial.print(" - ");
    Serial.print(g_blobs[i].id);
    Serial.print(" (");
    Serial.print(g_blobs[i].len);
    Serial.println(" bytes)");
  }
}
static const BlobReg* findBlob(const char* id) {
  for (size_t i = 0; i < g_blobs_count; ++i) {
    if (strcmp(g_blobs[i].id, id) == 0) return &g_blobs[i];
  }
  return nullptr;
}
static void dumpFileHead(const char* fname, uint32_t count) {
  uint32_t sz = 0;
  if (!fs.getFileSize(fname, sz) || sz == 0) {
    Serial.println("dump: missing/empty");
    return;
  }
  if (count > sz) count = sz;
  const size_t CHUNK = 32;
  uint8_t buf[CHUNK];
  uint32_t off = 0;
  Serial.print(fname);
  Serial.print(" size=");
  Serial.println(sz);
  while (off < count) {
    size_t n = (count - off > CHUNK) ? CHUNK : (count - off);
    fs.readFileRange(fname, off, buf, n);
    Serial.print("  ");
    for (size_t i = 0; i < n; ++i) {
      if (i) Serial.print(' ');
      printHexByte(buf[i]);
    }
    Serial.println();
    off += n;
  }
}
// Load file fully into an aligned RAM buffer suitable for execution.
// Returns aligned pointer in outBuf, size in outSize, and raw allocation in rawOut (free(rawOut) when done).
static bool loadFileToExecBuf(const char* fname, void*& rawOut, uint8_t*& alignedBuf, uint32_t& szOut) {
  rawOut = nullptr;
  alignedBuf = nullptr;
  szOut = 0;
  uint32_t sz = 0;
  if (!fs.getFileSize(fname, sz) || sz == 0) {
    Serial.println("load: file missing/empty");
    return false;
  }
  if (sz & 1u) {
    Serial.println("load: odd-sized blob (Thumb requires 16-bit alignment)");
    return false;
  }
  void* raw = malloc(sz + 4);
  if (!raw) {
    Serial.println("load: malloc failed");
    return false;
  }
  uint8_t* buf = (uint8_t*)((((uintptr_t)raw) + 3) & ~((uintptr_t)3));
  if (fs.readFile(fname, buf, sz) != sz) {
    Serial.println("load: read failed");
    free(raw);
    return false;
  }
  rawOut = raw;
  alignedBuf = buf;
  szOut = sz;
  return true;
}
// Create/Update a file with a blob (manual path: create slot if not present, in-place update if possible)
static bool ensureBlobFile(const char* fname, const uint8_t* data, uint32_t len, uint32_t reserve = DEFAULT_SLOT_RESERVE) {
  if (!checkNameLen(fname)) return false;
  if (!fs.exists(fname)) {
    Serial.print("Creating slot ");
    Serial.print(fname);
    Serial.print(" (");
    Serial.print(reserve);
    Serial.println(" bytes)...");
    if (fs.createFileSlot(fname, reserve, data, len)) {
      Serial.println("Created and wrote blob");
      return true;
    } else {
      Serial.println("Failed to create slot");
      return false;
    }
  }
  // Compare existing content
  uint32_t addr, size, cap;
  if (!fs.getFileInfo(fname, addr, size, cap)) {
    Serial.println("getFileInfo failed");
    return false;
  }
  bool same = (size == len);
  if (same) {
    const size_t CHUNK = 64;
    uint8_t buf[CHUNK];
    uint32_t off = 0;
    while (off < size) {
      size_t n = (size - off > CHUNK) ? CHUNK : (size - off);
      fs.readFileRange(fname, off, buf, n);
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
    Serial.println("Blob already up to date");
    return true;
  }
  if (cap >= len && fs.writeFileInPlace(fname, data, len)) {
    Serial.println("Updated in place");
    return true;
  }
  if (fs.writeFile(fname, data, len, W25QSimpleFS::WriteMode::ReplaceIfExists)) {
    Serial.println("Updated by allocating new space");
    return true;
  }
  Serial.println("Failed to update file");
  return false;
}
// Auto-write ONLY IF MISSING (does not update if present)
static bool ensureBlobIfMissing(const char* fname, const uint8_t* data, uint32_t len, uint32_t reserve = DEFAULT_SLOT_RESERVE) {
  if (!checkNameLen(fname)) return false;
  if (fs.exists(fname)) {
    Serial.print(fname);
    Serial.println(" present (skipping)");
    return true;
  }
  Serial.print("Auto-creating ");
  Serial.print(fname);
  Serial.print(" (");
  Serial.print(reserve);
  Serial.println(" bytes)...");
  if (fs.createFileSlot(fname, reserve, data, len)) {
    Serial.println("Created and wrote blob");
    return true;
  }
  Serial.println("Auto-create failed");
  return false;
}
// Call this to auto-create only the blobs you enabled with #defines above
static void autogenBlobWrites() {
  bool allOk = true;
#if AUTO_BLOB_RET42
  allOk &= ensureBlobIfMissing(FILE_RET42, blob_ret42, blob_ret42_len);
#endif
#if AUTO_BLOB_ADD2
  allOk &= ensureBlobIfMissing(FILE_ADD2, blob_add2, blob_add2_len);
#endif
#if AUTO_BLOB_SQUARE
  allOk &= ensureBlobIfMissing(FILE_SQUARE, blob_square, blob_square_len);
#endif
#if AUTO_BLOB_MULA
  allOk &= ensureBlobIfMissing(FILE_MULA, blob_muladd_a, blob_muladd_a_len);
#endif
#if AUTO_BLOB_MULB
  allOk &= ensureBlobIfMissing(FILE_MULB, blob_muladd_b, blob_muladd_b_len);
#endif
#if AUTO_BLOB_GPIO8BLINK
  allOk &= ensureBlobIfMissing(FILE_GPIO8BLINK, blob_gpio8blink, blob_gpio8blink_len);
#endif
#if AUTO_BLOB_GPIO1S
  allOk &= ensureBlobIfMissing(FILE_GPIO1S, blob_gpio_blink1s, blob_gpio_blink1s_len);
#endif
#if AUTO_BLOB_GPIOMS
  allOk &= ensureBlobIfMissing(FILE_GPIOMS, blob_gpio_blink_ms, blob_gpio_blink_ms_len);
#endif
#if AUTO_BLOB_GPIOPWM
  allOk &= ensureBlobIfMissing(FILE_GPIOPWM, blob_gpio_pwm, blob_gpio_pwm_len);
#endif
#if AUTO_BLOB_GPIOPWMC
  allOk &= ensureBlobIfMissing(FILE_GPIOPWMC, blob_gpio_pwm_cycles, blob_gpio_pwm_cycles_len);
#endif
  Serial.print("autogen: ");
  Serial.println(allOk ? "OK" : "some failures");
}

// Core1-based execution helpers (load -> run on core1 -> free)
static bool execBlobNoArg(const char* fname, int& retVal) {
  void* raw = nullptr;
  uint8_t* buf = nullptr;
  uint32_t sz = 0;
  if (!loadFileToExecBuf(fname, raw, buf, sz)) return false;
  Serial.print("Blob head: ");
  for (int i = 0; i < 8 && i < (int)sz; ++i) {
    if (i) Serial.print(' ');
    printHexByte(buf[i]);
  }
  Serial.println();
  uintptr_t code = (uintptr_t)buf;  // raw aligned address (LSB 0)
  Serial.print("Calling entry on core1 at 0x");
  Serial.println((uintptr_t)buf, HEX);
  bool ok = runOnCore1(EX0, code, sz, 0, 0, 0, 0, retVal, getTimeout(100));
  if (!ok) Serial.println("exec0: core1 run failed");
  free(raw);
  return ok;
}
static bool execBlob1Arg(const char* fname, int a, int& retVal) {
  void* raw = nullptr;
  uint8_t* buf = nullptr;
  uint32_t sz = 0;
  if (!loadFileToExecBuf(fname, raw, buf, sz)) return false;
  uintptr_t code = (uintptr_t)buf;
  Serial.print("Calling entry on core1 at 0x");
  Serial.println((uintptr_t)buf, HEX);
  // Raised timeout to 5000ms to allow visible blinks to complete
  bool ok = runOnCore1(EX1, code, sz, a, 0, 0, 0, retVal, getTimeout(5000));
  if (!ok) Serial.println("exec1: core1 run failed");
  free(raw);
  return ok;
}
static bool execBlob2Arg(const char* fname, int a, int b, int& retVal) {
  void* raw = nullptr;
  uint8_t* buf = nullptr;
  uint32_t sz = 0;
  if (!loadFileToExecBuf(fname, raw, buf, sz)) return false;
  uintptr_t code = (uintptr_t)buf;
  Serial.print("Calling entry on core1 at 0x");
  Serial.println((uintptr_t)buf, HEX);
  // Raised timeout to 5000ms to be safer for longer operations
  bool ok = runOnCore1(EX2, code, sz, a, b, 0, 0, retVal, getTimeout(5000));
  if (!ok) Serial.println("exec2: core1 run failed");
  free(raw);
  return ok;
}
static bool execBlob3Arg(const char* fname, int a, int b, int c, int& retVal) {
  void* raw = nullptr;
  uint8_t* buf = nullptr;
  uint32_t sz = 0;
  if (!loadFileToExecBuf(fname, raw, buf, sz)) return false;
  uintptr_t code = (uintptr_t)buf;
  Serial.print("Calling entry on core1 at 0x");
  Serial.println((uintptr_t)buf, HEX);
  // Raised timeout to 10000ms; for gpio_blink_ms this covers typical delays
  bool ok = runOnCore1(EX3, code, sz, a, b, c, 0, retVal, getTimeout(10000));
  if (!ok) Serial.println("exec3: core1 run failed");
  free(raw);
  return ok;
}
static bool execBlob4Arg(const char* fname, int a, int b, int c, int d, int& retVal) {
  void* raw = nullptr;
  uint8_t* buf = nullptr;
  uint32_t sz = 0;
  if (!loadFileToExecBuf(fname, raw, buf, sz)) return false;
  uintptr_t code = (uintptr_t)buf;
  Serial.print("Calling entry on core1 at 0x");
  Serial.println((uintptr_t)buf, HEX);
  // Keep 1000ms here; adjust per use-case
  bool ok = runOnCore1(EX4, code, sz, a, b, c, d, retVal, getTimeout(1000));
  if (!ok) Serial.println("exec4: core1 run failed");
  free(raw);
  return ok;
}

// ---------- Console ----------
static char lineBuf[192];
static int nextToken(char*& p, char*& tok) {
  // Skip spaces/tabs/newlines
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
  }  // null-terminate token
  return 1;
}
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
static void printHelp() {
  Serial.println("Commands (filename max 32 chars):");
  Serial.println("  help                         - this help");
  Serial.println("  blobs                        - list compiled-in blobs");
  Serial.println("  autogen                      - auto-create enabled blobs if missing");
  Serial.println("  files                        - list files in FS");
  Serial.println("  info <file>                  - show file addr/size/cap");
  Serial.println("  dump <file> <nbytes>         - hex dump head of file");
  Serial.println("  mkSlot <file> <reserve>      - create sector-aligned slot");
  Serial.println("  writeblob <file> <blobId>    - create/update file from blob");
  Serial.println("  exec0 <file>                 - execute as int entry(void) on core1");
  Serial.println("  exec1 <file> <a>             - execute as int entry(int) on core1");
  Serial.println("  exec2 <file> <a> <b>         - execute as int entry(int,int) on core1");
  Serial.println("  exec3 <file> <a> <b> <c>     - execute as int entry(int,int,int) on core1");
  Serial.println("  exec4 <file> <a> <b> <c> <d> - execute as int entry(int,int,int,int) on core1");
  Serial.println("  timeout [ms]                 - show or set core1 timeout override (0=defaults)");
  Serial.println();
  Serial.println("Examples:");
  Serial.println("  writeblob " FILE_RET42 " ret42");
  Serial.println("  exec0 " FILE_RET42);
  Serial.println("  writeblob " FILE_ADD2 " add2");
  Serial.println("  exec2 " FILE_ADD2 " 7 5");
  Serial.println("  writeblob " FILE_SQUARE " square");
  Serial.println("  exec2 " FILE_SQUARE " 9 0");
  Serial.println("  writeblob " FILE_MULA " muladd_a");
  Serial.println("  exec2 " FILE_MULA " 3 4");
  Serial.println("  writeblob " FILE_MULB " muladd_b");
  Serial.println("  exec2 " FILE_MULB " 3 4");
  Serial.println("  writeblob " FILE_GPIO8BLINK " gpio8blink");
  Serial.println("  exec0 " FILE_GPIO8BLINK);
  Serial.println("  writeblob " FILE_GPIO1S " gpio_blink1s");
  Serial.println("  exec1 " FILE_GPIO1S " 25");
  Serial.println("  writeblob " FILE_GPIOMS " gpio_blink_ms");
  Serial.println("  exec3 " FILE_GPIOMS " 25 250 750");
  Serial.println("  writeblob " FILE_GPIOPWM " gpio_pwm");
  Serial.println("  exec4 " FILE_GPIOPWM " <pin> <high_ms> <low_ms> <dur_ms>");
  Serial.println("  writeblob " FILE_GPIOPWMC " gpio_pwm_cycles");
  Serial.println("  exec4 " FILE_GPIOPWMC " <pin> <low_ms> <high_ms> <cycles>");
}
static void handleCommand(char* line) {
  char* p = line;
  char* t0;
  if (!nextToken(p, t0)) return;
  if (!strcmp(t0, "help")) {
    printHelp();
  } else if (!strcmp(t0, "reboot")) {
    rp2040.reboot();
  } else if (!strcmp(t0, "blobs")) {
    listBlobs();
  } else if (!strcmp(t0, "autogen")) {
    autogenBlobWrites();
  } else if (!strcmp(t0, "files")) {
    fs.listFilesToSerial();
  } else if (!strcmp(t0, "info")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Serial.println("usage: info <file>");
      return;
    }
    uint32_t a, s, c;
    if (fs.getFileInfo(fn, a, s, c)) {
      Serial.print(fn);
      Serial.print(": addr=0x");
      Serial.print(a, HEX);
      Serial.print(" size=");
      Serial.print(s);
      Serial.print(" cap=");
      Serial.println(c);
    } else Serial.println("not found");
  } else if (!strcmp(t0, "dump")) {
    char* fn;
    char* nstr;
    if (!nextToken(p, fn) || !nextToken(p, nstr)) {
      Serial.println("usage: dump <file> <nbytes>");
      return;
    }
    dumpFileHead(fn, (uint32_t)strtoul(nstr, nullptr, 0));
  } else if (!strcmp(t0, "mkSlot")) {
    char* fn;
    char* nstr;
    if (!nextToken(p, fn) || !nextToken(p, nstr)) {
      Serial.println("usage: mkSlot <file> <reserve>");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint32_t res = (uint32_t)strtoul(nstr, nullptr, 0);
    if (fs.createFileSlot(fn, res, nullptr, 0)) Serial.println("slot created");
    else Serial.println("mkSlot failed");
  } else if (!strcmp(t0, "writeblob")) {
    char* fn;
    char* bid;
    if (!nextToken(p, fn) || !nextToken(p, bid)) {
      Serial.println("usage: writeblob <file> <blobId>");
      return;
    }
    if (!checkNameLen(fn)) return;
    const BlobReg* br = findBlob(bid);
    if (!br) {
      Serial.println("unknown blobId; use 'blobs'");
      return;
    }
    if (ensureBlobFile(fn, br->data, br->len)) Serial.println("writeblob OK");
    else Serial.println("writeblob failed");
  } else if (!strcmp(t0, "exec0")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Serial.println("usage: exec0 <file>");
      return;
    }
    int rv;
    if (!execBlobNoArg(fn, rv)) Serial.println("exec0 failed");
    else {
      Serial.print("Return=");
      Serial.println(rv);
    }
  } else if (!strcmp(t0, "exec1")) {
    char* fn;
    char* as;
    if (!nextToken(p, fn) || !nextToken(p, as)) {
      Serial.println("usage: exec1 <file> <a>");
      return;
    }
    int a = (int)strtol(as, nullptr, 0);
    int rv;
    if (!execBlob1Arg(fn, a, rv)) Serial.println("exec1 failed");
    else {
      Serial.print("Return=");
      Serial.println(rv);
    }
  } else if (!strcmp(t0, "exec2")) {
    char* fn;
    char* as;
    char* bs;
    if (!nextToken(p, fn) || !nextToken(p, as) || !nextToken(p, bs)) {
      Serial.println("usage: exec2 <file> <a> <b>");
      return;
    }
    int a = (int)strtol(as, nullptr, 0);
    int b = (int)strtol(bs, nullptr, 0);
    int rv;
    if (!execBlob2Arg(fn, a, b, rv)) Serial.println("exec2 failed");
    else {
      Serial.print("Return=");
      Serial.println(rv);
    }
  } else if (!strcmp(t0, "exec3")) {
    char* fn;
    char* as;
    char* bs;
    char* cs;
    if (!nextToken(p, fn) || !nextToken(p, as) || !nextToken(p, bs) || !nextToken(p, cs)) {
      Serial.println("usage: exec3 <file> <a> <b> <c>");
      return;
    }
    int a = (int)strtol(as, nullptr, 0);
    int b = (int)strtol(bs, nullptr, 0);
    int c = (int)strtol(cs, nullptr, 0);
    int rv;
    if (!execBlob3Arg(fn, a, b, c, rv)) Serial.println("exec3 failed");
    else {
      Serial.print("Return=");
      Serial.println(rv);
    }
  } else if (!strcmp(t0, "exec4")) {
    char* fn;
    char* s0;
    char* s1;
    char* s2;
    char* s3;
    if (!nextToken(p, fn) || !nextToken(p, s0) || !nextToken(p, s1) || !nextToken(p, s2) || !nextToken(p, s3)) {
      Serial.println("usage: exec4 <file> <a> <b> <c> <d>");
      return;
    }
    int a = (int)strtol(s0, nullptr, 0);
    int b = (int)strtol(s1, nullptr, 0);
    int c = (int)strtol(s2, nullptr, 0);
    int d = (int)strtol(s3, nullptr, 0);
    int rv;
    if (!execBlob4Arg(fn, a, b, c, d, rv)) Serial.println("exec4 failed");
    else {
      Serial.print("Return=");
      Serial.println(rv);
    }
  } else if (!strcmp(t0, "del")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Serial.println("usage: del <file>");
      return;
    }
    if (fs.deleteFile(fn)) Serial.println("deleted");
    else Serial.println("delete failed");
  } else if (!strcmp(t0, "format")) {
    if (fs.format()) Serial.println("FS formatted");
    else Serial.println("format failed");
  } else if (!strcmp(t0, "wipebootloader")) {
    Serial.println("Erasing entire chip... this can take a while");
    if (fs.wipeChip()) {
      Serial.println("Chip wiped, rebooting to bootloader now..");
      rp2040.rebootToBootloader();
    } else {
      Serial.println("wipe failed");
    }
  } else if (!strcmp(t0, "wipereboot")) {
    Serial.println("Erasing entire chip... this can take a while");
    if (fs.wipeChip()) {
      Serial.println("Chip wiped, rebooting now..");
      rp2040.reboot();
    } else {
      Serial.println("wipe failed");
    }
  } else if (!strcmp(t0, "wipe")) {
    Serial.println("Erasing entire chip... this can take a while");
    if (fs.wipeChip()) Serial.println("Chip wiped");
    else Serial.println("wipe failed");
  } else if (!strcmp(t0, "timeout")) {
    char* msStr;
    if (!nextToken(p, msStr)) {
      Serial.print("timeout override = ");
      Serial.print((uint32_t)g_timeout_override_ms);
      Serial.println(" ms (0 = use per-call defaults)");
      return;
    }
    uint32_t ms = (uint32_t)strtoul(msStr, nullptr, 0);
    g_timeout_override_ms = ms;
    Serial.print("timeout override set to ");
    Serial.print(ms);
    Serial.println(" ms");
  } else {
    Serial.println("Unknown command. Type 'help'.");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(5);
  flash.begin();
  uint8_t mfr, memType, capCode;
  uint32_t capacityBytes = flash.readJEDEC(mfr, memType, capCode);
  Serial.print("JEDEC ID: ");
  printHexByte(mfr);
  Serial.print(' ');
  printHexByte(memType);
  Serial.print(' ');
  printHexByte(capCode);
  Serial.print("  Capacity: ");
  Serial.print(capacityBytes);
  Serial.println(" bytes");
  if (!fs.mount(true)) {
    Serial.println("FS mount failed");
    return;
  }
  Serial.println("Initial listing:");
  fs.listFilesToSerial();
  fs.setAlignToPageBoundary(false);  // packing preference for append-only writes
#if AUTO_RUN_AUTOGEN_ON_BOOT
  autogenBlobWrites();
#endif
  Serial.println();
  printHelp();
  Serial.println();
  listBlobs();
  Serial.println();
  Serial.print("> ");
}

// Core1 (Arduino-Pico) entry points
void setup1() {
  // Start idle
  g_job_flag = 0u;
}
void loop1() {
  // Poll mailbox; returns immediately if no job
  core1WorkerPoll();
  // Power-friendly idle between polls
  tight_loop_contents();
}
void loop() {
  if (readLine()) {
    handleCommand(lineBuf);
    Serial.print("> ");
  }
}