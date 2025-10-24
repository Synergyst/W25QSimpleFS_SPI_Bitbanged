/*
  main_psram_flash_switch.ino
  Integrated program:
   - RP2040 + W25Q bit-banged flash (W25QSimpleFS)
   - Optional PSRAM bit-banged storage (PSRAMSimpleFS)
   - Runtime-selectable storage backend (storage flash|psram)
   - Dev/Prod mode toggle (mode dev|prod)
   - Persistent block read/write stored in chosen backend
   - Single-line binary upload (puthex/putb64) into the active FS
  Notes:
   - FS_SECTOR_SIZE is a compile-time constant used for buffers.
   - PSRAM capacity should be set correctly when constructing PSRAMSimpleFS.
*/
#include <Arduino.h>
#include "W25QSimpleFS.h"
#include "W25QBitbang.h"
#include "PSRAMBitbang.h"
#include "PSRAMSimpleFS.h"
#include "blob_mailbox_config.h"
#include "blob_add2.h"
#define FILE_ADD2 "add2"
#include "blob_ret42.h"
#define FILE_RET42 "ret42"
#include "blob_square.h"
#define FILE_SQUARE "square"
#include "blob_pwmc.h"
#define FILE_PWMC "pwmc"
#include "blob_uart_rx_to_mailbox.h"
#define FILE_RX "rxmb"
#include "blob_mcp4921_bb_wave.h"
#define FILE_WAVE "wave"

// Static buffer-related compile-time constant (used previously as fs.SECTOR_SIZE)
#define FS_SECTOR_SIZE 4096

// ========== bitbang pin definitions (flash) ==========
const uint8_t PIN_FLASH_MISO = 0;  // GP0
const uint8_t PIN_FLASH_CS = 1;    // GP1
const uint8_t PIN_FLASH_SCK = 2;   // GP2
const uint8_t PIN_FLASH_MOSI = 3;  // GP3

// ========== bitbang pin definitions (psram) ==========
const bool PSRAM_ENABLE_QPI = false;
const uint8_t PSRAM_CLOCK_DELAY_US = 0;  // 3 is a safe slow default
const uint8_t PIN_PSRAM_CS = 9;
const uint8_t PIN_PSRAM_MISO = 12;
const uint8_t PIN_PSRAM_MOSI = 11;
const uint8_t PIN_PSRAM_SCK = 10;
const uint8_t PIN_PSRAM_IO2 = 13;
const uint8_t PIN_PSRAM_IO3 = 14;

// Instantiate bitbang drivers
W25QBitbang flashBB(PIN_FLASH_MISO, PIN_FLASH_CS, PIN_FLASH_SCK, PIN_FLASH_MOSI);
W25QSimpleFS fsFlash(flashBB);

// PSRAM driver; capacity must be provided (example 8 MiB)
PSRAMBitbang psramBB(PIN_PSRAM_CS, PIN_PSRAM_MISO, PIN_PSRAM_MOSI, PIN_PSRAM_SCK);
constexpr uint32_t PSRAM_CAPACITY_BYTES = 8UL * 1024UL * 1024UL;
PSRAMSimpleFS fsPSRAM(psramBB, PSRAM_CAPACITY_BYTES);

// Choose active storage at runtime
enum class StorageBackend {
  Flash,
  PSRAM_BACKEND
};
static StorageBackend g_storage = StorageBackend::Flash;

// Dev/prod mode
static bool g_dev_mode = true;

// Active FS forwarding helpers (thin wrapper)
struct ActiveFS {
  bool (*mount)(bool) = nullptr;
  bool (*format)() = nullptr;
  bool (*wipeChip)() = nullptr;
  bool (*exists)(const char*) = nullptr;
  bool (*createFileSlot)(const char*, uint32_t, const uint8_t*, uint32_t) = nullptr;
  // NOTE: use int for mode so we don't tie the function pointer type to either FS' enum type.
  // Lambdas will cast int -> the appropriate enum.
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

// ========== Blob registry from your original file ==========
struct BlobReg {
  const char* id;
  const uint8_t* data;
  unsigned int len;
};
static const BlobReg g_blobs[] = {
  { FILE_RET42, blob_ret42, blob_ret42_len },
  { FILE_ADD2, blob_add2, blob_add2_len },
  { FILE_SQUARE, blob_square, blob_square_len },
  { FILE_PWMC, blob_pwmc, blob_pwmc_len },
  { FILE_RX, blob_uart_rx_to_mailbox, blob_uart_rx_to_mailbox_len },
  { FILE_WAVE, blob_mcp4921_bb_wave, blob_mcp4921_bb_wave_len },
};
static const size_t g_blobs_count = sizeof(g_blobs) / sizeof(g_blobs[0]);

static void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

// Helper to bind either flash or psram into activeFs at runtime
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
    // cast int -> W25QSimpleFS::WriteMode before calling fsFlash
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
  } else {  // PSRAM backend
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
    // cast int -> PSRAMSimpleFS::WriteMode before calling fsPSRAM
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsPSRAM.writeFile(n, d, s, static_cast<PSRAMSimpleFS::WriteMode>(m));
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

// mailbox & exec plumbing preserved
extern "C" __scratch_x("blob_mailbox") __attribute__((aligned(4)))
int8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX] = { 0 };

// ===== Core1 execution plumbing (mailbox) =====
#define MAX_EXEC_ARGS 64
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
static bool runOnCore1(uintptr_t codeAligned, uint32_t sz, uint32_t argc, const int32_t* argv, int& retVal, uint32_t timeoutMs = 100) {
  if (g_job_flag != 0u) {
    Serial.println("core1 busy");
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
      Serial.println("core1 timeout");
      g_job_flag = 0u;  // allow new jobs; worker may later set 3 (ok)
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
  g_job_flag = 0u;
  return true;
}

// ===== Mailbox helpers =====
static inline void mailboxClearFirstByte() {
  volatile uint8_t* mb = (volatile uint8_t*)(uintptr_t)BLOB_MAILBOX_ADDR;
  mb[0] = 0;  // sentinel
}
static void mailboxPrintIfAny() {
  const volatile char* p = (const volatile char*)(uintptr_t)BLOB_MAILBOX_ADDR;
  if (p[0] == '\0') return;
  Serial.print(" Info=\"");
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) {
    char c = p[i];
    if (!c) break;
    Serial.print(c);
  }
  Serial.println("\"");
}

// ===== Execution helper =====
static bool execBlobGeneric(const char* fname, int argc, const int argv[], int& retVal) {
  if (argc < 0) argc = 0;
  if (argc > (int)MAX_EXEC_ARGS) argc = (int)MAX_EXEC_ARGS;
  void* raw = nullptr;
  uint8_t* buf = nullptr;
  uint32_t sz = 0;
  if (!loadFileToExecBuf(fname, raw, buf, sz)) return false;
  mailboxClearFirstByte();
  uintptr_t code = (uintptr_t)buf;  // aligned buffer address
  Serial.print("Calling entry on core1 at 0x");
  Serial.println((uintptr_t)buf, HEX);
  bool ok = runOnCore1(code, sz, (uint32_t)argc, (const int32_t*)argv, retVal, getTimeout(100000));
  if (!ok) {
    Serial.println("exec: core1 run failed");
    free(raw);
    return false;
  }
  Serial.print("Return=");
  Serial.println(retVal);
  mailboxPrintIfAny();
  free(raw);
  return true;
}

// ========== FS helper replacements using activeFs ==========
static bool checkNameLen(const char* name) {
  size_t n = strlen(name);
  if (n == 0 || n > ActiveFS::MAX_NAME) {
    Serial.print("Error: filename length ");
    Serial.print(n);
    Serial.print(" exceeds max ");
    Serial.print(ActiveFS::MAX_NAME);
    Serial.println(". Use a shorter name.");
    return false;
  }
  return true;
}
static void listBlobs() {
  Serial.println("Available blobs:");
  for (size_t i = 0; i < g_blobs_count; ++i) {
    Serial.printf(" - %s  \t(%d bytes)\n", g_blobs[i].id, g_blobs[i].len);
  }
}
static const BlobReg* findBlob(const char* id) {
  for (size_t i = 0; i < g_blobs_count; ++i)
    if (strcmp(g_blobs[i].id, id) == 0) return &g_blobs[i];
  return nullptr;
}

// Example: dumpFileHead() now uses activeFs.readFileRange etc.
static void dumpFileHead(const char* fname, uint32_t count) {
  uint32_t sz = 0;
  if (!activeFs.getFileSize(fname, sz) || sz == 0) {
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
    uint32_t got = activeFs.readFileRange(fname, off, buf, n);
    if (got != n) {
      Serial.println("  read error");
      break;
    }
    Serial.print("  ");
    for (size_t i = 0; i < n; ++i) {
      if (i) Serial.print(' ');
      printHexByte(buf[i]);
    }
    Serial.println();
    off += n;
  }
}

// loadFileToExecBuf uses activeFs.readFile
static bool loadFileToExecBuf(const char* fname, void*& rawOut, uint8_t*& alignedBuf, uint32_t& szOut) {
  rawOut = nullptr;
  alignedBuf = nullptr;
  szOut = 0;
  uint32_t sz = 0;
  if (!activeFs.getFileSize(fname, sz) || sz == 0) {
    Serial.println("load: missing/empty");
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
  if (activeFs.readFile(fname, buf, sz) != sz) {
    Serial.println("load: read failed");
    free(raw);
    return false;
  }
  rawOut = raw;
  alignedBuf = buf;
  szOut = sz;
  return true;
}

// ensureBlobFile & ensureBlobIfMissing - forward to activeFs
static bool ensureBlobFile(const char* fname, const uint8_t* data, uint32_t len, uint32_t reserve = ActiveFS::SECTOR_SIZE) {
  if (!checkNameLen(fname)) return false;
  if (!activeFs.exists(fname)) {
    Serial.print("Creating slot ");
    Serial.print(fname);
    Serial.print(" (");
    Serial.print(reserve);
    Serial.println(" bytes)...");
    if (activeFs.createFileSlot(fname, reserve, data, len)) {
      Serial.println("Created and wrote blob");
      return true;
    } else {
      Serial.println("Failed to create slot");
      return false;
    }
  }
  uint32_t addr, size, cap;
  if (!activeFs.getFileInfo(fname, addr, size, cap)) {
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
    Serial.println("Blob already up to date");
    return true;
  }
  if (cap >= len && activeFs.writeFileInPlace(fname, data, len, false)) {
    Serial.println("Updated in place");
    return true;
  }
  if (activeFs.writeFile(fname, data, len, static_cast<int>(W25QSimpleFS::WriteMode::ReplaceIfExists))) {
    Serial.println("Updated by allocating new space");
    return true;
  }
  Serial.println("Failed to update file");
  return false;
}
static bool ensureBlobIfMissing(const char* fname, const uint8_t* data, uint32_t len, uint32_t reserve = ActiveFS::SECTOR_SIZE) {
  if (!checkNameLen(fname)) return false;
  if (activeFs.exists(fname)) {
    Serial.printf("Skipping: %s\n", fname);
    return true;
  }
  Serial.printf("Auto-creating %s (%d bytes)...\n", fname, reserve);
  if (activeFs.createFileSlot(fname, reserve, data, len)) {
    Serial.println("Created and wrote blob");
    return true;
  }
  Serial.println("Auto-create failed");
  return false;
}
static void autogenBlobWrites() {
  bool allOk = true;
  allOk &= ensureBlobIfMissing(FILE_RET42, blob_ret42, blob_ret42_len);
  allOk &= ensureBlobIfMissing(FILE_ADD2, blob_add2, blob_add2_len);
  allOk &= ensureBlobIfMissing(FILE_SQUARE, blob_square, blob_square_len);
  allOk &= ensureBlobIfMissing(FILE_PWMC, blob_pwmc, blob_pwmc_len);
  allOk &= ensureBlobIfMissing(FILE_RX, blob_uart_rx_to_mailbox, blob_uart_rx_to_mailbox_len);
  allOk &= ensureBlobIfMissing(FILE_WAVE, blob_mcp4921_bb_wave, blob_mcp4921_bb_wave_len);
  Serial.print("Autogen:  ");
  Serial.println(allOk ? "OK" : "some failures");
}

// ---------- persistent block helpers (now operate on active FS) ----------
const uint32_t PERSIST_ADDR = 0x000200;  // logical within FS data region (we use file-based storage; keep for reference)
const size_t PERSIST_LEN = 32;
const uint32_t MAGIC = 0x5053524D;  // "PSRM"

// Read a 32-bit big-endian value at addr (relative to file data block via readDataRange usage)
static bool readUint32FromFile(const char* fname, uint32_t offset, uint32_t& v) {
  uint8_t tmp[4];
  if (activeFs.readFileRange(fname, offset, tmp, 4) != 4) return false;
  v = ((uint32_t)tmp[0] << 24) | ((uint32_t)tmp[1] << 16) | ((uint32_t)tmp[2] << 8) | (uint32_t)tmp[3];
  return true;
}
static bool writeUint32ToFile(const char* fname, uint32_t offset, uint32_t v) {
  // Read file info and writeData into same file (simple approach: in-place rewrite)
  uint8_t tmp[4];
  tmp[0] = (v >> 24) & 0xFF;
  tmp[1] = (v >> 16) & 0xFF;
  tmp[2] = (v >> 8) & 0xFF;
  tmp[3] = v & 0xFF;
  // If file doesn't exist, create slot and write
  if (!activeFs.exists(fname)) {
    if (!activeFs.createFileSlot(fname, ActiveFS::SECTOR_SIZE, nullptr, 0)) return false;
  }
  // read existing size
  uint32_t size = 0;
  activeFs.getFileSize(fname, size);
  // expand file if necessary
  uint32_t need = offset + 4;
  if (need > size) {
    // allocate new file with appended content (read old if exists)
    uint8_t* buf = (uint8_t*)malloc(need);
    if (!buf) return false;
    memset(buf, 0x00, need);
    if (size > 0) activeFs.readFile(fname, buf, size);
    memcpy(buf + offset, tmp, 4);
    bool ok = activeFs.writeFile(fname, buf, need, static_cast<int>(W25QSimpleFS::WriteMode::ReplaceIfExists));
    free(buf);
    return ok;
  } else {
    // in-place write of the first 4 bytes (NOTE: ActiveFS doesn't support offset writes natively)
    return activeFs.writeFileInPlace(fname, tmp, 4, true);
  }
}

// ---------- Binary upload helpers (single-line puthex/putb64) ----------
static inline int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Decode contiguous hex string (no spaces). Returns malloc'd buffer in out, size in outLen.
static bool decodeHexString(const char* hex, uint8_t*& out, uint32_t& outLen) {
  out = nullptr;
  outLen = 0;
  if (!hex) return false;
  size_t n = strlen(hex);
  if (n == 0) return false;
  if (n & 1) {
    Serial.println("puthex: error: hex string length is odd");
    return false;
  }
  uint32_t bytes = (uint32_t)(n / 2);
  uint8_t* buf = (uint8_t*)malloc(bytes);
  if (!buf) {
    Serial.println("puthex: malloc failed");
    return false;
  }
  for (uint32_t i = 0; i < bytes; ++i) {
    int hi = hexVal(hex[2 * i + 0]);
    int lo = hexVal(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) {
      Serial.println("puthex: invalid hex character");
      free(buf);
      return false;
    }
    buf[i] = (uint8_t)((hi << 4) | lo);
  }
  out = buf;
  outLen = bytes;
  return true;
}

// Base64 decoder (ignores whitespace). Returns malloc'd buffer in out, size in outLen.
static int8_t b64Map[256];
static void initB64MapOnce() {
  static bool inited = false;
  if (inited) return;
  for (int i = 0; i < 256; ++i) b64Map[i] = -1;
  for (char c = 'A'; c <= 'Z'; ++c) b64Map[(uint8_t)c] = (int8_t)(c - 'A');
  for (char c = 'a'; c <= 'z'; ++c) b64Map[(uint8_t)c] = (int8_t)(26 + (c - 'a'));
  for (char c = '0'; c <= '9'; ++c) b64Map[(uint8_t)c] = (int8_t)(52 + (c - '0'));
  b64Map[(uint8_t)'+'] = 62;
  b64Map[(uint8_t)'/'] = 63;
  inited = true;
}
static bool decodeBase64String(const char* s, uint8_t*& out, uint32_t& outLen) {
  out = nullptr;
  outLen = 0;
  if (!s) return false;
  initB64MapOnce();
  size_t inLen = strlen(s);
  // Worst-case output length
  uint32_t outCap = (uint32_t)(((inLen + 3) / 4) * 3);
  uint8_t* buf = (uint8_t*)malloc(outCap ? outCap : 1);
  if (!buf) {
    Serial.println("putb64: malloc failed");
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
        Serial.println("putb64: invalid base64 character");
        free(buf);
        return false;
      }
      vals[vCount++] = v;
    }
    if (vCount == 4) {
      uint32_t v0 = (uint32_t)vals[0];
      uint32_t v1 = (uint32_t)vals[1];
      uint32_t v2 = (uint32_t)vals[2];
      uint32_t v3 = (uint32_t)vals[3];
      uint8_t b0 = (uint8_t)((v0 << 2) | (v1 >> 4));
      uint8_t b1 = (uint8_t)(((v1 & 0x0F) << 4) | (v2 >> 2));
      uint8_t b2 = (uint8_t)(((v2 & 0x03) << 6) | v3);
      if (pad == 0) {
        if (o + 3 > outCap) {
          free(buf);
          return false;
        }
        buf[o++] = b0;
        buf[o++] = b1;
        buf[o++] = b2;
      } else if (pad == 1) {
        if (o + 2 > outCap) {
          free(buf);
          return false;
        }
        buf[o++] = b0;
        buf[o++] = b1;
      } else if (pad == 2) {
        if (o + 1 > outCap) {
          free(buf);
          return false;
        }
        buf[o++] = b0;
      } else {
        // pad > 2 invalid
        free(buf);
        Serial.println("putb64: invalid padding");
        return false;
      }
      vCount = 0;
      pad = 0;
    }
  }
  if (vCount != 0) {
    // Partial quartet not allowed
    free(buf);
    Serial.println("putb64: truncated input");
    return false;
  }
  out = buf;
  outLen = o;
  return true;
}

static bool writeBinaryToFS(const char* fname, const uint8_t* data, uint32_t len) {
  if (!checkNameLen(fname)) return false;
  // Replace if exists, allocate new space otherwise
  bool ok = activeFs.writeFile(fname, data, len, static_cast<int>(W25QSimpleFS::WriteMode::ReplaceIfExists));
  return ok;
}

// ---------- Serial console / command handling ----------
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
  Serial.println("Commands (filename max 32 chars):");
  Serial.println("  help                         - this help");
  Serial.println("  storage                      - show active storage");
  Serial.println("  storage flash|psram          - switch storage backend");
  Serial.println("  mode                         - show mode (dev/prod)");
  Serial.println("  mode dev|prod                - set dev or prod mode");
  Serial.println("  persist read                 - read persist file from active storage");
  Serial.println("  persist write                - write persist file to active storage");
  Serial.println("  blobs                        - list compiled-in blobs");
  Serial.println("  autogen                      - auto-create enabled blobs if missing");
  Serial.println("  files                        - list files in FS");
  Serial.println("  info <file>                  - show file addr/size/cap");
  Serial.println("  dump <file> <nbytes>         - hex dump head of file");
  Serial.println("  mkSlot <file> <reserve>      - create sector-aligned slot");
  Serial.println("  writeblob <file> <blobId>    - create/update file from blob");
  Serial.printf("  exec <file> [a0..aN]         - execute blob with 0..%d int args on core1\n", (MAX_EXEC_ARGS - 1));
  Serial.println("  del <file>                   - delete a file");
  Serial.println("  format                       - format active FS");
  Serial.println("  wipe                         - erase active chip (DANGEROUS)");
  Serial.println("  wipereboot                   - erase chip then reboot");
  Serial.println("  wipebootloader               - erase chip then reboot to bootloader");
  Serial.println("  meminfo                      - show heap/stack info");
  Serial.println("  reboot                       - reboot the MCU");
  Serial.println("  timeout [ms]                 - show or set core1 timeout override (0=defaults)");
  Serial.println("  puthex <file> <hex>          - upload binary as hex string (single line)");
  Serial.println("  putb64 <file> <base64>       - upload binary as base64 (single line)");
  Serial.println();
  Serial.println("Linux hints:");
  Serial.println("  # Base64 (single line, no wraps)");
  Serial.println("  base64 -w0 your.bin");
  Serial.println("  # Hex (single line, no spaces)");
  Serial.println("  xxd -p -c 999999 your.bin | tr -d '\\n'");
  Serial.println("Example:");
  Serial.println("  putb64 myprog <paste_output_of_base64>");
  Serial.println("  puthex myprog <paste_output_of_xxd>");
  Serial.println("Note: For blobs you intend to exec, even byte length is recommended (Thumb).");
  Serial.println();
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
      Serial.print("Active storage: ");
      Serial.println(g_storage == StorageBackend::Flash ? "flash" : "psram");
      return;
    }
    if (!strcmp(tok, "flash")) {
      g_storage = StorageBackend::Flash;
      bindActiveFs(g_storage);
      bool ok = activeFs.mount(true);  // flash: safe to auto-format if empty
      Serial.println("Switched active storage to FLASH");
      Serial.println(ok ? "Mounted FLASH (auto-format if empty)" : "Mount failed (FLASH)");
    } else if (!strcmp(tok, "psram")) {
      g_storage = StorageBackend::PSRAM_BACKEND;
      bindActiveFs(g_storage);
      bool ok = activeFs.mount(false);  // PSRAM: DO NOT auto-format, preserve contents
      Serial.println("Switched active storage to PSRAM");
      Serial.println(ok ? "Mounted PSRAM (no auto-format)" : "Mount failed (PSRAM)");
    } else {
      Serial.println("usage: storage [flash|psram]");
    }

  } else if (!strcmp(t0, "mode")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Serial.print("Mode: ");
      Serial.println(g_dev_mode ? "dev" : "prod");
      return;
    }
    if (!strcmp(tok, "dev")) {
      g_dev_mode = true;
      Serial.println("mode=dev");
    } else if (!strcmp(tok, "prod")) {
      g_dev_mode = false;
      Serial.println("mode=prod");
    } else Serial.println("usage: mode [dev|prod]");

  } else if (!strcmp(t0, "persist")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Serial.println("usage: persist [read|write]");
      return;
    }
    if (!strcmp(tok, "read")) {
      // we store persistent block as a file called ".persist" (hidden-ish)
      const char* pf = ".persist";
      if (!activeFs.exists(pf)) {
        Serial.println("persist read: not found");
        return;
      }
      uint32_t sz = 0;
      activeFs.getFileSize(pf, sz);
      uint8_t buf[PERSIST_LEN];
      memset(buf, 0, PERSIST_LEN);
      uint32_t got = activeFs.readFile(pf, buf, PERSIST_LEN);
      Serial.printf("persist read: %d bytes: ", got);
      for (size_t i = 0; i < got; ++i) {
        if (i) Serial.print(' ');
        printHexByte(buf[i]);
      }
      Serial.println();
    } else if (!strcmp(tok, "write")) {
      const char* pf = ".persist";
      uint8_t buf[PERSIST_LEN];
      for (size_t i = 0; i < PERSIST_LEN; ++i) buf[i] = (uint8_t)(i ^ (g_dev_mode ? 0xA5 : 0x5A));
      // create or overwrite
      if (!activeFs.exists(pf)) {
        if (!activeFs.createFileSlot(pf, ActiveFS::SECTOR_SIZE, buf, PERSIST_LEN)) {
          Serial.println("persist write: create failed");
          return;
        }
      } else {
        if (!activeFs.writeFileInPlace(pf, buf, PERSIST_LEN, true)) {
          if (!activeFs.writeFile(pf, buf, PERSIST_LEN, static_cast<int>(W25QSimpleFS::WriteMode::ReplaceIfExists))) {
            Serial.println("persist write: write failed");
            return;
          }
        }
      }
      Serial.println("persist write: OK");
    } else {
      Serial.println("usage: persist [read|write]");
    }

  } else if (!strcmp(t0, "autogen")) {
    autogenBlobWrites();

  } else if (!strcmp(t0, "files")) {
    activeFs.listFilesToSerial();

  } else if (!strcmp(t0, "info")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Serial.println("usage: info <file>");
      return;
    }
    uint32_t a, s, c;
    if (activeFs.getFileInfo(fn, a, s, c)) {
      Serial.print(fn);
      Serial.print(": addr=0x");
      Serial.print(a, HEX);
      Serial.print(" size=");
      Serial.print(s);
      Serial.print(" cap=");
      Serial.println(c);
    } else {
      Serial.println("not found");
    }

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
    if (activeFs.createFileSlot(fn, res, nullptr, 0)) Serial.println("slot created");
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

  } else if (!strcmp(t0, "exec")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Serial.println("usage: exec <file> [a0 ... aN]");
      return;
    }
    static int argvN[MAX_EXEC_ARGS];
    int argc = 0;
    char* tok;
    while (argc < (int)MAX_EXEC_ARGS && nextToken(p, tok)) {
      argvN[argc++] = (int)strtol(tok, nullptr, 0);
    }
    int rv;
    if (!execBlobGeneric(fn, argc, argvN, rv)) {
      Serial.println("exec failed");
    }

  } else if (!strcmp(t0, "blobs")) {
    listBlobs();

  } else if (!strcmp(t0, "timeout")) {
    char* msStr;
    if (!nextToken(p, msStr)) {
      Serial.print("timeout override = ");
      Serial.print((uint32_t)g_timeout_override_ms);
      Serial.println(" ms (0 = use per-call defaults)");
    } else {
      uint32_t ms = (uint32_t)strtoul(msStr, nullptr, 0);
      g_timeout_override_ms = ms;
      Serial.print("timeout override set to ");
      Serial.print(ms);
      Serial.println(" ms");
    }

  } else if (!strcmp(t0, "meminfo")) {
    Serial.println();
    Serial.printf("Total Heap:        %d bytes\n", rp2040.getTotalHeap());
    Serial.printf("Free Heap:         %d bytes\n", rp2040.getFreeHeap());
    Serial.printf("Used Heap:         %d bytes\n", rp2040.getUsedHeap());
    Serial.printf("Total PSRAM Heap:  %d bytes\n", rp2040.getTotalPSRAMHeap());
    Serial.printf("Free PSRAM Heap:   %d bytes\n", rp2040.getFreePSRAMHeap());
    Serial.printf("Used PSRAM Heap:   %d bytes\n", rp2040.getUsedPSRAMHeap());
    Serial.printf("Free Stack:        %d bytes\n", rp2040.getFreeStack());

  } else if (!strcmp(t0, "reboot")) {
    Serial.printf("Rebooting..\n");
    delay(20);
    yield();
    rp2040.reboot();

  } else if (!strcmp(t0, "del")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Serial.println("usage: del <file>");
      return;
    }
    if (activeFs.deleteFile && activeFs.deleteFile(fn)) Serial.println("deleted");
    else Serial.println("delete failed");

  } else if (!strcmp(t0, "format")) {
    if (activeFs.format && activeFs.format()) Serial.println("FS formatted");
    else Serial.println("format failed");

  } else if (!strcmp(t0, "wipebootloader")) {
    Serial.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) {
      Serial.println("Chip wiped, rebooting to bootloader now..");
      delay(20);
      yield();
      rp2040.rebootToBootloader();
    } else {
      Serial.println("wipe failed");
    }

  } else if (!strcmp(t0, "wipereboot")) {
    Serial.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) {
      Serial.println("Chip wiped, rebooting now..");
      delay(20);
      yield();
      rp2040.reboot();
    } else {
      Serial.println("wipe failed");
    }

  } else if (!strcmp(t0, "wipe")) {
    Serial.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) Serial.println("Chip wiped");
    else Serial.println("wipe failed");

  } else if (!strcmp(t0, "puthex")) {
    // Usage: puthex <file> <hex>
    char* fn;
    char* hex;
    if (!nextToken(p, fn) || !nextToken(p, hex)) {
      Serial.println("usage: puthex <file> <hex>");
      Serial.println("tip: xxd -p -c 999999 your.bin | tr -d '\\n'");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint8_t* bin = nullptr;
    uint32_t binLen = 0;
    if (!decodeHexString(hex, bin, binLen)) {
      Serial.println("puthex: decode failed");
      return;
    }
    bool ok = writeBinaryToFS(fn, bin, binLen);
    free(bin);
    if (ok) {
      Serial.print("puthex: wrote ");
      Serial.print(binLen);
      Serial.print(" bytes to ");
      Serial.println(fn);
      if (binLen & 1u) {
        Serial.println("note: odd-sized file; if used as Thumb blob, exec will reject (needs even bytes).");
      }
    } else {
      Serial.println("puthex: write failed");
    }

  } else if (!strcmp(t0, "putb64")) {
    // Usage: putb64 <file> <base64>
    char* fn;
    char* b64;
    if (!nextToken(p, fn) || !nextToken(p, b64)) {
      Serial.println("usage: putb64 <file> <base64>");
      Serial.println("tip: base64 -w0 your.bin");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint8_t* bin = nullptr;
    uint32_t binLen = 0;
    if (!decodeBase64String(b64, bin, binLen)) {
      Serial.println("putb64: decode failed");
      return;
    }
    bool ok = writeBinaryToFS(fn, bin, binLen);
    free(bin);
    if (ok) {
      Serial.print("putb64: wrote ");
      Serial.print(binLen);
      Serial.print(" bytes to ");
      Serial.println(fn);
      if (binLen & 1u) {
        Serial.println("note: odd-sized file; if used as Thumb blob, exec will reject (needs even bytes).");
      }
    } else {
      Serial.println("putb64: write failed");
    }

  } else {
    Serial.println("Unknown command. Type 'help'.");
  }
}

// ========== Setup and main loop ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(5);

  // Start both drivers
  flashBB.begin();
  psramBB.begin();
  psramBB.setClockDelayUs(PSRAM_CLOCK_DELAY_US);  // safe slow default
  psramBB.setExtraDataPins(PIN_PSRAM_IO2, PIN_PSRAM_IO3);
  psramBB.setModeQuad(PSRAM_ENABLE_QPI);
  flashBB.begin();

  // Bind default active FS (flash) then attempt mount
  bindActiveFs(g_storage);
  bool mounted = false;
  if (g_storage == StorageBackend::Flash) {
    mounted = activeFs.mount(true);  // flash: may auto-format if empty
  } else {
    mounted = activeFs.mount(false);  // psram: preserve data across reboots
  }
  if (!mounted) {
    Serial.println("FS mount failed on active storage");
  }

  // Optional: clear mailbox at boot
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) BLOB_MAILBOX[i] = 0;

  Serial.println("System ready. Type 'help'.");
  // Do NOT print the prompt here; we’ll echo the prompt with the next command.
}

void setup1() {
  g_job_flag = 0u;  // Start idle
}

void loop1() {
  core1WorkerPoll();
  tight_loop_contents();
}

void loop() {
  if (readLine()) {
    // Move off the existing prompt line, then echo the command
    Serial.println();
    Serial.print("> ");
    Serial.println(lineBuf);
    // Run the command (it will print its outputs)
    handleCommand(lineBuf);
    // Show prompt for the next command
    Serial.print("> ");
  }
}