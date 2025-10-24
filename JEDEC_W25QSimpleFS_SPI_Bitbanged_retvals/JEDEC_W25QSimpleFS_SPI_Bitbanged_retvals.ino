// RP2040 + Winbond W25Q128, bit-banged SPI, simple FS + blob loader + console + autogen + core1 exec via mailbox
#include "W25QSimpleFS.h"
//#include "MCPDACBitbang.h"
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

// ================== Bitbang SPI flash filesystem ==================
const uint8_t PIN_MISO = 0;  // GP0 -> MISO
const uint8_t PIN_CS = 1;    // GP1 -> CS
const uint8_t PIN_SCK = 2;   // GP2 -> SCK
const uint8_t PIN_MOSI = 3;  // GP3 -> MOSI
W25QBitbang flash(PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI);
W25QSimpleFS fs(flash);

// ================== Return mailbox reservation (Scratch) ==================
extern "C" __scratch_x("blob_mailbox") __attribute__((aligned(4)))  // Reserve mailbox in Scratch X (0x20040000..0x20040FFF)
int8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX] = { 0 };

// ================== Blob registry (compiled-in arrays) ==================
struct BlobReg {
  const char* id;       // console id (e.g., "ret42")
  const uint8_t* data;  // array from header
  unsigned int len;     // length from header
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
static volatile uint32_t g_timeout_override_ms = 0;  // Timeout override (0 = use per-call defaults)
static inline uint32_t getTimeout(uint32_t defMs) {
  return g_timeout_override_ms ? g_timeout_override_ms : defMs;
}

// ================== Core1 execution plumbing (mailbox) ==================
#define MAX_EXEC_ARGS 64
extern "C" int call_with_args_thumb(void* entryThumb, uint32_t argc, const int32_t* args) {
  if (!entryThumb) return 0;
  // Always prepare 64 ints; extra entries are zero.
  constexpr uint32_t N = 64;
  int32_t a64[N] = { 0 };
  uint32_t n = argc;
  if (n > N) n = N;
  if (args && n) {
    for (uint32_t i = 0; i < n; ++i) a64[i] = args[i];
  }
  using fvar_t = int (*)(...);  // call ABI: r0..r3 then stack per AAPCS
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

// ================== Shared mailbox ==================
static volatile ExecJob g_job;
static volatile int32_t g_result = 0;
static volatile int32_t g_status = 0;
static volatile uint32_t g_job_flag = 0;  // Job flag: 0=idle, 1=ready, 2=running, 3=done
static void core1WorkerPoll() {
  // Core1 worker poll (called from loop1())
  if (g_job_flag != 1u) return;

  __asm volatile("dsb" ::
                   : "memory");
  __asm volatile("isb" ::
                   : "memory");

  g_job_flag = 2u;

  // Snapshot job to local (avoid volatile reads during call)
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
    void* entryThumb = (void*)(job.code | 1u);
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
      // One last check in case it completed just now
      if (g_job_flag == 3u) break;
      Serial.println("core1 timeout");
      // Clear flag so we don't wedge the system; core1 will either be idle or eventually
      // set to 3, but we allow new jobs.
      g_job_flag = 0u;
      return false;
    }
  }
  // If worker signaled an error
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

// ================== FS helpers and console ==================
static bool checkNameLen(const char* name) {
  size_t n = strlen(name);
  if (n == 0 || n > fs.MAX_NAME) {
    Serial.print("Error: filename length ");
    Serial.print(n);
    Serial.print(" exceeds max ");
    Serial.print(fs.MAX_NAME);
    Serial.println(". Use a shorter name (e.g., mulA.bin, mulB.bin).");
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
    //fs.readFileRange(fname, off, buf, n);
    size_t got = fs.readFileRange(fname, off, buf, n);
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
static bool loadFileToExecBuf(const char* fname, void*& rawOut, uint8_t*& alignedBuf, uint32_t& szOut) {
  // Load file fully into an aligned RAM buffer suitable for execution.
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
static bool ensureBlobFile(const char* fname, const uint8_t* data, uint32_t len, uint32_t reserve = fs.SECTOR_SIZE) {
  // Create/Update a file with a blob (manual path: create slot if not present, in-place update if possible)
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
static bool ensureBlobIfMissing(const char* fname, const uint8_t* data, uint32_t len, uint32_t reserve = fs.SECTOR_SIZE) {
  // Auto-write ONLY IF MISSING (does not update if present)
  if (!checkNameLen(fname)) return false;
  if (fs.exists(fname)) {
    Serial.printf("Skipping: %s\n", fname);
    return true;
  }
  Serial.printf("Auto-creating %s (%d bytes)...\n", fname, reserve);
  if (fs.createFileSlot(fname, reserve, data, len)) {
    Serial.println("Created and wrote blob");
    return true;
  }
  Serial.println("Auto-create failed");
  return false;
}
static void autogenBlobWrites() {
  // Call this to auto-create only the blobs you enabled with #defines above
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

// ================== Mailbox helpers ==================
static inline void mailboxClearFirstByte() {
  volatile uint8_t* mb = (volatile uint8_t*)(uintptr_t)BLOB_MAILBOX_ADDR;
  mb[0] = 0;  // sentinel
}
static void mailboxPrintIfAny() {
  const volatile char* p = (const volatile char*)(uintptr_t)BLOB_MAILBOX_ADDR;
  if (p[0] == '\0') return;  // nothing written
  Serial.print(" Info=\"");
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) {
    char c = p[i];
    if (!c) break;
    Serial.print(c);
  }
  Serial.println("\"");
}

// ================== Execution helpers ==================
static char lineBuf[fs.SECTOR_SIZE];
static bool execBlobGeneric(const char* fname, int argc, const int argv[], int& retVal) {
  // Generic exec: runs blob with 0..MAX_EXEC_ARGS integer args, handles mailbox clear/print.
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
static int nextToken(char*& p, char*& tok) {
  // Skip spaces/tabs/newlines
  while (*p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
  if (!*p) {
    tok = nullptr;
    return 0;
  }
  tok = p;
  // Read up to next whitespace
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  if (*p) {
    *p = 0;
    ++p;
  }
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
  Serial.printf("  exec <file> [a0..aN]         - execute blob with 0..%d int args on core1\n", (MAX_EXEC_ARGS - 1));
  Serial.println("  timeout [ms]                 - show or set core1 timeout override (0=defaults)");
  Serial.println();
}
static void handleCommand(char* line) {
  char* p = line;
  char* t0;
  if (!nextToken(p, t0)) return;

  if (!strcmp(t0, "help")) {
    printHelp();

  } else if (!strcmp(t0, "reboot")) {
    rp2040.reboot();

  } else if (!strcmp(t0, "meminfo")) {
    Serial.println();
    Serial.printf("Total Heap:        %d bytes\n", rp2040.getTotalHeap());
    Serial.printf("Free Heap:         %d bytes\n", rp2040.getFreeHeap());
    Serial.printf("Used Heap:         %d bytes\n", rp2040.getUsedHeap());
    Serial.printf("Total PSRAM Heap:  %d bytes\n", rp2040.getTotalPSRAMHeap());
    Serial.printf("Free PSRAM Heap:   %d bytes\n", rp2040.getFreePSRAMHeap());
    Serial.printf("Used PSRAM Heap:   %d bytes\n", rp2040.getUsedPSRAMHeap());
    Serial.printf("Free Stack:        %d bytes\n", rp2040.getFreeStack());

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

// ================== Core0/Core1 entries ==================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(5);

  // Verify mailbox placement is inside Scratch X window
  uintptr_t actual = (uintptr_t)BLOB_MAILBOX;
  uintptr_t expect = (uintptr_t)BLOB_MAILBOX_ADDR;  // now 0x20040000
  constexpr uintptr_t SCRATCH_X_SIZE = 4096;        // 4KB

  if (!(actual >= expect && (actual + BLOB_MAILBOX_MAX) <= (expect + SCRATCH_X_SIZE))) {
    Serial.println("WARNING: Mailbox not fully inside expected Scratch X range.");
    Serial.print("  Expect range: 0x");
    Serial.print(expect, HEX);
    Serial.print(" .. 0x");
    Serial.println(expect + SCRATCH_X_SIZE - 1, HEX);
    Serial.print("  Actual addr:  0x");
    Serial.println(actual, HEX);
  } else {
    Serial.print("Reserved mailbox at 0x");
    Serial.print(actual, HEX);
    Serial.print(" size ");
    Serial.println((unsigned)BLOB_MAILBOX_MAX);
  }

  // Clear mailbox
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) BLOB_MAILBOX[i] = 0;

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

  autogenBlobWrites();  // Automatically generate blob binaries to SPI flash from hard-coded examples

  Serial.println();
  printHelp();
  Serial.println();
  listBlobs();
  Serial.println();
  Serial.print("> ");

  // e.g., MOSI=7, SCK=6, CS=5, LDAC tied to GND externally
  //initWavegen(7, 6, 5, -1);
}
void setup1() {
  g_job_flag = 0u;  // Start idle
}
void loop1() {
  core1WorkerPoll();      // Poll mailbox; returns immediately if no job
  tight_loop_contents();  // Power-friendly idle
}
void loop() {
  if (readLine()) {
    handleCommand(lineBuf);
    Serial.print("> ");
  }
}
