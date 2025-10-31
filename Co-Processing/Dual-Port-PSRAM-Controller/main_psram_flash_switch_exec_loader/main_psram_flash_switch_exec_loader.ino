// main_psram_flash_switch_exec_loader.ino
// ------- Shared HW SPI (FLASH + PSRAM) -------
#include "ConsolePrint.h"
#include "UnifiedSPIMemSimpleFS.h"
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
struct BlobReg;               // Forward declaration
static ConsolePrint Console;  // Console wrapper
// ------- MC Compiiler -------
#include "MCCompiler.h"
// ------- User preference configuration -------
#include "blob_mailbox_config.h"  // Do not remove this line. This is the memory address where we handle the shared interprocess-mailbox-buffer
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
// ========== Static buffer-related compile-time constant ==========
#define FS_SECTOR_SIZE 4096
// Pins you already have:
const uint8_t PIN_FLASH_MISO = 12;  // GP12
const uint8_t PIN_FLASH_CS = 9;     // GP9
const uint8_t PIN_FLASH_SCK = 10;   // GP10
const uint8_t PIN_FLASH_MOSI = 11;  // GP11
const uint8_t PIN_PSRAM_MISO = 12;  // GP12
const uint8_t PIN_PSRAM_MOSI = 11;  // GP11
const uint8_t PIN_PSRAM_SCK = 10;   // GP10
const uint8_t PIN_PSRAM_CS0 = 14;   // GP14
const uint8_t PIN_PSRAM_CS1 = 15;   // GP15
const uint8_t PIN_PSRAM_CS2 = 26;   // GP26
const uint8_t PIN_PSRAM_CS3 = 27;   // GP27
// ========== Flash/PSRAM instances (needed before bindActiveFs) ==========
// Persisted config you already have
const size_t PERSIST_LEN = 32;
enum class StorageBackend {
  Flash,
  PSRAM_BACKEND
};
static StorageBackend g_storage = StorageBackend::Flash;
// Unified manager: one bus, many CS
UnifiedSpiMem::Manager uniMem(PIN_PSRAM_SCK, PIN_PSRAM_MOSI, PIN_PSRAM_MISO);
// SimpleFS facades (these are your “handles”)
W25QUnifiedSimpleFS fsFlash;  // Use MX35UnifiedSimpleFS if your flash is MX35LF SPI-NAND
PSRAMUnifiedSimpleFS fsPSRAM;
// If your flash chip is actually MX35LF (SPI-NAND), do this instead:
// MX35UnifiedSimpleFS fsFlash;
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
struct FsIndexEntry {
  // Scan directory region and build the latest view of files.
  // Out arrays are optional; pass nullptr to only count or check emptiness.
  char name[ActiveFS::MAX_NAME + 1];
  uint32_t size;
  bool deleted;
  uint32_t seq;
};
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
      return fsFlash.writeFile(n, d, s, static_cast<W25QUnifiedSimpleFS::WriteMode>(m));
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
static bool makePath(char* out, size_t outCap, const char* folder, const char* name) {
  if (!out || !folder || !name) return false;
  // Compose "folder/name"
  int needed = snprintf(out, outCap, "%s/%s", folder, name);
  if (needed < 0) return false;
  // needed = number of chars (excluding null). Enforce FS name limit and buffer size.
  if ((size_t)needed > ActiveFS::MAX_NAME) return false;
  if ((size_t)needed >= outCap) return false;
  return true;
}
static void normalizePathInPlace(char* s, bool wantTrailingSlash) {
  // Collapse duplicate slashes, strip leading '/', and keep/strip trailing slash as requested
  if (!s) return;
  // Strip leading slashes
  size_t r = 0;
  while (s[r] == '/') ++r;
  // Collapse duplicate slashes
  size_t w = 0;
  for (; s[r]; ++r) {
    char c = s[r];
    if (c == '/' && w > 0 && s[w - 1] == '/') continue;
    s[w++] = c;
  }
  s[w] = 0;
  // Enforce or strip trailing slash
  size_t n = strlen(s);
  if (wantTrailingSlash) {
    if (n > 0 && s[n - 1] != '/') {
      if (n + 1 < ActiveFS::MAX_NAME + 1) {
        s[n] = '/';
        s[n + 1] = 0;
      }
    }
  } else {
    while (n > 0 && s[n - 1] == '/') { s[--n] = 0; }
  }
}
static bool makePathSafe(char* out, size_t outCap, const char* folder, const char* name) {
  // Safe "folder/name" join without introducing double slashes; stores without trailing slash.
  if (!out || !folder || !name) return false;
  char tmp[ActiveFS::MAX_NAME + 1];
  if (folder[0] == 0) {
    if (snprintf(tmp, sizeof(tmp), "%s", name) < 0) return false;
  } else {
    if (snprintf(tmp, sizeof(tmp), "%s/%s", folder, name) < 0) return false;
  }
  normalizePathInPlace(tmp, /*wantTrailingSlash=*/false);
  size_t L = strlen(tmp);
  if (L > ActiveFS::MAX_NAME || L >= outCap) return false;
  memcpy(out, tmp, L + 1);
  return true;
}
static bool writeFileToFolder(const char* folder, const char* fname, const uint8_t* data, uint32_t len) {
  char path[ActiveFS::MAX_NAME + 1];
  if (!makePathSafe(path, sizeof(path), folder, fname)) {
    Console.println("path too long for SimpleFS (max 32 chars)");
    return false;
  }
  // Create sector-aligned slot (optional) then write; or just writeFile() directly.
  if (!activeFs.exists(path)) {
    // Reserve at least one sector; if data is larger, round up to sector boundary.
    uint32_t reserve = ActiveFS::SECTOR_SIZE;
    if (len > reserve) reserve = (len + (ActiveFS::SECTOR_SIZE - 1)) & ~(ActiveFS::SECTOR_SIZE - 1);
    return activeFs.createFileSlot(path, reserve, data, len);
  }
  // Try in-place update first, allow reallocate if needed
  if (activeFs.writeFileInPlace(path, data, len, true)) return true;
  return activeFs.writeFile(path, data, len, /*ReplaceIfExists*/ 0);
}
static uint32_t readFileFromFolder(const char* folder, const char* fname, uint8_t* buf, uint32_t bufSize) {
  char path[ActiveFS::MAX_NAME + 1];
  if (!makePathSafe(path, sizeof(path), folder, fname)) {
    Console.println("path too long for SimpleFS (max 32 chars)");
    return 0;
  }
  return activeFs.readFile(path, buf, bufSize);
}
static bool folderExists(const char* absFolder) {
  // Marker is a zero-length file named "folder/". This consumes no data space.
  // absFolder must end with '/'
  if (!absFolder || !absFolder[0]) return false;
  if (absFolder[strlen(absFolder) - 1] != '/') return false;
  return activeFs.exists(absFolder);
}
static bool mkdirFolder(const char* path) {
  // Create zero-length "folder/" marker
  if (activeFs.exists(path)) return true;  // already exists as a file or marker
  return activeFs.writeFile(path, nullptr, 0, /*ReplaceIfExists*/ 0);
}
static bool touchPath(const char* cwd, const char* arg) {
  // Create an empty file if it doesn't exist. No-op if it already exists.
  // If the path ends with '/', treat it like a folder marker creation (mkdir shortcut).
  if (!arg) return false;
  // If arg ends with '/', interpret as folder marker (mkdir).
  size_t L = strlen(arg);
  if (L > 0 && arg[L - 1] == '/') {
    char marker[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(marker, sizeof(marker), cwd, arg, /*wantTrailingSlash*/ true)) {
      Console.println("touch: folder path too long (<= 32 chars)");
      return false;
    }
    if (folderExists(marker)) return true;  // already present
    return mkdirFolder(marker);
  }
  // Regular file path
  char path[ActiveFS::MAX_NAME + 1];
  if (!pathJoin(path, sizeof(path), cwd, arg, /*wantTrailingSlash*/ false)) {
    Console.println("touch: path too long (<= 32 chars)");
    return false;
  }
  if (activeFs.exists(path)) {
    // POSIX touch would update mtime; we have no timestamps, so it's a no-op.
    return true;
  }
  // Create zero-length file (no slot reserved, does not advance data head)
  return activeFs.writeFile(path, /*data*/ nullptr, /*size*/ 0, static_cast<int>(W25QUnifiedSimpleFS::WriteMode::ReplaceIfExists));
}
// ========== mv helpers ==========
static const char* lastSlash(const char* s) {
  const char* p = strrchr(s, '/');
  return p ? (p + 1) : s;
}
static bool buildAbsFile(char* out, size_t outCap, const char* cwd, const char* path) {
  // Normalize: build absolute file path (no trailing slash). Reject folder paths.
  if (!path || !out) return false;
  size_t L = strlen(path);
  if (L > 0 && path[L - 1] == '/') return false;  // folder-like path not allowed here
  return pathJoin(out, outCap, cwd, path, /*wantTrailingSlash*/ false);
}
static bool cmdMvImpl(const char* cwd, const char* srcArg, const char* dstArg) {
  // Move/rename a file within the same SimpleFS (copy then delete).
  // - If dst ends with '/', treat as folder (dst/<basename(src)>).
  // - Destination is created as a sector-aligned slot (slotSafe=Y) using source capacity when possible.
  if (!srcArg || !dstArg) return false;
  // Build absolute source file (reject folder source)
  char srcAbs[ActiveFS::MAX_NAME + 1];
  if (!buildAbsFile(srcAbs, sizeof(srcAbs), cwd, srcArg)) {
    Console.println("mv: invalid source path");
    return false;
  }
  // Optional convenience for legacy/typed double slashes:
  bool srcExists = activeFs.exists(srcAbs);
  if (!srcExists) {
    if (strstr(srcAbs, "//")) {
      char alt[sizeof(srcAbs)];
      strncpy(alt, srcAbs, sizeof(alt));
      alt[sizeof(alt) - 1] = 0;
      normalizePathInPlace(alt, /*wantTrailingSlash=*/false);
      if (activeFs.exists(alt)) {
        strncpy(srcAbs, alt, sizeof(srcAbs));
        srcAbs[sizeof(srcAbs) - 1] = 0;
        srcExists = true;
      }
    }
  }
  if (!srcExists) {
    Console.println("mv: source not found");
    return false;
  }
  // Does source exist? Retrieve size and capacity
  uint32_t srcAddr = 0, srcSize = 0, srcCap = 0;
  if (!activeFs.getFileInfo(srcAbs, srcAddr, srcSize, srcCap)) {
    Console.println("mv: getFileInfo failed");
    return false;
  }
  // Build destination (folder-aware)
  char dstAbs[ActiveFS::MAX_NAME + 1];
  size_t Ldst = strlen(dstArg);
  bool dstIsFolder = (Ldst > 0 && dstArg[Ldst - 1] == '/');
  if (dstIsFolder) {
    // Make a folder path WITHOUT trailing slash
    char folderNoSlash[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(folderNoSlash, sizeof(folderNoSlash), cwd, dstArg, /*wantTrailingSlash*/ false)) {
      Console.println("mv: destination folder path too long");
      return false;
    }
    // Append basename safely (guarantees single slash)
    const char* base = lastSlash(srcAbs);
    if (!makePathSafe(dstAbs, sizeof(dstAbs), folderNoSlash, base)) {
      Console.println("mv: resulting path too long");
      return false;
    }
  } else {
    if (!buildAbsFile(dstAbs, sizeof(dstAbs), cwd, dstArg)) {
      Console.println("mv: invalid destination path");
      return false;
    }
  }
  // Final normalization (safety)
  normalizePathInPlace(dstAbs, /*wantTrailingSlash=*/false);
  if (strcmp(srcAbs, dstAbs) == 0) {
    Console.println("mv: source and destination are the same");
    return true;
  }
  if (pathTooLongForOnDisk(dstAbs)) {
    Console.println("mv: destination name too long for FS (would be truncated)");
    return false;
  }
  // Read source contents
  uint8_t* buf = (uint8_t*)malloc(srcSize ? srcSize : 1);
  if (!buf) {
    Console.println("mv: malloc failed");
    return false;
  }
  uint32_t got = activeFs.readFile(srcAbs, buf, srcSize);
  if (got != srcSize) {
    Console.println("mv: read failed");
    free(buf);
    return false;
  }
  // Prepare destination slot
  uint32_t reserve = srcCap;
  if (reserve < ActiveFS::SECTOR_SIZE) {
    // If src wasn't allocated via slot, reserve at least one sector aligned
    uint32_t a = (srcSize + (ActiveFS::SECTOR_SIZE - 1)) & ~(ActiveFS::SECTOR_SIZE - 1);
    if (a > reserve) reserve = a;
  }
  bool ok = false;
  if (!activeFs.exists(dstAbs)) {
    // Create slot with same capacity budget and write contents in one go
    ok = activeFs.createFileSlot(dstAbs, reserve, buf, srcSize);
  } else {
    // Destination exists: try in-place, otherwise replace
    uint32_t dA, dS, dC;
    if (!activeFs.getFileInfo(dstAbs, dA, dS, dC)) dC = 0;
    if (dC >= srcSize) {
      ok = activeFs.writeFileInPlace(dstAbs, buf, srcSize, false);
    }
    if (!ok) {
      ok = activeFs.writeFile(dstAbs, buf, srcSize, /*ReplaceIfExists*/ 0);
    }
  }
  if (!ok) {
    Console.println("mv: write to destination failed");
    free(buf);
    return false;
  }
  // Delete source so another file can reuse its slot
  if (!activeFs.deleteFile(srcAbs)) {
    Console.println("mv: warning: source delete failed");
    // still return success for the copy
  } else {
    Console.println("mv: ok");
  }
  free(buf);
  return true;
}
static void printPct2(uint32_t num, uint32_t den) {
  // Percent printer with 2 decimals (no float)
  if (den == 0) {
    Console.print("n/a");
    return;
  }
  uint32_t scaled = (uint32_t)(((uint64_t)num * 10000ULL + (den / 2)) / den);
  Console.printf("%lu.%02lu%%", (unsigned long)(scaled / 100), (unsigned long)(scaled % 100));
}
static uint32_t dirBytesUsedEstimate() {
  // Count directory entries to estimate directory space used (32 bytes per valid entry)
  FsIndexEntry tmp[64];
  size_t n = buildFsIndex(tmp, 64);  // robust scan over full DIR
  return (uint32_t)(n * 32u);
}
static void cmdDf() {
  // Active device (mounted)
  // FIX: choose the device for the currently active storage
  UnifiedSpiMem::MemDevice* dev = (g_storage == StorageBackend::Flash) ? fsFlash.raw().device() : fsPSRAM.raw().device();
  if (dev) {
    const auto t = dev->type();
    const char* style = UnifiedSpiMem::deviceTypeName(t);
    const uint8_t cs = dev->cs();
    const uint64_t devCap = dev->capacity();
    const uint32_t dataStart = activeFs.dataRegionStart();
    const uint32_t fsCap32 = activeFs.capacity();
    const uint32_t dataCap = (fsCap32 > dataStart) ? (fsCap32 - dataStart) : 0;
    const uint32_t dataUsed = (activeFs.nextDataAddr() > dataStart) ? (activeFs.nextDataAddr() - dataStart) : 0;
    const uint32_t dataFree = (dataCap > dataUsed) ? (dataCap - dataUsed) : 0;
    const uint32_t dirUsed = dirBytesUsedEstimate();
    const uint32_t dirFree = (64u * 1024u > dirUsed) ? (64u * 1024u - dirUsed) : 0;
    Console.println("Filesystem (active):");
    Console.printf("  Device:  %s  CS=%u\n", style, (unsigned)cs);
    Console.printf("  DevCap:  %llu bytes\n", (unsigned long long)devCap);
    Console.printf("  FS data: %lu used (", (unsigned long)dataUsed);
    printPct2(dataUsed, dataCap);
    Console.printf(")  %lu free (", (unsigned long)dataFree);
    printPct2(dataFree, dataCap);
    Console.println(")");
    Console.printf("  DIR:     %lu used (", (unsigned long)dirUsed);
    printPct2(dirUsed, 64u * 1024u);
    Console.printf(")  %lu free\n", (unsigned long)dirFree);
  } else {
    Console.println("Filesystem (active): none");
  }
  // Other detected devices (raw capacities)
  size_t n = uniMem.detectedCount();
  if (n == 0) return;
  Console.println("Detected devices:");
  for (size_t i = 0; i < n; ++i) {
    const auto* di = uniMem.detectedInfo(i);
    if (!di) continue;
    // Mark which one is the active filesystem device (by CS + type)
    bool isActive = false;
    if (dev) {
      isActive = (di->cs == dev->cs() && di->type == dev->type());
    }
    Console.printf("  CS=%u  Type=%s  Vendor=%s  Cap=%llu bytes%s\n",
                   di->cs,
                   UnifiedSpiMem::deviceTypeName(di->type),
                   di->vendorName,
                   (unsigned long long)di->capacityBytes,
                   isActive ? "  [mounted]" : "");
  }
}
// ========== Minimal directory enumeration (reads the DIR table directly) ==========
static inline UnifiedSpiMem::MemDevice* activeFsDevice() {
  // FIX: helper to return the MemDevice for the currently active storage
  return (g_storage == StorageBackend::Flash) ? fsFlash.raw().device() : fsPSRAM.raw().device();
}
static inline uint32_t rd32_be(const uint8_t* p) {
  return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3];
}
static bool isAllFF(const uint8_t* p, size_t n) {
  for (size_t i = 0; i < n; ++i)
    if (p[i] != 0xFF) return false;
  return true;
}
static size_t buildFsIndex(FsIndexEntry* out, size_t outMax) {
  // FIX: scan the DIR of the currently active storage (no cross-device preference)
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) return 0;
  constexpr uint32_t DIR_START = 0x000000;
  constexpr uint32_t DIR_SIZE = 64 * 1024;
  constexpr uint32_t ENTRY = 32;
  constexpr uint32_t CHUNK = 256;
  FsIndexEntry map[64];
  size_t mapCount = 0;
  uint8_t buf[CHUNK];
  for (uint32_t off = 0; off < DIR_SIZE; off += CHUNK) {
    size_t toRead = ((off + CHUNK) <= DIR_SIZE) ? CHUNK : (DIR_SIZE - off);
    size_t got = dev->read(DIR_START + off, buf, toRead);
    if (got == 0) continue;  // treat as empty; keep scanning
    // Walk this chunk in 32-byte records
    for (uint32_t c = 0; c + ENTRY <= got; c += ENTRY) {
      const uint8_t* rec = &buf[c];
      // Only accept records with "WF" header; skip everything else (including 0xFF blocks)
      if (rec[0] != 0x57 || rec[1] != 0x46) continue;
      uint8_t flags = rec[2];
      uint8_t nlen = rec[3];
      if (nlen == 0 || nlen > ActiveFS::MAX_NAME) continue;
      char name[ActiveFS::MAX_NAME + 1];
      memset(name, 0, sizeof(name));
      for (uint8_t i = 0; i < nlen; ++i) name[i] = (char)rec[4 + i];
      uint32_t size = rd32_be(&rec[24]);
      uint32_t seq = rd32_be(&rec[28]);
      bool deleted = (flags & 0x01) != 0;
      // Upsert by name, keep highest seq
      int idx = -1;
      for (size_t i = 0; i < mapCount; ++i) {
        if (strncmp(map[i].name, name, ActiveFS::MAX_NAME) == 0) {
          idx = (int)i;
          break;
        }
      }
      if (idx < 0) {
        if (mapCount >= 64) continue;
        idx = (int)mapCount++;
        strncpy(map[idx].name, name, ActiveFS::MAX_NAME);
        map[idx].name[ActiveFS::MAX_NAME] = 0;
        map[idx].seq = 0;
      }
      if (seq >= map[idx].seq) {
        map[idx].seq = seq;
        map[idx].deleted = deleted;
        map[idx].size = size;
      }
    }
  }
  if (out && outMax) {
    size_t n = (mapCount < outMax) ? mapCount : outMax;
    for (size_t i = 0; i < n; ++i) out[i] = map[i];
  }
  return mapCount;
}
static bool hasPrefix(const char* name, const char* prefix) {
  // Helpers to test prefix match and extract single-level child
  size_t lp = strlen(prefix);
  return strncmp(name, prefix, lp) == 0;
}
static bool childOfFolder(const char* name, const char* folderPrefix, const char*& childOut) {
  // folderPrefix must end with '/'
  size_t lp = strlen(folderPrefix);
  if (strncmp(name, folderPrefix, lp) != 0) return false;
  const char* rest = name + lp;
  if (*rest == 0) return false;  // this is the folder marker itself
  // Only immediate child (no further '/')
  const char* slash = strchr(rest, '/');
  if (slash) return false;
  childOut = rest;
  return true;
}
static void dumpDirHeadRaw(uint32_t bytes = 256) {
  // FIX: dump from the active storage device
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) {
    Console.println("lsdebug: no device");
    return;
  }
  if (bytes == 0 || bytes > 1024) bytes = 256;  // keep it small on console
  uint8_t buf[1024];
  size_t got = dev->read(0, buf, bytes);
  Console.printf("lsdebug: read %u bytes @ 0x000000\n", (unsigned)got);
  for (size_t i = 0; i < got; i += 16) {
    Console.printf("  %04u: ", (unsigned)i);
    for (size_t j = 0; j < 16 && (i + j) < got; ++j) {
      uint8_t b = buf[i + j];
      if (b < 16) Console.print('0');
      Console.print(b, HEX);
      Console.print(' ');
    }
    Console.println();
  }
}
// ========== CWD + path helpers ==========
static char g_cwd[ActiveFS::MAX_NAME + 1] = "";  // "" = root, printed as "/"
static void pathStripTrailingSlashes(char* p) {
  if (!p) return;
  size_t n = strlen(p);
  while (n > 0 && p[n - 1] == '/') { p[--n] = 0; }
}
static bool pathJoin(char* out, size_t outCap, const char* base, const char* name, bool wantTrailingSlash) {
  if (!out || !name) return false;
  // Absolute path: strip one leading '/'
  if (name[0] == '/') {
    const char* s = name + 1;
    size_t L = strlen(s);
    if (L > ActiveFS::MAX_NAME || L >= outCap) return false;
    memcpy(out, s, L + 1);
    // Add trailing slash only for non-root
    if (wantTrailingSlash && L > 0 && out[L - 1] != '/') {
      if (L + 1 > ActiveFS::MAX_NAME || L + 2 > outCap) return false;
      out[L] = '/';
      out[L + 1] = 0;
    }
    return true;
  }
  // Relative
  if (!base) base = "";
  char tmp[ActiveFS::MAX_NAME + 1];
  int need = 0;
  if (base[0] == 0) need = snprintf(tmp, sizeof(tmp), "%s", name);
  else if (name[0] == 0) need = snprintf(tmp, sizeof(tmp), "%s", base);
  else need = snprintf(tmp, sizeof(tmp), "%s/%s", base, name);
  if (need < 0 || (size_t)need > ActiveFS::MAX_NAME || (size_t)need >= sizeof(tmp)) return false;
  size_t L = strlen(tmp);
  // Add trailing slash only for non-root
  if (wantTrailingSlash && L > 0 && tmp[L - 1] != '/') {
    if (L + 1 > ActiveFS::MAX_NAME) return false;
    tmp[L] = '/';
    tmp[L + 1] = 0;
  }
  if (L >= outCap) return false;
  memcpy(out, tmp, L + 1);
  return true;
}
static void pathParent(char* p) {
  if (!p) return;
  pathStripTrailingSlashes(p);
  size_t n = strlen(p);
  if (n == 0) return;
  // find last '/'
  char* last = strrchr(p, '/');
  if (!last) {
    p[0] = 0;  // up to root
  } else if (last == p) {
    p[0] = 0;  // treat single-level as root (SimpleFS has no true root entry)
  } else {
    *last = 0;
  }
}
static bool isFolderMarkerName(const char* nm) {
  size_t n = strlen(nm);
  return (n > 0 && nm[n - 1] == '/');
}
static const char* firstSlash(const char* s) {
  return strchr(s, '/');
}
static constexpr size_t FS_NAME_ONDISK_MAX = 17;  // conservative (writer seems to truncate at 17 here)
static bool pathTooLongForOnDisk(const char* full) {
  return strlen(full) > FS_NAME_ONDISK_MAX;
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
// ========== Return mailbox reservation (Scratch) ==========
extern "C" __scratch_x("blob_mailbox") __attribute__((aligned(4)))
int8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX] = { 0 };
// ================= ExecHost header =================
#include "ExecHost.h"
static ExecHost Exec;
// ================= FSHelpers header =================
static void updateExecFsTable() {
  // Helper to supply Exec with current FS function pointers
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
// ================= Memory Diagnostics =================
#ifdef ARDUINO_ARCH_RP2040
// Portable free stack helper for core0 (current core).
// On RP2040, uses linker symbols __StackTop/__StackBottom.
// Otherwise, falls back to a non-negative "gap to heap" estimate.
extern "C" {
  extern char __StackTop;     // high address (start of empty stack)
  extern char __StackBottom;  // low address (stack limit)
}
#include "unistd.h"
static uint32_t freeStackCurrentCoreApprox() {
  volatile uint8_t marker;            // lives on current stack
  uintptr_t sp = (uintptr_t)&marker;  // approximate SP
  uintptr_t top = (uintptr_t)&__StackTop;
  uintptr_t bottom = (uintptr_t)&__StackBottom;
  // Validate and compute: ARM stack grows down => bottom < sp <= top
  if (top > bottom && sp >= bottom && sp <= top) {
    return (uint32_t)(sp - bottom);  // free = SP - bottom
  }
  // Fallback: clamp "gap to heap end" at 0
  void* heapEnd = sbrk(0);
  intptr_t gap = (intptr_t)sp - (intptr_t)heapEnd;
  return (gap > 0) ? (uint32_t)gap : 0u;
}
#else
static uint32_t freeStackCurrentCoreApprox() {
  volatile uint8_t marker;
  void* heapEnd = sbrk(0);
  intptr_t gap = (intptr_t)&marker - (intptr_t)heapEnd;
  return (gap > 0) ? (uint32_t)gap : 0u;
}
#endif
#ifdef ARDUINO_ARCH_RP2040
static void printStackInfo() {
  volatile uint8_t marker;
  uintptr_t sp = (uintptr_t)&marker;
  uintptr_t top = (uintptr_t)&__StackTop;
  uintptr_t bottom = (uintptr_t)&__StackBottom;
  if (top > bottom && sp >= bottom && sp <= top) {
    uint32_t total = (uint32_t)(top - bottom);
    uint32_t used = (uint32_t)(top - sp);
    uint32_t free = (uint32_t)(sp - bottom);
    Console.printf("Stack total=%u used=%u free=%u bytes\n", total, used, free);
    return;
  }
  Console.printf("Free Stack (core0 approx): %u bytes\n", freeStackCurrentCoreApprox());
}
#endif
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
      if (buf[i] < 0x10) Console.print('0');
      Console.print(buf[i], HEX);
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
  if (activeFs.writeFile(fname, data, len, static_cast<int>(W25QUnifiedSimpleFS::WriteMode::ReplaceIfExists))) {
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
// ========== PSRAM diagnostics using UnifiedSPIMem ==========
static void psramPrintCapacityReport(UnifiedSpiMem::Manager& mgr, Stream& out = Serial) {
  size_t banks = 0;
  uint64_t total = 0;
  out.println();
  out.println("PSRAM capacity report (Unified):");
  for (size_t i = 0; i < mgr.detectedCount(); ++i) {
    const auto* di = mgr.detectedInfo(i);
    if (!di || di->type != UnifiedSpiMem::DeviceType::Psram) continue;
    banks++;
    total += di->capacityBytes;
    out.print("  Bank ");
    out.print(banks - 1);
    out.print(" (CS=");
    out.print(di->cs);
    out.print(")  Vendor=");
    out.print(di->vendorName);
    out.print("  Cap=");
    out.print((unsigned long)di->capacityBytes);
    out.print(" bytes");
    if (di->partHint) {
      out.print("  Part=");
      out.print(di->partHint);
    }
    out.print("  JEDEC:");
    for (uint8_t k = 0; k < di->jedecLen; ++k) {
      out.print(' ');
      if (di->jedec[k] < 16) out.print('0');
      out.print(di->jedec[k], HEX);
    }
    out.println();
  }
  out.print("Banks: ");
  out.println((unsigned)banks);
  out.print("Total: ");
  out.print((unsigned long)total);
  out.print(" bytes (");
  out.print((unsigned long)(total / (1024UL * 1024UL)));
  out.println(" MB)");
  out.println();
}
static void psramSafeSmokeTest(PSRAMUnifiedSimpleFS& fs, Stream& out = Serial) {
  auto* dev = fs.raw().device();
  if (!dev || dev->type() != UnifiedSpiMem::DeviceType::Psram) {
    out.println("PSRAM smoke test: PSRAM device not open");
    return;
  }
  const uint64_t cap = dev->capacity();
  if (cap < 1024) {
    out.println("PSRAM smoke test: capacity too small");
    return;
  }
  // Prefer testing right after FS allocation head when possible
  const uint32_t fsHead = fs.raw().nextDataAddr();
  const uint32_t dataStart = fs.raw().dataRegionStart();
  const uint32_t TEST_SIZE = 1024;    // small, quick, non-destructive
  uint64_t testAddr = fsHead + 4096;  // leave a small gap from FS head
  if (testAddr + TEST_SIZE > cap) {
    // fallback to end-of-chip window
    if (cap > TEST_SIZE) testAddr = cap - TEST_SIZE;
    else testAddr = 0;
  }
  // Align a bit (not strictly needed for PSRAM)
  testAddr = (testAddr + 0xFF) & ~0xFFull;
  if (testAddr < dataStart) testAddr = dataStart;
  out.print("PSRAM smoke test @ 0x");
  out.print((unsigned long)testAddr, HEX);
  out.print(" size=");
  out.print(TEST_SIZE);
  out.println(" bytes");
  uint8_t* original = (uint8_t*)malloc(TEST_SIZE);
  uint8_t* verify = (uint8_t*)malloc(TEST_SIZE);
  if (!original || !verify) {
    out.println("malloc failed for buffers");
    if (original) free(original);
    if (verify) free(verify);
    return;
  }
  // Save original contents
  if (dev->read(testAddr, original, TEST_SIZE) != TEST_SIZE) {
    out.println("read (backup) failed");
    free(original);
    free(verify);
    return;
  }
  // Pattern 1: 0xAA
  memset(verify, 0xAA, TEST_SIZE);
  if (!dev->write(testAddr, verify, TEST_SIZE)) {
    out.println("write pattern 0xAA failed");
    // attempt to restore
    (void)dev->write(testAddr, original, TEST_SIZE);
    free(original);
    free(verify);
    return;
  }
  memset(verify, 0x00, TEST_SIZE);
  if (dev->read(testAddr, verify, TEST_SIZE) != TEST_SIZE) {
    out.println("readback (0xAA) failed");
    (void)dev->write(testAddr, original, TEST_SIZE);
    free(original);
    free(verify);
    return;
  }
  bool okAA = true;
  for (uint32_t i = 0; i < TEST_SIZE; ++i) {
    if (verify[i] != 0xAA) {
      okAA = false;
      break;
    }
    if ((i & 63) == 0) yield();
  }
  out.println(okAA ? "Pattern 0xAA OK" : "Pattern 0xAA mismatch");
  // Pattern 2: address-based
  for (uint32_t i = 0; i < TEST_SIZE; ++i) {
    verify[i] = (uint8_t)((testAddr + i) ^ 0x5A);
  }
  if (!dev->write(testAddr, verify, TEST_SIZE)) {
    out.println("write pattern addr^0x5A failed");
    (void)dev->write(testAddr, original, TEST_SIZE);
    free(original);
    free(verify);
    return;
  }
  uint8_t* rb = (uint8_t*)malloc(TEST_SIZE);
  if (!rb) {
    out.println("malloc failed for readback");
    (void)dev->write(testAddr, original, TEST_SIZE);
    free(original);
    free(verify);
    return;
  }
  if (dev->read(testAddr, rb, TEST_SIZE) != TEST_SIZE) {
    out.println("readback (addr^0x5A) failed");
    free(rb);
    (void)dev->write(testAddr, original, TEST_SIZE);
    free(original);
    free(verify);
    return;
  }
  bool okAddr = true;
  for (uint32_t i = 0; i < TEST_SIZE; ++i) {
    uint8_t exp = (uint8_t)((testAddr + i) ^ 0x5A);
    if (rb[i] != exp) {
      okAddr = false;
      break;
    }
    if ((i & 63) == 0) yield();
  }
  out.println(okAddr ? "Pattern addr^0x5A OK" : "Pattern addr^0x5A mismatch");
  free(rb);
  // Restore original contents
  if (!dev->write(testAddr, original, TEST_SIZE)) {
    out.println("restore failed (data left with test pattern)");
  } else {
    out.println("restore OK");
  }
  free(original);
  free(verify);
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
  bool ok = activeFs.writeFile(fname, data, len, static_cast<int>(W25QUnifiedSimpleFS::WriteMode::ReplaceIfExists));
  return ok;
}
// ================= CompilerHelpers header =================
#include "CompilerHelpers.h"
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
static bool coprocCallFunc(const char* name, const int32_t* argv, uint32_t argc, int32_t& outResult) {
  // Send CMD_FUNC with classic int32 argv[argc]. On success returns true and sets outResult.
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
// ----------------- Nano-like editor integration -----------------
#include "TextEditor.h"
// ========== Serial console / command handling ==========
static char lineBuf[FS_SECTOR_SIZE];
static bool readLine() {
  // Cooked console line input with echo, backspace handling, and CR/LF normalization.
  // This fixes lack of echo in terminal emulators and supports Windows CRLF.
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
  Console.println("  puthex <file> <hex>          - upload binary as hex string (single line) to FS");
  Console.println("  putb64 <file> <base64>       - upload binary as base64 (single line) to FS");
  Console.println("  cc <src> <dst>               - compile Tiny-C source file to raw ARM Thumb-1 binary");
  Console.println();
  Console.println("Editor (Nano-style) commands:");
  Console.println("  nano <file>                  - open Nano-like editor for <file> (ActiveFS-backed)");
  Console.println("  edit <file>                  - alias for 'nano'");
  Console.println("    Keys in editor:");
  Console.println("      Arrows: move   Backspace/Delete: edit   Enter: newline");
  Console.println("      Ctrl+C: show cursor position   Ctrl+X: prompt to save and exit");
  Console.println();
  Console.println("Folders (SimpleFS path emulation; full path <= 32 chars):");
  Console.println("  pwd                         - show current folder (\"/\" = root)");
  Console.println("  cd / | cd .. | cd .         - change folder (root, parent, stay)");
  Console.println("  cd <path>                   - change to relative or absolute path");
  Console.println("  mkdir <path>                - create folder (creates zero-byte marker)");
  Console.println("  ls [path]                   - list current or specified folder");
  Console.println("  rmdir <path> [-r]           - remove folder; -r deletes all children");
  Console.println("  touch <path|name|folder/>   - create empty file if missing; no-op if exists");
  Console.println("                                if path ends with '/', creates folder marker (mkdir shortcut)");
  Console.println("  df                          - show device and FS usage (capacity/used/free/%)");
  Console.println("  mv <src> <dst|folder/>      - move/rename file; if dst ends with '/', moves into folder");
  Console.println("Examples:");
  Console.println("  mkdir scripts            -> creates folder marker 'scripts/'");
  Console.println("  cd scripts               -> change into 'scripts'");
  Console.println("  writeblob scripts/blink blinkscript");
  Console.println("  ls                       -> shows 'blink' and any child folders");
  Console.println("  cd /; ls scripts         -> list contents of '/scripts'");
  Console.println("  rmdir scripts -r         -> delete folder and all files under it");
  Console.println("Notes:");
  Console.println("  - SimpleFS is flat; folders are filename prefixes + trailing '/'.");
  Console.println("  - Use short paths (<= 32 chars total). No nested enumeration by FS API; ls filters by prefix.");
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
  Console.println("  coproc sexec myscript [a0..aN]");
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
        if (buf[i] < 0x10) Console.print('0');
        Console.print(buf[i], HEX);
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
          if (!activeFs.writeFile(pf, buf, PERSIST_LEN, static_cast<int>(W25QUnifiedSimpleFS::WriteMode::ReplaceIfExists))) {
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
#ifdef ARDUINO_ARCH_RP2040
    printStackInfo();  // detailed (total/used/free)
#else
    Console.printf("Free Stack (core0 approx): %u bytes\n", freeStackCurrentCoreApprox());
#endif
    psramPrintCapacityReport(uniMem);
  } else if (!strcmp(t0, "psramsmoketest")) {
    psramPrintCapacityReport(uniMem);
    psramSafeSmokeTest(fsPSRAM);
  } else if (!strcmp(t0, "reboot")) {
    Console.printf("Rebooting..\n");
    delay(20);
    yield();
    rp2040.reboot();
  } else if (!strcmp(t0, "del")) {
    char* arg;
    if (!nextToken(p, arg)) {
      Console.println("usage: del <file>");
      return;
    }
    // Build absolute, normalize
    char abs[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(abs, sizeof(abs), g_cwd, arg, /*wantTrailingSlash*/ false)) {
      Console.println("del: path too long (<= 32 chars buffer)");
      return;
    }
    normalizePathInPlace(abs, /*wantTrailingSlash=*/false);
    bool ok = false;
    if (activeFs.exists && activeFs.exists(abs)) ok = activeFs.deleteFile(abs);
    if (!ok && strstr(abs, "//")) {
      char alt[sizeof abs];
      strncpy(alt, abs, sizeof alt);
      alt[sizeof alt - 1] = 0;
      normalizePathInPlace(alt, false);
      if (activeFs.exists && activeFs.exists(alt)) ok = activeFs.deleteFile(alt);
    }
    Console.println(ok ? "deleted" : "delete failed");
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
  } else if (!strcmp(t0, "pwd")) {
    Console.print("cwd: /");
    Console.println(g_cwd);
  } else if (!strcmp(t0, "cd")) {
    char* arg;
    if (!nextToken(p, arg)) {
      Console.print("cwd: /");
      Console.println(g_cwd);
      return;
    }
    if (!strcmp(arg, "/")) {
      g_cwd[0] = 0;  // root
      Console.println("ok");
      return;
    }
    if (!strcmp(arg, ".")) {
      Console.println("ok");
      return;
    }
    if (!strcmp(arg, "..")) {
      pathParent(g_cwd);
      Console.println("ok");
      return;
    }
    char target[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(target, sizeof(target), g_cwd, arg, /*wantTrailingSlash*/ true)) {
      Console.println("cd: path too long for SimpleFS (<=32 chars)");
      return;
    }
    // Only allow cd into existing folder marker, otherwise reject (you can relax this if desired)
    if (!folderExists(target)) {
      Console.println("cd: no such folder (create with mkdir)");
      return;
    }
    // Store without trailing slash in cwd
    target[strlen(target) - 1] = 0;  // drop trailing '/'
    strncpy(g_cwd, target, sizeof(g_cwd));
    g_cwd[sizeof(g_cwd) - 1] = 0;
    Console.println("ok");
  } else if (!strcmp(t0, "mkdir")) {
    char* arg;
    if (!nextToken(p, arg)) {
      Console.println("usage: mkdir <name|/abs|rel/path>");
      return;
    }
    // Disallow root
    if (strcmp(arg, "/") == 0 || arg[0] == 0) {
      Console.println("mkdir: refusing to create root");
      return;
    }
    char marker[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(marker, sizeof(marker), g_cwd, arg, /*wantTrailingSlash*/ true)) {
      Console.println("mkdir: path too long for SimpleFS (<=32 chars)");
      return;
    }
    if (folderExists(marker)) {
      Console.println("mkdir: already exists");
      return;
    }
    if (mkdirFolder(marker)) Console.println("mkdir: ok");
    else Console.println("mkdir: failed");
  } else if (!strcmp(t0, "ls")) {
    // Determine target folder prefix (no leading slash, trailing slash if non-root)
    char* arg;
    char folder[ActiveFS::MAX_NAME + 1] = { 0 };
    bool ok;
    if (!nextToken(p, arg)) {
      ok = pathJoin(folder, sizeof(folder), g_cwd, "", /*wantTrailingSlash*/ true);
    } else if (!strcmp(arg, "/")) {
      folder[0] = 0;
      ok = true;  // root
    } else if (!strcmp(arg, ".")) {
      ok = pathJoin(folder, sizeof(folder), g_cwd, "", true);
    } else if (!strcmp(arg, "..")) {
      char parent[ActiveFS::MAX_NAME + 1];
      strncpy(parent, g_cwd, sizeof(parent));
      parent[sizeof(parent) - 1] = 0;
      pathParent(parent);
      ok = pathJoin(folder, sizeof(folder), parent, "", true);
    } else {
      ok = pathJoin(folder, sizeof(folder), g_cwd, arg, /*wantTrailingSlash*/ true);
    }
    if (!ok) {
      Console.println("ls: path too long");
      return;
    }
    FsIndexEntry idx[64];
    size_t n = buildFsIndex(idx, 64);
    Console.print("Listing /");
    Console.print(folder);
    Console.println(":");
    // Root: list top-level files (no '/'), and show top-level folders once
    if (folder[0] == 0) {
      // First, files at root (no '/')
      for (size_t i = 0; i < n; ++i) {
        if (idx[i].deleted) continue;
        if (strchr(idx[i].name, '/') == nullptr) {
          Console.print("  ");
          Console.print(idx[i].name);
          Console.print("  (");
          Console.print(idx[i].size);
          Console.println(" bytes)");
        }
      }
      // Then, top-level folders (dedup by first segment)
      const size_t MAX_SEEN = 32;
      char seen[MAX_SEEN][ActiveFS::MAX_NAME + 1];
      size_t seenCount = 0;
      auto seenPush = [&](const char* seg) {
        for (size_t k = 0; k < seenCount; ++k)
          if (strcmp(seen[k], seg) == 0) return false;
        if (seenCount < MAX_SEEN) {
          strncpy(seen[seenCount], seg, ActiveFS::MAX_NAME);
          seen[seenCount][ActiveFS::MAX_NAME] = 0;
          ++seenCount;
          return true;
        }
        return false;
      };
      for (size_t i = 0; i < n; ++i) {
        if (idx[i].deleted) continue;
        const char* slash = strchr(idx[i].name, '/');
        if (!slash) continue;
        char seg[ActiveFS::MAX_NAME + 1];
        size_t segLen = (size_t)(slash - idx[i].name);
        if (segLen > ActiveFS::MAX_NAME) segLen = ActiveFS::MAX_NAME;
        memcpy(seg, idx[i].name, segLen);
        seg[segLen] = 0;
        if (seenPush(seg)) {
          Console.print("  ");
          Console.print(seg);
          Console.println("/");
        }
      }
      return;
    }
    // Subfolder: immediate children under "folder"
    size_t folderLen = strlen(folder);
    if (folderExists(folder)) Console.println("  .");
    const size_t MAX_SEEN = 32;
    char seenChild[MAX_SEEN][ActiveFS::MAX_NAME + 1];
    size_t seenCount = 0;
    auto seenPush = [&](const char* seg) {
      for (size_t k = 0; k < seenCount; ++k)
        if (strcmp(seenChild[k], seg) == 0) return false;
      if (seenCount < MAX_SEEN) {
        strncpy(seenChild[seenCount], seg, ActiveFS::MAX_NAME);
        seenChild[seenCount][ActiveFS::MAX_NAME] = 0;
        ++seenCount;
        return true;
      }
      return false;
    };
    for (size_t i = 0; i < n; ++i) {
      if (idx[i].deleted) continue;
      if (strncmp(idx[i].name, folder, folderLen) != 0) continue;
      const char* rest = idx[i].name + folderLen;
      if (*rest == 0) continue;  // marker itself
      // tolerate legacy "folder//file" -> skip the extra '/'
      if (*rest == '/') ++rest;
      if (*rest == 0) continue;
      const char* slash = strchr(rest, '/');
      if (!slash) {
        Console.print("  ");
        Console.print(rest);
        Console.print("  (");
        Console.print(idx[i].size);
        Console.println(" bytes)");
      } else {
        // if the first char is a slash, skip this malformed entry
        if (slash == rest) continue;
        size_t segLen = (size_t)(slash - rest);
        char seg[ActiveFS::MAX_NAME + 1];
        if (segLen > ActiveFS::MAX_NAME) segLen = ActiveFS::MAX_NAME;
        memcpy(seg, rest, segLen);
        seg[segLen] = 0;
        if (seenPush(seg)) {
          Console.print("  ");
          Console.print(seg);
          Console.println("/");
        }
      }
    }
  } else if (!strcmp(t0, "lsdebug")) {
    char* nstr;
    uint32_t n = 256;
    if (nextToken(p, nstr)) n = (uint32_t)strtoul(nstr, nullptr, 0);
    dumpDirHeadRaw(n);
  } else if (!strcmp(t0, "rmdir")) {
    // rmdir <path> [-r]
    char* arg;
    if (!nextToken(p, arg)) {
      Console.println("usage: rmdir <name|path> [-r]");
      return;
    }
    bool recursive = false;
    char* opt;
    if (nextToken(p, opt) && !strcmp(opt, "-r")) recursive = true;
    char folder[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(folder, sizeof(folder), g_cwd, arg, /*wantTrailingSlash*/ true)) {
      Console.println("rmdir: path too long");
      return;
    }
    // Build index to find children under prefix
    FsIndexEntry idx[64];
    size_t n = buildFsIndex(idx, 64);
    if (n == 0) {
      Console.println("(ls fallback) Directory scan empty; printing files view:");
      activeFs.listFilesToSerial();
      return;
    }
    bool empty = true;
    for (size_t i = 0; i < n; ++i) {
      if (idx[i].deleted) continue;
      if (hasPrefix(idx[i].name, folder)) {
        // ignore the folder marker itself
        if (strlen(idx[i].name) == strlen(folder)) continue;
        empty = false;
        break;
      }
    }
    if (!empty && !recursive) {
      Console.println("rmdir: not empty (use -r to remove all files under folder)");
      return;
    }
    // If recursive, delete everything with the prefix
    if (recursive) {
      size_t delCount = 0;
      for (size_t i = 0; i < n; ++i) {
        if (idx[i].deleted) continue;
        if (hasPrefix(idx[i].name, folder)) {
          if (activeFs.deleteFile(idx[i].name)) delCount++;
          yield();
        }
      }
      Console.print("rmdir -r: deleted ");
      Console.print((unsigned)delCount);
      Console.println(" entries");
      return;
    }
    // Empty: remove just the marker if it exists
    if (folderExists(folder)) {
      if (activeFs.deleteFile(folder)) Console.println("rmdir: ok");
      else Console.println("rmdir: failed to remove marker");
    } else {
      Console.println("rmdir: marker not found (already removed?)");
    }
  } else if (!strcmp(t0, "touch")) {
    char* pathArg;
    if (!nextToken(p, pathArg)) {
      Console.println("usage: touch <path|name|folder/>");
      return;
    }
    if (touchPath(g_cwd, pathArg)) {
      Console.println("ok");
    } else {
      Console.println("touch failed");
    }
  } else if (!strcmp(t0, "df")) {
    cmdDf();
  } else if (!strcmp(t0, "mv")) {
    char* srcArg;
    char* dstArg;
    if (!nextToken(p, srcArg) || !nextToken(p, dstArg)) {
      Console.println("usage: mv <src> <dst|folder/>");
      return;
    }
    if (!cmdMvImpl(g_cwd, srcArg, dstArg)) {
      Console.println("mv failed");
    }
  } else if (!strcmp(t0, "lsraw")) {
    FsIndexEntry idx[64];
    size_t n = buildFsIndex(idx, 64);
    for (size_t i = 0; i < n; ++i) {
      if (idx[i].deleted) continue;
      Console.printf("- %s  (%lu bytes)\n", idx[i].name, (unsigned long)idx[i].size);
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
  Console.println("System booting..");
  Console.begin();
  uniMem.begin();
  uniMem.setPreservePsramContents(true);
  uniMem.setCsList({ PIN_FLASH_CS, PIN_PSRAM_CS0, PIN_PSRAM_CS1, PIN_PSRAM_CS2, PIN_PSRAM_CS3 });
  size_t found = uniMem.rescan();
  Console.printf("Unified scan: found %u device(s)\n", (unsigned)found);
  for (size_t i = 0; i < uniMem.detectedCount(); ++i) {
    const auto* di = uniMem.detectedInfo(i);
    if (!di) continue;
    Console.printf("  CS=%u  \tType=%s \tVendor=%s \tCap=%llu bytes\n", di->cs, UnifiedSpiMem::deviceTypeName(di->type), di->vendorName, (unsigned long long)di->capacityBytes);
  }
  // Open both handles (reservations handled internally)
  bool flashOk = fsFlash.begin(uniMem);  // or MX35UnifiedSimpleFS if using SPI-NAND
  bool psramOk = fsPSRAM.begin(uniMem);
  if (!flashOk) Console.println("Flash FS: no suitable device found or open failed");
  if (!psramOk) Console.println("PSRAM FS: no suitable device found or open failed");
  // IMPORTANT: Bind activeFs before using it anywhere
  bindActiveFs(g_storage);
  updateExecFsTable();
  // Mount through activeFs (keeps table + handle consistent)
  bool mounted = activeFs.mount(g_storage == StorageBackend::Flash /*autoFormatIfEmpty*/);
  if (!mounted) {
    Console.println("FS mount failed on active storage");
  }
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) BLOB_MAILBOX[i] = 0;
  Exec.attachConsole(&Console);
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
    handleCommand(lineBuf);
    Console.print("> ");
  }
}