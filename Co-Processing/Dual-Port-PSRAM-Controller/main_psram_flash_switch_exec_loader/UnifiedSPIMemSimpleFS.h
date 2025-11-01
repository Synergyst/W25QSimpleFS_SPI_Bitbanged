#pragma once
/*
  UnifiedSPIMemSimpleFS.h
  - Single-header SimpleFS on top of UnifiedSPIMem (PSRAM, NOR W25Q, MX35LF SPI-NAND)
  - Provides a drop-in SimpleFS-style API similar to PSRAMSimpleFS_Generic
  - Exposes convenience endpoints:
      * PSRAM-only
      * W25Q-only (NOR)
      * MX35LF-only (SPI-NAND)
    and a raw endpoint that accepts an already-open UnifiedSpiMem::MemDevice*
  - Omits any 74HC-series features (no external decoder content)
  Notes:
    - The SimpleFS core is append-only directory + linear data region:
        DIR: 64 KiB at 0x000000, entries of 32 bytes (NOR/PSRAM) or 1 NAND page each (NAND)
        DATA: starts at 0x00010000
    - NOR/NAND specifics:
        * Erasing is required before programming (NOR: 4K sectors, NAND: block size)
        * This layer auto-detects and performs erases when writing:
            - If the payload is all 0xFF: it issues eraseRange() on the covered range.
            - If the payload is not all 0xFF:
                - For DIR writes: it assumes the destination bytes are already erased (0xFF).
                  If they are not, write fails (to avoid erasing earlier directory entries).
                - For DATA writes: if any byte is not erased, it erases the covering range before programming.
        * For NAND (MX35LF): the directory entry is written as one full page at a time
          (the 32-byte record is placed at page start, rest 0xFF) to avoid partial-page program limits.
    - For PSRAM: raw writes are used (no erase).
  Usage (PSRAM example):
    UnifiedSpiMem::Manager mgr(SCK, MOSI, MISO);
    mgr.begin();
    mgr.scan({ cs_psram }); // or mgr.scan({ ... multiple CS ... });
    UnifiedSPIMemSimpleFS psramFS;
    if (psramFS.beginAutoPSRAM(mgr)) {
      psramFS.mount(true);
      const char* name = "hello.txt";
      const uint8_t data[] = "Hello!";
      psramFS.writeFile(name, data, sizeof(data)-1);
      psramFS.listFilesToSerial();
    }
  License: MIT (use freely; attribution appreciated)
*/
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "UnifiedSPIMem.h"

// -------------------------------------------
// UnifiedSPIMem driver adapter for SimpleFS
// -------------------------------------------
class UnifiedMemFSDriver {
public:
  using DeviceType = UnifiedSpiMem::DeviceType;
  UnifiedMemFSDriver()
    : _dev(nullptr), _type(DeviceType::Unknown), _eraseSize(0) {}
  explicit UnifiedMemFSDriver(UnifiedSpiMem::MemDevice* dev) {
    attach(dev);
  }
  void attach(UnifiedSpiMem::MemDevice* dev) {
    _dev = dev;
    _type = _dev ? _dev->type() : DeviceType::Unknown;
    _eraseSize = _dev ? _dev->eraseSize() : 0;
  }
  UnifiedSpiMem::MemDevice* device() const {
    return _dev;
  }
  DeviceType deviceType() const {
    return _type;
  }
  uint32_t eraseSize() const {
    return _eraseSize;
  }
  uint32_t pageSize() const {
    return _dev ? _dev->pageSize() : 256u;
  }
  bool eraseRange(uint64_t addr, uint64_t len) {
    if (!_dev) return false;
    if (_eraseSize == 0) return false;
    return _dev->eraseRange(addr, len);
  }
  // SimpleFS expects these methods:
  bool readData03(uint32_t addr, uint8_t* buf, size_t len) {
    if (!_dev || !buf || len == 0) return true;
    size_t r = _dev->read((uint64_t)addr, buf, len);
    return r == len;
  }
  bool writeData02(uint32_t addr, const uint8_t* buf, size_t len, bool /*needsWriteEnable*/ = false) {
    if (!_dev || !buf || len == 0) return true;
    switch (_type) {
      case DeviceType::Psram:
        // No erase required
        return _dev->write((uint64_t)addr, buf, len);
      case DeviceType::NorW25Q:
      case DeviceType::SpiNandMX35:
        return writeWithErasePolicy(addr, buf, len);
      default:
        // Unknown: best effort raw write
        return _dev->write((uint64_t)addr, buf, len);
    }
  }
  const char* styleName() const {
    return UnifiedSpiMem::deviceTypeName(_type);
  }
  uint8_t cs() const {
    return _dev ? _dev->cs() : 0xFF;
  }
  uint64_t capacityBytes() const {
    return _dev ? _dev->capacity() : 0;
  }
private:
  static constexpr uint32_t DIR_START = 0x000000UL;
  static constexpr uint32_t DIR_SIZE = 64UL * 1024UL;
  static constexpr uint32_t DATA_START = DIR_START + DIR_SIZE;
  static inline bool isAllFF(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; ++i)
      if (p[i] != 0xFF) return false;
    return true;
  }
  static inline uint64_t alignDown(uint64_t v, uint64_t a) {
    return v & ~(a - 1);
  }
  static inline uint64_t alignUp64(uint64_t v, uint64_t a) {
    return (v + (a - 1)) & ~(a - 1);
  }
  bool regionIsErased(uint32_t addr, size_t len) {
    if (!_dev || len == 0) return true;
    // Read-chunk scan for any non-0xFF
    uint8_t tmp[256];
    uint64_t pos = addr;
    uint64_t end = (uint64_t)addr + (uint64_t)len;
    while (pos < end) {
      size_t n = (size_t)min<uint64_t>(sizeof(tmp), end - pos);
      size_t r = _dev->read(pos, tmp, n);
      if (r != n) return false;  // I/O fail treated as non-erased
      for (size_t i = 0; i < n; ++i) {
        if (tmp[i] != 0xFF) return false;
      }
      pos += n;
    }
    return true;
  }
  bool writeWithErasePolicy(uint32_t addr, const uint8_t* buf, size_t len) {
    // If caller writes "all 0xFF", we translate to erase on erase-capable devices.
    if (_eraseSize > 0 && isAllFF(buf, len)) {
      uint64_t start = alignDown((uint64_t)addr, (uint64_t)_eraseSize);
      uint64_t end = alignUp64((uint64_t)addr + (uint64_t)len, (uint64_t)_eraseSize);
      uint64_t elen = (end > start) ? (end - start) : 0;
      if (elen) {
        if (!eraseRange(start, elen)) return false;
      }
      return true;
    }
    // For non-FF payload:
    const bool inDir = (addr < DATA_START);
    if (_eraseSize > 0) {
      if (inDir) {
        // Directory writes MUST target previously erased (0xFF) space.
        // Avoid erasing to preserve prior directory entries.
        if (!regionIsErased(addr, len)) {
          return false;
        }
      } else {
        // DATA region: ensure erased. If not, erase the covering range.
        if (!regionIsErased(addr, len)) {
          uint64_t start = alignDown((uint64_t)addr, (uint64_t)_eraseSize);
          uint64_t end = alignUp64((uint64_t)addr + (uint64_t)len, (uint64_t)_eraseSize);
          uint64_t elen = (end > start) ? (end - start) : 0;
          if (elen) {
            if (!eraseRange(start, elen)) return false;
          }
        }
      }
    }
    return _dev->write((uint64_t)addr, buf, len);
  }
  UnifiedSpiMem::MemDevice* _dev;
  DeviceType _type;
  uint32_t _eraseSize;
};

// -------------------------------------------
/* SimpleFS core (generic, header-only)
   NAND-safe:
   - On MX35LF (SPI-NAND) the directory uses one full page per entry to avoid
     partial-page program limits. The 32-byte entry is stored at the beginning
     of that page; the rest is 0xFF. The scanner steps by page size. */
template<typename Driver>
class UnifiedSimpleFS_Generic {
public:
  static const uint32_t DIR_START = 0x000000UL;
  static const uint32_t DIR_SIZE = 64UL * 1024UL;
  static const uint32_t ENTRY_SIZE = 32;  // logical entry size
  static const uint32_t DATA_START = DIR_START + DIR_SIZE;
  static const size_t MAX_NAME = 32;
  enum class WriteMode : uint8_t {
    ReplaceIfExists = 0,
    FailIfExists = 1
  };
  struct FileInfo {
    char name[MAX_NAME + 1];
    uint32_t addr;
    uint32_t size;
    uint32_t seq;
    bool deleted;
    uint32_t capEnd;
    bool slotSafe;
  };
  UnifiedSimpleFS_Generic(Driver& dev, uint32_t capacityBytes)
    : _dev(dev), _capacity(capacityBytes) {
    _fileCount = 0;
    _dirWriteOffset = 0;
    _nextSeq = 1;
    _dataHead = DATA_START;
    // Runtime params (init lazily)
    _paramsInit = false;
    _isNand = false;
    _eraseAlign = 1;
    _nandPage = ENTRY_SIZE;
    _dirStride = ENTRY_SIZE;
    _dirScratch = nullptr;
    _lastSeqWritten = 0;
  }
  ~UnifiedSimpleFS_Generic() {
    if (_dirScratch) {
      delete[] _dirScratch;
      _dirScratch = nullptr;
    }
  }
  bool mount(bool autoFormatIfEmpty = true) {
    if (_capacity == 0 || _capacity <= DATA_START) return false;
    ensureParams();
    _fileCount = 0;
    _dirWriteOffset = 0;
    _nextSeq = 1;
    _dataHead = DATA_START;
    uint32_t maxEnd = DATA_START;
    uint32_t maxSeq = 0;
    bool sawAny = false;

    const uint32_t stride = _dirStride;  // 32 for NOR/PSRAM, pageSize for NAND
    const uint32_t entries = DIR_SIZE / stride;
    uint8_t buf[ENTRY_SIZE];

    for (uint32_t i = 0; i < entries; ++i) {
      uint32_t addr = DIR_START + i * stride;
      // Only read logical entry header (32 bytes)
      if (!_dev.readData03(addr, buf, ENTRY_SIZE)) {
        // Read error: treat as empty and stop scanning to avoid corruption
        _dirWriteOffset = i * stride;
        break;
      }
      if (isAllFF(buf, ENTRY_SIZE)) {
        _dirWriteOffset = i * stride;
        break;
      }
      sawAny = true;
      if (buf[0] != 0x57 || buf[1] != 0x46) continue;
      uint8_t flags = buf[2];
      uint8_t nameLen = buf[3];
      if (nameLen == 0 || nameLen > MAX_NAME) continue;
      char nameBuf[MAX_NAME + 1];
      memset(nameBuf, 0, sizeof(nameBuf));
      for (uint8_t k = 0; k < nameLen; ++k) nameBuf[k] = (char)buf[4 + k];
      uint32_t faddr = rd32(&buf[20]);
      uint32_t fsize = rd32(&buf[24]);
      uint32_t seq = rd32(&buf[28]);
      if (seq > maxSeq) maxSeq = seq;
      int idx = findIndexByName(nameBuf);
      if (idx < 0) {
        if (_fileCount < MAX_FILES) {
          idx = _fileCount++;
          copyName(_files[idx].name, nameBuf);
        } else {
          continue;
        }
      }
      bool deleted = (flags & 0x01) != 0;
      _files[idx].seq = seq;
      _files[idx].deleted = deleted;
      if (!deleted) {
        _files[idx].addr = faddr;
        _files[idx].size = fsize;
        uint32_t end = faddr + fsize;
        if (end > maxEnd) maxEnd = end;
      } else {
        _files[idx].addr = 0;
        _files[idx].size = 0;
      }
      if (i == entries - 1) _dirWriteOffset = DIR_SIZE;
    }
    if (!sawAny) {
      _dirWriteOffset = 0;
      if (autoFormatIfEmpty) format();
    }
    _nextSeq = maxSeq + 1;
    if (_nextSeq == 0) _nextSeq = 1;
    _dataHead = maxEnd;
    computeCapacities(_dataHead);
    return true;
  }
  bool format() {
    ensureParams();
    // Fill DIR with 0xFF (driver will translate to erase on NOR/NAND)
    const uint32_t PAGE_CHUNK = 256;
    uint8_t tmp[PAGE_CHUNK];
    memset(tmp, 0xFF, PAGE_CHUNK);
    for (uint32_t i = 0; i < DIR_SIZE; i += PAGE_CHUNK) {
      uint32_t chunk = (i + PAGE_CHUNK <= DIR_SIZE) ? PAGE_CHUNK : (DIR_SIZE - i);
      if (!_dev.writeData02(DIR_START + i, tmp, chunk)) return false;
    }
    _fileCount = 0;
    _dirWriteOffset = 0;
    _nextSeq = 1;
    _dataHead = DATA_START;
    computeCapacities(_dataHead);
    return true;
  }
  bool wipeChip() {
    ensureParams();
    if (_capacity == 0) return false;
    if (_eraseAlign > 1) {
      // Fast path: use erase units across the device
      uint64_t pos = 0;
      while (pos < _capacity) {
        uint64_t remain = _capacity - pos;
        uint64_t n = (remain >= _eraseAlign) ? _eraseAlign : remain;
        if (!_dev.eraseRange(pos, n)) return false;
        pos += n;
      }
    } else {
      // PSRAM: write 0xFF
      const uint32_t CHUNK = 256;
      uint8_t tmp[CHUNK];
      memset(tmp, 0xFF, CHUNK);
      uint32_t pos = 0;
      while (pos < _capacity) {
        uint32_t n = (pos + CHUNK <= _capacity) ? CHUNK : (_capacity - pos);
        if (!_dev.writeData02(pos, tmp, n)) return false;
        pos += n;
      }
    }
    _fileCount = 0;
    _dirWriteOffset = 0;
    _dataHead = DATA_START;
    computeCapacities(_dataHead);
    return true;
  }
  bool writeFile(const char* name, const uint8_t* data, uint32_t size, WriteMode mode = WriteMode::ReplaceIfExists) {
    ensureParams();
    if (!validName(name) || size > 0xFFFFFFUL) return false;
    if (_dirWriteOffset + _dirStride > DIR_SIZE) return false;
    int idxExisting = findIndexByName(name);
    bool exists = (idxExisting >= 0 && !_files[idxExisting].deleted);
    if (exists && mode == WriteMode::FailIfExists) return false;

    uint32_t start = _dataHead;
    if (start < DATA_START) start = DATA_START;
    if (start + size > _capacity) return false;

    if (size > 0) {
      if (!_dev.writeData02(start, data, size)) return false;
    }

    uint32_t seq = 0;
    if (!appendDirEntry(0x00, name, start, size, seq)) return false;
    upsertFileIndex(name, start, size, false, seq);
    _dataHead = start + size;
    computeCapacities(_dataHead);
    return true;
  }
  bool writeFile(const char* name, const uint8_t* data, uint32_t size, int modeInt) {
    WriteMode m = (modeInt == (int)WriteMode::FailIfExists) ? WriteMode::FailIfExists : WriteMode::ReplaceIfExists;
    return writeFile(name, data, size, m);
  }
  template<typename ModeT>
  bool writeFile(const char* name, const uint8_t* data, uint32_t size, ModeT modeOther) {
    return writeFile(name, data, size, (int)modeOther);
  }
  bool createFileSlot(const char* name, uint32_t reserveBytes, const uint8_t* initialData = nullptr, uint32_t initialSize = 0) {
    ensureParams();
    if (!validName(name)) return false;
    if (_dirWriteOffset + _dirStride > DIR_SIZE) return false;
    if (initialSize > reserveBytes) return false;
    if (exists(name)) return false;

    // Align capacity and start to erase alignment if erase is needed
    uint32_t align = (_eraseAlign > 1) ? _eraseAlign : 1u;
    uint32_t cap = alignUp((reserveBytes < 1u ? 1u : reserveBytes), align);
    uint32_t start = alignUp(_dataHead, align);
    if (start < DATA_START) start = DATA_START;
    if (start + cap > _capacity) return false;

    // Pre-erase/fill with 0xFF
    if (_eraseAlign > 1) {
      if (!_dev.eraseRange(start, cap)) return false;
    } else {
      // PSRAM fallback
      const uint32_t PAGE_CHUNK = 256;
      uint8_t tmp[PAGE_CHUNK];
      memset(tmp, 0xFF, PAGE_CHUNK);
      uint32_t p = start;
      while (p < start + cap) {
        uint32_t n = min<uint32_t>(PAGE_CHUNK, start + cap - p);
        if (!_dev.writeData02(p, tmp, n)) return false;
        p += n;
      }
    }

    if (initialSize > 0) {
      if (!_dev.writeData02(start, initialData, initialSize)) return false;
    }
    uint32_t seq = 0;
    if (!appendDirEntry(0x00, name, start, initialSize, seq)) return false;
    upsertFileIndex(name, start, initialSize, false, seq);
    _dataHead = start + cap;
    computeCapacities(_dataHead);
    return true;
  }
  bool writeFileInPlace(const char* name, const uint8_t* data, uint32_t size, bool allowReallocate = false) {
    ensureParams();
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;
    FileInfo& fi = _files[idx];
    uint32_t cap = (fi.capEnd > fi.addr) ? (fi.capEnd - fi.addr) : 0;
    if (fi.slotSafe && cap >= size) {
      if (size > 0) {
        if (!_dev.writeData02(fi.addr, data, size)) return false;
      }
      uint32_t seq = 0;
      if (!appendDirEntry(0x00, name, fi.addr, size, seq)) return false;
      fi.size = size;
      fi.seq = seq;
      return true;
    }
    if (!allowReallocate) return false;
    return writeFile(name, data, size, WriteMode::ReplaceIfExists);
  }
  uint32_t readFile(const char* name, uint8_t* buf, uint32_t bufSize) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return 0;
    uint32_t n = _files[idx].size;
    if (bufSize < n) n = bufSize;
    if (n == 0) return 0;
    if (!_dev.readData03(_files[idx].addr, buf, n)) return 0;
    return n;
  }
  uint32_t readFileRange(const char* name, uint32_t offset, uint8_t* buf, uint32_t len) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return 0;
    if (offset >= _files[idx].size) return 0;
    uint32_t maxLen = _files[idx].size - offset;
    if (len > maxLen) len = maxLen;
    if (len == 0) return 0;
    if (!_dev.readData03(_files[idx].addr + offset, buf, len)) return 0;
    return len;
  }
  bool getFileSize(const char* name, uint32_t& sizeOut) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;
    sizeOut = _files[idx].size;
    return true;
  }
  bool getFileInfo(const char* name, uint32_t& addrOut, uint32_t& sizeOut, uint32_t& capOut) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;
    addrOut = _files[idx].addr;
    sizeOut = _files[idx].size;
    capOut = (_files[idx].capEnd > _files[idx].addr) ? (_files[idx].capEnd - _files[idx].addr) : 0;
    return true;
  }
  bool exists(const char* name) {
    int idx = findIndexByName(name);
    return (idx >= 0 && !_files[idx].deleted);
  }
  bool deleteFile(const char* name) {
    ensureParams();
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;
    uint32_t seq = 0;
    if (!appendDirEntry(0x01, name, 0, 0, seq)) return false;
    _files[idx].deleted = true;
    _files[idx].addr = 0;
    _files[idx].size = 0;
    _files[idx].seq = seq;
    computeCapacities(_dataHead);
    return true;
  }
  void listFilesToSerial(Stream& out = Serial) {
    // Device info (style, CS, capacity)
    const char* style = _dev.styleName();
    const uint8_t cs = _dev.cs();
    const uint64_t devCap = _dev.capacityBytes();
    // FS region math (DIR + DATA)
    const uint32_t dirUsed = _dirWriteOffset;
    const uint32_t dirFree = (DIR_SIZE > dirUsed) ? (DIR_SIZE - dirUsed) : 0;
    const uint32_t dataCap = (_capacity > DATA_START) ? (_capacity - DATA_START) : 0;
    uint32_t dataUsed = (_dataHead > DATA_START) ? (_dataHead - DATA_START) : 0;
    if (dataUsed > dataCap) dataUsed = dataCap;
    const uint32_t dataFree = (dataCap > dataUsed) ? (dataCap - dataUsed) : 0;
    auto printPct = [&](uint32_t num, uint32_t den) {
      if (den == 0) {
        out.print("n/a");
        return;
      }
      uint32_t scaled = (uint32_t)(((uint64_t)num * 10000ULL + (den / 2)) / den);  // 2 decimals, rounded
      uint32_t ip = scaled / 100;
      uint32_t fp = scaled % 100;
      out.print(ip);
      out.print('.');
      if (fp < 10) out.print('0');
      out.print(fp);
      out.print('%');
    };
    out.printf("Files (%s, CS=%u, %llu bytes total; FS data=%lu bytes)\n", style, cs, (unsigned long long)devCap, (unsigned long)dataCap);
    out.print("Usage: data used=");
    out.print((unsigned long)dataUsed);
    out.print(" (");
    printPct(dataUsed, dataCap);
    out.print(")  data free=");
    out.print((unsigned long)dataFree);
    out.print(" (");
    printPct(dataFree, dataCap);
    out.println(")");
    out.print("       dir used=");
    out.print((unsigned long)dirUsed);
    out.print(" (");
    printPct(dirUsed, DIR_SIZE);
    out.print(")  dir free=");
    out.print((unsigned long)dirFree);
    out.print(" (");
    printPct(dirFree, DIR_SIZE);
    out.println(")");
    // File list
    for (size_t i = 0; i < _fileCount; ++i) {
      if (_files[i].deleted) continue;
      const char* nm = _files[i].name;
      size_t nlen = strlen(nm);
      bool isFolder = (nlen > 0 && nm[nlen - 1] == '/') && (_files[i].size == 0);
      if (isFolder) {
        out.printf("- %s\t (folder)\n", nm);
        continue;
      }
      out.printf("- %s\t size=%u\t addr=0x%08lX", nm, (unsigned)_files[i].size, (unsigned long)_files[i].addr);
      uint32_t cap = (_files[i].capEnd > _files[i].addr) ? (_files[i].capEnd - _files[i].addr) : 0;
      out.printf("\t cap=%u\t slotSafe=%s\t seq=%u\n", (unsigned)cap, _files[i].slotSafe ? "Y" : "N", (unsigned)_files[i].seq);
    }
  }
  size_t fileCount() const {
    size_t n = 0;
    for (size_t i = 0; i < _fileCount; ++i)
      if (!_files[i].deleted) ++n;
    return n;
  }
  uint32_t nextDataAddr() const {
    return _dataHead;
  }
  uint32_t capacity() const {
    return _capacity;
  }
  uint32_t dataRegionStart() const {
    return DATA_START;
  }
private:
  Driver& _dev;
  uint32_t _capacity;
  static const size_t MAX_FILES = 64;
  FileInfo _files[MAX_FILES];
  size_t _fileCount;
  uint32_t _dirWriteOffset;
  uint32_t _dataHead;
  uint32_t _nextSeq;

  // Runtime parameters
  bool _paramsInit;
  bool _isNand;
  uint32_t _eraseAlign;  // erase unit (1 for PSRAM)
  uint32_t _nandPage;    // NAND page size
  uint32_t _dirStride;   // logical stride between entries (32 or NAND page)
  uint8_t* _dirScratch;  // scratch for writing a full NAND page
  uint32_t _lastSeqWritten;

  // Utilities
  static inline uint32_t rd32(const uint8_t* p) {
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3];
  }
  static inline void wr32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v >> 0);
  }
  static inline uint32_t alignUp(uint32_t v, uint32_t a) {
    return (a > 1) ? ((v + (a - 1)) & ~(a - 1)) : v;
  }
  static bool isAllFF(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; ++i)
      if (p[i] != 0xFF) return false;
    return true;
  }
  static bool validName(const char* name) {
    if (!name) return false;
    size_t n = strlen(name);
    return n >= 1 && n <= MAX_NAME;
  }
  static inline void copyName(char* dst, const char* src) {
    size_t n = strlen(src);
    if (n > MAX_NAME) n = MAX_NAME;
    memcpy(dst, src, n);
    dst[n] = '\0';
  }
  void ensureParams() {
    if (_paramsInit) return;
    auto t = _dev.deviceType();
    _isNand = (t == UnifiedSpiMem::DeviceType::SpiNandMX35);
    uint32_t e = _dev.eraseSize();
    _eraseAlign = (e > 0) ? e : 1u;
    _nandPage = _dev.pageSize();
    if (_isNand) {
      if (_nandPage < 512u || _nandPage > 8192u) _nandPage = 4096u;  // sane default
      _dirStride = _nandPage;
      if (!_dirScratch) {
        _dirScratch = new uint8_t[_dirStride];
      }
    } else {
      _dirStride = ENTRY_SIZE;
    }
    if (_dirScratch) memset(_dirScratch, 0xFF, _dirStride);
    _paramsInit = true;
  }
  int findIndexByName(const char* name) const {
    for (size_t i = 0; i < _fileCount; ++i)
      if (strncmp(_files[i].name, name, MAX_NAME) == 0) return (int)i;
    return -1;
  }
  void upsertFileIndex(const char* name, uint32_t addr, uint32_t size, bool deleted, uint32_t seq) {
    int idx = findIndexByName(name);
    if (idx < 0) {
      if (_fileCount >= MAX_FILES) return;
      idx = _fileCount++;
      copyName(_files[idx].name, name);
    }
    _files[idx].addr = addr;
    _files[idx].size = size;
    _files[idx].deleted = deleted;
    _files[idx].seq = seq;
  }
  bool appendDirEntry(uint8_t flags, const char* name, uint32_t addr, uint32_t size, uint32_t& outSeq) {
    ensureParams();
    outSeq = 0;
    if (!validName(name)) return false;
    if (_dirWriteOffset + _dirStride > DIR_SIZE) return false;

    // Prepare logical record (32 bytes)
    uint8_t rec[ENTRY_SIZE];
    memset(rec, 0xFF, ENTRY_SIZE);
    rec[0] = 0x57;
    rec[1] = 0x46;
    rec[2] = flags;
    uint8_t nameLen = (uint8_t)min((size_t)MAX_NAME, strlen(name));
    rec[3] = nameLen;
    for (uint8_t i = 0; i < nameLen; ++i) rec[4 + i] = (uint8_t)name[i];

    // Assign sequence and stamp it (increment once on success)
    uint32_t seq = _nextSeq;
    wr32(&rec[20], addr);
    wr32(&rec[24], size);
    wr32(&rec[28], seq);

    bool ok = false;
    const uint32_t dest = DIR_START + _dirWriteOffset;
    if (_isNand) {
      // NAND: write one full page with rec at start, rest 0xFF
      memset(_dirScratch, 0xFF, _dirStride);
      memcpy(_dirScratch, rec, ENTRY_SIZE);
      ok = _dev.writeData02(dest, _dirScratch, _dirStride);
      if (ok) _dirWriteOffset += _dirStride;
    } else {
      // NOR/PSRAM: write just the record (32 bytes)
      ok = _dev.writeData02(dest, rec, ENTRY_SIZE);
      if (ok) _dirWriteOffset += ENTRY_SIZE;
    }
    if (!ok) return false;
    _lastSeqWritten = seq;
    _nextSeq = (_nextSeq == 0xFFFFFFFFu) ? 1u : (_nextSeq + 1u);
    outSeq = _lastSeqWritten;
    return true;
  }
  void computeCapacities(uint32_t maxEnd) {
    ensureParams();
    int idxs[MAX_FILES];
    size_t n = 0;
    for (size_t i = 0; i < _fileCount; ++i)
      if (!_files[i].deleted) idxs[n++] = (int)i;
    // insertion sort by start address
    for (size_t i = 1; i < n; ++i) {
      int key = idxs[i];
      size_t j = i;
      while (j > 0 && _files[idxs[j - 1]].addr > _files[key].addr) {
        idxs[j] = idxs[j - 1];
        --j;
      }
      idxs[j] = key;
    }
    uint32_t align = (_eraseAlign > 1) ? _eraseAlign : 1u;
    for (size_t i = 0; i < n; ++i) {
      FileInfo& fi = _files[idxs[i]];
      uint32_t nextStart = (i + 1 < n) ? _files[idxs[i + 1]].addr : alignUp(maxEnd, align);
      fi.capEnd = nextStart;
      fi.slotSafe = ((fi.addr % align) == 0) && ((fi.capEnd % align) == 0) && (fi.capEnd > fi.addr);
    }
  }
};

// -------------------------------------------
// UnifiedSPIMemSimpleFS convenience facade
// -------------------------------------------
class UnifiedSPIMemSimpleFS {
public:
  using DeviceType = UnifiedSpiMem::DeviceType;
  using WriteMode = typename UnifiedSimpleFS_Generic<UnifiedMemFSDriver>::WriteMode;
  UnifiedSPIMemSimpleFS()
    : _mgr(nullptr), _handle(nullptr), _ownsHandle(false),
      _fs(nullptr), _capacity32(0) {}
  ~UnifiedSPIMemSimpleFS() {
    close();
  }
  // Open helpers
  bool beginWithDevice(UnifiedSpiMem::MemDevice* dev, bool takeOwnership = false) {
    close();
    if (!dev) return false;
    _handle = dev;
    _ownsHandle = takeOwnership;
    _driver.attach(_handle);
    _capacity32 = (uint32_t)min<uint64_t>(_handle->capacity(), 0xFFFFFFFFull);
    _fs = new UnifiedSimpleFS_Generic<UnifiedMemFSDriver>(_driver, _capacity32);
    return true;
  }
  bool beginAutoPSRAM(UnifiedSpiMem::Manager& mgr) {
    return beginByType(mgr, DeviceType::Psram);
  }
  bool beginAutoNOR(UnifiedSpiMem::Manager& mgr) {
    return beginByType(mgr, DeviceType::NorW25Q);
  }
  bool beginAutoMX35(UnifiedSpiMem::Manager& mgr) {
    return beginByType(mgr, DeviceType::SpiNandMX35);
  }
  // Mount/format/etc (forwarded to FS)
  bool mount(bool autoFormatIfEmpty = true) {
    if (!_fs) return false;
    return _fs->mount(autoFormatIfEmpty);
  }
  bool format() {
    if (!_fs) return false;
    return _fs->format();
  }
  bool wipeChip() {
    if (!_fs) return false;
    return _fs->wipeChip();
  }
  bool writeFile(const char* name, const uint8_t* data, uint32_t size, WriteMode mode = WriteMode::ReplaceIfExists) {
    if (!_fs) return false;
    return _fs->writeFile(name, data, size, mode);
  }
  bool writeFile(const char* name, const uint8_t* data, uint32_t size, int modeInt) {
    if (!_fs) return false;
    return _fs->writeFile(name, data, size, modeInt);
  }
  template<typename ModeT>
  bool writeFile(const char* name, const uint8_t* data, uint32_t size, ModeT modeOther) {
    if (!_fs) return false;
    return _fs->template writeFile<ModeT>(name, data, size, modeOther);
  }
  bool createFileSlot(const char* name, uint32_t reserveBytes, const uint8_t* initialData = nullptr, uint32_t initialSize = 0) {
    if (!_fs) return false;
    return _fs->createFileSlot(name, reserveBytes, initialData, initialSize);
  }
  bool writeFileInPlace(const char* name, const uint8_t* data, uint32_t size, bool allowReallocate = false) {
    if (!_fs) return false;
    return _fs->writeFileInPlace(name, data, size, allowReallocate);
  }
  uint32_t readFile(const char* name, uint8_t* buf, uint32_t bufSize) {
    if (!_fs) return 0;
    return _fs->readFile(name, buf, bufSize);
  }
  uint32_t readFileRange(const char* name, uint32_t offset, uint8_t* buf, uint32_t len) {
    if (!_fs) return 0;
    return _fs->readFileRange(name, offset, buf, len);
  }
  bool getFileSize(const char* name, uint32_t& sizeOut) {
    if (!_fs) return false;
    return _fs->getFileSize(name, sizeOut);
  }
  bool getFileInfo(const char* name, uint32_t& addrOut, uint32_t& sizeOut, uint32_t& capOut) {
    if (!_fs) return false;
    return _fs->getFileInfo(name, addrOut, sizeOut, capOut);
  }
  bool exists(const char* name) {
    if (!_fs) return false;
    return _fs->exists(name);
  }
  bool deleteFile(const char* name) {
    if (!_fs) return false;
    return _fs->deleteFile(name);
  }
  void listFilesToSerial(Stream& out = Serial) {
    if (!_fs) return;
    _fs->listFilesToSerial(out);
  }
  size_t fileCount() const {
    if (!_fs) return 0;
    return _fs->fileCount();
  }
  uint32_t nextDataAddr() const {
    if (!_fs) return 0;
    return _fs->nextDataAddr();
  }
  uint32_t capacity() const {
    if (!_fs) return 0;
    return _fs->capacity();
  }
  uint32_t dataRegionStart() const {
    if (!_fs) return UnifiedSimpleFS_Generic<UnifiedMemFSDriver>::DATA_START;
    return _fs->dataRegionStart();
  }
  // Accessors
  UnifiedSpiMem::MemDevice* device() const {
    return _handle;
  }
  UnifiedSpiMem::DeviceType deviceType() const {
    return _driver.deviceType();
  }
  // Release resources (and reservation if managed by Manager)
  void close() {
    if (_fs) {
      delete _fs;
      _fs = nullptr;
    }
    if (_mgr && _handle) {
      _mgr->release(_handle);
      _handle = nullptr;
      _mgr = nullptr;
      _ownsHandle = false;
    } else if (_ownsHandle && _handle) {
      delete _handle;
      _handle = nullptr;
      _ownsHandle = false;
    } else {
      _handle = nullptr;
    }
    _capacity32 = 0;
  }
private:
  bool beginByType(UnifiedSpiMem::Manager& mgr, DeviceType t) {
    close();
    _mgr = &mgr;
    UnifiedSpiMem::MemDevice* dev = mgr.openPreferred(t);
    if (!dev) {
      _mgr = nullptr;
      return false;
    }
    _handle = dev;
    _ownsHandle = false;  // Owned by Manager reservation
    _driver.attach(_handle);
    _capacity32 = (uint32_t)min<uint64_t>(_handle->capacity(), 0xFFFFFFFFull);
    _fs = new UnifiedSimpleFS_Generic<UnifiedMemFSDriver>(_driver, _capacity32);
    return true;
  }
  UnifiedSpiMem::Manager* _mgr;
  UnifiedSpiMem::MemDevice* _handle;
  bool _ownsHandle;
  UnifiedMemFSDriver _driver;
  UnifiedSimpleFS_Generic<UnifiedMemFSDriver>* _fs;
  uint32_t _capacity32;
};

// -------------------------------------------
// Type-specific convenience facades (matching endpoints)
// -------------------------------------------
class PSRAMUnifiedSimpleFS {
public:
  using WriteMode = typename UnifiedSimpleFS_Generic<UnifiedMemFSDriver>::WriteMode;
  PSRAMUnifiedSimpleFS() {}
  bool begin(UnifiedSpiMem::Manager& mgr) {
    return _core.beginAutoPSRAM(mgr);
  }
  // Forwarders
  bool mount(bool autoFormatIfEmpty = true) {
    return _core.mount(autoFormatIfEmpty);
  }
  bool format() {
    return _core.format();
  }
  bool wipeChip() {
    return _core.wipeChip();
  }
  bool writeFile(const char* n, const uint8_t* d, uint32_t s, WriteMode m = WriteMode::ReplaceIfExists) {
    return _core.writeFile(n, d, s, m);
  }
  bool writeFile(const char* n, const uint8_t* d, uint32_t s, int m) {
    return _core.writeFile(n, d, s, m);
  }
  template<typename ModeT> bool writeFile(const char* n, const uint8_t* d, uint32_t s, ModeT mo) {
    return _core.writeFile<ModeT>(n, d, s, mo);
  }
  bool createFileSlot(const char* n, uint32_t r, const uint8_t* id = nullptr, uint32_t isz = 0) {
    return _core.createFileSlot(n, r, id, isz);
  }
  bool writeFileInPlace(const char* n, const uint8_t* d, uint32_t s, bool ar = false) {
    return _core.writeFileInPlace(n, d, s, ar);
  }
  uint32_t readFile(const char* n, uint8_t* b, uint32_t bs) {
    return _core.readFile(n, b, bs);
  }
  uint32_t readFileRange(const char* n, uint32_t off, uint8_t* b, uint32_t l) {
    return _core.readFileRange(n, off, b, l);
  }
  bool getFileSize(const char* n, uint32_t& so) {
    return _core.getFileSize(n, so);
  }
  bool getFileInfo(const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
    return _core.getFileInfo(n, a, s, c);
  }
  bool exists(const char* n) {
    return _core.exists(n);
  }
  bool deleteFile(const char* n) {
    return _core.deleteFile(n);
  }
  void listFilesToSerial(Stream& out = Serial) {
    _core.listFilesToSerial(out);
  }
  size_t fileCount() const {
    return _core.fileCount();
  }
  uint32_t nextDataAddr() const {
    return _core.nextDataAddr();
  }
  uint32_t capacity() const {
    return _core.capacity();
  }
  uint32_t dataRegionStart() const {
    return _core.dataRegionStart();
  }
  void close() {
    _core.close();
  }
  UnifiedSPIMemSimpleFS& raw() {
    return _core;
  }
private:
  UnifiedSPIMemSimpleFS _core;
};
class W25QUnifiedSimpleFS {
public:
  using WriteMode = typename UnifiedSimpleFS_Generic<UnifiedMemFSDriver>::WriteMode;
  W25QUnifiedSimpleFS() {}
  bool begin(UnifiedSpiMem::Manager& mgr) {
    return _core.beginAutoNOR(mgr);
  }
  // Forwarders
  bool mount(bool autoFormatIfEmpty = true) {
    return _core.mount(autoFormatIfEmpty);
  }
  bool format() {
    return _core.format();
  }
  bool wipeChip() {
    return _core.wipeChip();
  }
  bool writeFile(const char* n, const uint8_t* d, uint32_t s, WriteMode m = WriteMode::ReplaceIfExists) {
    return _core.writeFile(n, d, s, m);
  }
  bool writeFile(const char* n, const uint8_t* d, uint32_t s, int m) {
    return _core.writeFile(n, d, s, m);
  }
  template<typename ModeT> bool writeFile(const char* n, const uint8_t* d, uint32_t s, ModeT mo) {
    return _core.writeFile<ModeT>(n, d, s, mo);
  }
  bool createFileSlot(const char* n, uint32_t r, const uint8_t* id = nullptr, uint32_t isz = 0) {
    return _core.createFileSlot(n, r, id, isz);
  }
  bool writeFileInPlace(const char* n, const uint8_t* d, uint32_t s, bool ar = false) {
    return _core.writeFileInPlace(n, d, s, ar);
  }
  uint32_t readFile(const char* n, uint8_t* b, uint32_t bs) {
    return _core.readFile(n, b, bs);
  }
  uint32_t readFileRange(const char* n, uint32_t off, uint8_t* b, uint32_t l) {
    return _core.readFileRange(n, off, b, l);
  }
  bool getFileSize(const char* n, uint32_t& so) {
    return _core.getFileSize(n, so);
  }
  bool getFileInfo(const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
    return _core.getFileInfo(n, a, s, c);
  }
  bool exists(const char* n) {
    return _core.exists(n);
  }
  bool deleteFile(const char* n) {
    return _core.deleteFile(n);
  }
  void listFilesToSerial(Stream& out = Serial) {
    _core.listFilesToSerial(out);
  }
  size_t fileCount() const {
    return _core.fileCount();
  }
  uint32_t nextDataAddr() const {
    return _core.nextDataAddr();
  }
  uint32_t capacity() const {
    return _core.capacity();
  }
  uint32_t dataRegionStart() const {
    return _core.dataRegionStart();
  }
  void close() {
    _core.close();
  }
  UnifiedSPIMemSimpleFS& raw() {
    return _core;
  }
private:
  UnifiedSPIMemSimpleFS _core;
};
class MX35UnifiedSimpleFS {
public:
  using WriteMode = typename UnifiedSimpleFS_Generic<UnifiedMemFSDriver>::WriteMode;
  MX35UnifiedSimpleFS() {}
  bool begin(UnifiedSpiMem::Manager& mgr) {
    return _core.beginAutoMX35(mgr);
  }
  // Forwarders
  bool mount(bool autoFormatIfEmpty = true) {
    return _core.mount(autoFormatIfEmpty);
  }
  bool format() {
    return _core.format();
  }
  bool wipeChip() {
    return _core.wipeChip();
  }
  bool writeFile(const char* n, const uint8_t* d, uint32_t s, WriteMode m = WriteMode::ReplaceIfExists) {
    return _core.writeFile(n, d, s, m);
  }
  bool writeFile(const char* n, const uint8_t* d, uint32_t s, int m) {
    return _core.writeFile(n, d, s, m);
  }
  template<typename ModeT> bool writeFile(const char* n, const uint8_t* d, uint32_t s, ModeT mo) {
    return _core.writeFile<ModeT>(n, d, s, mo);
  }
  bool createFileSlot(const char* n, uint32_t r, const uint8_t* id = nullptr, uint32_t isz = 0) {
    return _core.createFileSlot(n, r, id, isz);
  }
  bool writeFileInPlace(const char* n, const uint8_t* d, uint32_t s, bool ar = false) {
    return _core.writeFileInPlace(n, d, s, ar);
  }
  uint32_t readFile(const char* n, uint8_t* b, uint32_t bs) {
    return _core.readFile(n, b, bs);
  }
  uint32_t readFileRange(const char* n, uint32_t off, uint8_t* b, uint32_t l) {
    return _core.readFileRange(n, off, b, l);
  }
  bool getFileSize(const char* n, uint32_t& so) {
    return _core.getFileSize(n, so);
  }
  bool getFileInfo(const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
    return _core.getFileInfo(n, a, s, c);
  }
  bool exists(const char* n) {
    return _core.exists(n);
  }
  bool deleteFile(const char* n) {
    return _core.deleteFile(n);
  }
  void listFilesToSerial(Stream& out = Serial) {
    return _core.listFilesToSerial(out);
  }
  size_t fileCount() const {
    return _core.fileCount();
  }
  uint32_t nextDataAddr() const {
    return _core.nextDataAddr();
  }
  uint32_t capacity() const {
    return _core.capacity();
  }
  uint32_t dataRegionStart() const {
    return _core.dataRegionStart();
  }
  void close() {
    _core.close();
  }
  UnifiedSPIMemSimpleFS& raw() {
    return _core;
  }
private:
  UnifiedSPIMemSimpleFS _core;
};