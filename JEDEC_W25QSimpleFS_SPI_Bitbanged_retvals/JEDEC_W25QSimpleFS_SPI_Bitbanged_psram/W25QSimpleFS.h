#pragma once
#include "W25QBitbang.h"

// Simple FS for W25Q128 with two write styles:
//  - Append-only (default): writeFile() appends new versions at the tail.
//  - Fixed-slot (in-place): createFileSlot() reserves sector-aligned space,
//    writeFileInPlace() erases+programs within that slot at the same base address.
//
// Directory: 64 KiB log at 0x000000, 32-byte records (last-wins by name).
// Data: from 0x010000 to end of chip.
class W25QSimpleFS {
public:
  static const uint32_t DIR_START = 0x000000UL;
  static const uint32_t DIR_SIZE = 64UL * 1024UL;  // 64 KiB
  static const uint32_t ENTRY_SIZE = 32;
  static const uint32_t DATA_START = DIR_START + DIR_SIZE;
  static const uint32_t SECTOR_SIZE = 4096;  // erase/program granularity for safe in-place
  static const uint32_t PAGE_SIZE = 256;     // program page size
  static const uint8_t MAGIC0 = 0x57;        // 'W'
  static const uint8_t MAGIC1 = 0x46;        // 'F'
  static const uint8_t FLAG_DELETED = 0x01;
  static const size_t MAX_FILES = 64;  // in-memory index limit
  static const size_t MAX_NAME = 32;   // stored name length (bytes in record)

  enum class WriteMode : uint8_t {
    ReplaceIfExists,  // append new version at tail; last-wins
    FailIfExists      // fail if name already exists
  };

  struct FileInfo {
    char name[MAX_NAME + 1];  // null-terminated
    uint32_t addr;            // base address in DATA
    uint32_t size;            // logical size
    uint32_t seq;             // last-seen sequence/id
    bool deleted;
    // Computed on mount:
    uint32_t capEnd;  // exclusive end of reserved region (gap to next file or data end)
    bool slotSafe;    // true if addr and capEnd are both sector-aligned
  };

  explicit W25QSimpleFS(W25QBitbang& flash)
    : _flash(flash) {}

  // Optional: pack vs align new allocations to page boundary (append-only path only).
  void setAlignToPageBoundary(bool on) {
    _alignToPage = on;
  }

  bool mount(bool autoFormatIfEmpty = true) {
    _capacity = 0;
    uint8_t m, t, c;
    _capacity = _flash.readJEDEC(m, t, c);
    if (_capacity == 0 || _capacity <= DATA_START) return false;

    _fileCount = 0;
    _dirWriteOffset = 0;
    _nextSeq = 1;
    _dataHead = DATA_START;

    uint32_t maxEnd = DATA_START;
    uint32_t maxSeq = 0;
    bool sawAny = false;

    const uint32_t dirEntries = DIR_SIZE / ENTRY_SIZE;
    uint8_t buf[ENTRY_SIZE];

    for (uint32_t i = 0; i < dirEntries; ++i) {
      uint32_t addr = DIR_START + i * ENTRY_SIZE;
      _flash.readData(addr, buf, ENTRY_SIZE);

      if (isAllFF(buf, ENTRY_SIZE)) {
        _dirWriteOffset = i * ENTRY_SIZE;
        break;
      }
      sawAny = true;

      if (buf[0] != MAGIC0 || buf[1] != MAGIC1) continue;

      uint8_t flags = buf[2];
      uint8_t nameLen = buf[3];
      if (nameLen == 0 || nameLen > MAX_NAME) continue;

      char nameBuf[MAX_NAME + 1] = { 0 };
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
          continue;  // out of slots; we still compute maxEnd below
        }
      }

      bool deleted = (flags & FLAG_DELETED) != 0;

      // Last-wins: just overwrite the entry we have
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

      if (i == dirEntries - 1) _dirWriteOffset = DIR_SIZE;  // dir full
    }

    if (!sawAny) {
      _dirWriteOffset = 0;
      if (autoFormatIfEmpty) {
        if (!eraseRange(DIR_START, DIR_SIZE)) return false;
      }
    }

    _nextSeq = maxSeq + 1;
    if (_nextSeq == 0) _nextSeq = 1;

    // Next data head (append-only path)
    _dataHead = _alignToPage ? alignUp(maxEnd, PAGE_SIZE) : maxEnd;
    if (_dataHead < DATA_START) _dataHead = DATA_START;

    // Compute per-file capacity and slotSafe by sorting live files by address
    computeCapacities(maxEnd);
    return true;
  }

  bool format() {
    _fileCount = 0;
    _dirWriteOffset = 0;
    _nextSeq = 1;
    _dataHead = DATA_START;
    bool ok = eraseRange(DIR_START, DIR_SIZE);
    return ok;
  }

  bool wipeChip() {
    bool ok = _flash.chipErase();
    if (ok) {
      _fileCount = 0;
      _dirWriteOffset = 0;
      _nextSeq = 1;
      _dataHead = DATA_START;
    }
    return ok;
  }

  // Append-only create/replace. Always allocates new space at tail.
  bool writeFile(const char* name, const uint8_t* data, uint32_t size,
                 WriteMode mode = WriteMode::ReplaceIfExists) {
    if (!validName(name) || size > 0xFFFFFFUL) return false;
    if (_dirWriteOffset + ENTRY_SIZE > DIR_SIZE) return false;

    int idxExisting = findIndexByName(name);
    bool exists = (idxExisting >= 0 && !_files[idxExisting].deleted);
    if (exists && mode == WriteMode::FailIfExists) return false;

    uint32_t start = _alignToPage ? alignUp(_dataHead, PAGE_SIZE) : _dataHead;
    if (start < DATA_START) start = DATA_START;
    if (start + size > _capacity) return false;

    if (!eraseRange(start, size)) return false;
    if (size > 0 && !_flash.pageProgram(start, data, size)) return false;

    if (!appendDirEntry(0x00, name, start, size)) return false;
    upsertFileIndex(name, start, size, false);

    _dataHead = start + size;
    // Recompute capacities since we added a file
    computeCapacities(_dataHead);
    return true;
  }

  // Reserve a sector-aligned slot for a file. Future in-place rewrites will keep the same base address.
  // reserveBytes will be rounded up to SECTOR_SIZE.
  bool createFileSlot(const char* name, uint32_t reserveBytes, const uint8_t* initialData = nullptr, uint32_t initialSize = 0) {
    if (!validName(name)) return false;
    if (_dirWriteOffset + ENTRY_SIZE > DIR_SIZE) return false;
    if (initialSize > reserveBytes) return false;

    // Fail if the name already exists (avoid confusion)
    if (exists(name)) return false;

    uint32_t cap = alignUp((reserveBytes < 1u ? 1u : reserveBytes), SECTOR_SIZE);
    uint32_t start = alignUp(_dataHead, SECTOR_SIZE);
    if (start < DATA_START) start = DATA_START;
    if (start + cap > _capacity) return false;

    if (!eraseRange(start, cap)) return false;
    if (initialSize > 0) {
      if (!_flash.pageProgram(start, initialData, initialSize)) return false;
    }

    if (!appendDirEntry(0x00, name, start, initialSize)) return false;
    upsertFileIndex(name, start, initialSize, false);

    _dataHead = start + cap;
    // Mark slot capacity
    computeCapacities(_dataHead);
    return true;
  }

  // In-place rewrite: keep the same base address, erase + program within the reserved slot.
  // If allowReallocate is true and slot is too small or unsafe, falls back to append-only allocation (address will change).
  bool writeFileInPlace(const char* name, const uint8_t* data, uint32_t size, bool allowReallocate = false) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;

    FileInfo& fi = _files[idx];
    uint32_t cap = (fi.capEnd > fi.addr) ? (fi.capEnd - fi.addr) : 0;

    if (fi.slotSafe && cap >= size) {
      // Erase only the sectors we will rewrite (rounded up)
      uint32_t eraseLen = alignUp(size, SECTOR_SIZE);
      if (!eraseRange(fi.addr, eraseLen)) return false;
      if (size > 0 && !_flash.pageProgram(fi.addr, data, size)) return false;

      // Append a new dir record pointing to the SAME base address with the new size
      if (!appendDirEntry(0x00, name, fi.addr, size)) return false;

      // Update in-memory info
      fi.size = size;
      return true;
    }

    if (!allowReallocate) {
      // Not safe to in-place (slot not sector-aligned or too small)
      return false;
    }

    // Fallback: append-only new version (address will change)
    return writeFile(name, data, size, WriteMode::ReplaceIfExists);
  }

  // Unique create: fail if exists
  bool createFileUnique(const char* name, const uint8_t* data, uint32_t size) {
    return writeFile(name, data, size, WriteMode::FailIfExists);
  }

  bool deleteFile(const char* name) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;
    if (_dirWriteOffset + ENTRY_SIZE > DIR_SIZE) return false;

    if (!appendDirEntry(FLAG_DELETED, name, 0, 0)) return false;

    _files[idx].deleted = true;
    _files[idx].addr = 0;
    _files[idx].size = 0;

    // Recompute capacities (the gap may expand)
    computeCapacities(_dataHead);
    return true;
  }

  // Readers
  uint32_t readFile(const char* name, uint8_t* buf, uint32_t bufSize) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return 0;
    uint32_t n = _files[idx].size;
    if (bufSize < n) n = bufSize;
    if (n == 0) return 0;
    _flash.readData(_files[idx].addr, buf, n);
    return n;
  }

  uint32_t readFileRange(const char* name, uint32_t offset, uint8_t* buf, uint32_t len) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return 0;
    if (offset >= _files[idx].size) return 0;
    uint32_t maxLen = _files[idx].size - offset;
    if (len > maxLen) len = maxLen;
    if (len == 0) return 0;
    _flash.readData(_files[idx].addr + offset, buf, len);
    return len;
  }

  bool getFileSize(const char* name, uint32_t& sizeOut) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;
    sizeOut = _files[idx].size;
    return true;
  }

  bool getFileInfo(const char* name, uint32_t& addrOut, uint32_t& sizeOut, uint32_t& capacityOut) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;
    addrOut = _files[idx].addr;
    sizeOut = _files[idx].size;
    capacityOut = (_files[idx].capEnd > _files[idx].addr) ? (_files[idx].capEnd - _files[idx].addr) : 0;
    return true;
  }

  bool exists(const char* name) {
    int idx = findIndexByName(name);
    return (idx >= 0 && !_files[idx].deleted);
  }

  void listFilesToSerial() {
    Serial.println("Files:");
    for (size_t i = 0; i < _fileCount; ++i) {
      if (_files[i].deleted) continue;
      Serial.printf("- %s  \tsize=%d  \taddr=0x", _files[i].name, _files[i].size);
      Serial.print(_files[i].addr, HEX);
      uint32_t cap = (_files[i].capEnd > _files[i].addr) ? (_files[i].capEnd - _files[i].addr) : 0;
      Serial.printf("  \tcap=%d  \tslotSafe=%s\n", cap, _files[i].slotSafe ? "Y" : "N");
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
  W25QBitbang& _flash;
  FileInfo _files[MAX_FILES];
  size_t _fileCount = 0;
  uint32_t _capacity = 0;
  uint32_t _dirWriteOffset = 0;  // offset within DIR region
  uint32_t _dataHead = DATA_START;
  uint32_t _nextSeq = 1;
  bool _alignToPage = false;  // append-only packing preference

  // Utils
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
    return (v + (a - 1)) & ~(a - 1);
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

  int findIndexByName(const char* name) const {
    for (size_t i = 0; i < _fileCount; ++i) {
      if (strncmp(_files[i].name, name, MAX_NAME) == 0) return (int)i;
    }
    return -1;
  }

  void upsertFileIndex(const char* name, uint32_t addr, uint32_t size, bool deleted) {
    int idx = findIndexByName(name);
    if (idx < 0) {
      if (_fileCount >= MAX_FILES) return;
      idx = _fileCount++;
      copyName(_files[idx].name, name);
    }
    _files[idx].addr = addr;
    _files[idx].size = size;
    _files[idx].deleted = deleted;
    _files[idx].seq = _nextSeq;  // approx
    // capEnd/slotSafe recomputed by computeCapacities()
  }

  bool appendDirEntry(uint8_t flags, const char* name, uint32_t addr, uint32_t size) {
    if (!validName(name)) return false;
    if (_dirWriteOffset + ENTRY_SIZE > DIR_SIZE) return false;

    // Ensure target 32 bytes are empty
    uint8_t existing[ENTRY_SIZE];
    _flash.readData(DIR_START + _dirWriteOffset, existing, ENTRY_SIZE);
    if (!isAllFF(existing, ENTRY_SIZE)) return false;

    uint8_t rec[ENTRY_SIZE];
    memset(rec, 0xFF, sizeof(rec));
    rec[0] = MAGIC0;
    rec[1] = MAGIC1;
    rec[2] = flags;

    uint8_t nameLen = (uint8_t)min((size_t)MAX_NAME, strlen(name));
    rec[3] = nameLen;
    for (uint8_t i = 0; i < nameLen; ++i) rec[4 + i] = (uint8_t)name[i];

    wr32(&rec[20], addr);
    wr32(&rec[24], size);
    wr32(&rec[28], _nextSeq++);

    bool ok = _flash.pageProgram(DIR_START + _dirWriteOffset, rec, ENTRY_SIZE);
    if (!ok) return false;
    _dirWriteOffset += ENTRY_SIZE;
    return true;
  }

  bool eraseRange(uint32_t start, uint32_t len) {
    if (len == 0) return true;
    uint32_t a = start & ~(SECTOR_SIZE - 1);
    uint32_t end = alignUp(start + len, SECTOR_SIZE);
    for (uint32_t s = a; s < end; s += SECTOR_SIZE) {
      if (s >= _capacity) return false;
      if (!_flash.sectorErase4K(s)) return false;
      yield();
    }
    return true;
  }

  void computeCapacities(uint32_t maxEnd) {
    // Build a list of live files with addresses
    int idxs[MAX_FILES];
    size_t n = 0;
    for (size_t i = 0; i < _fileCount; ++i) {
      if (_files[i].deleted) continue;
      idxs[n++] = (int)i;
    }

    // Simple insertion sort by addr (n is small)
    for (size_t i = 1; i < n; ++i) {
      int key = idxs[i];
      size_t j = i;
      while (j > 0 && _files[idxs[j - 1]].addr > _files[key].addr) {
        idxs[j] = idxs[j - 1];
        --j;
      }
      idxs[j] = key;
    }

    // Compute capEnd and slotSafe
    for (size_t i = 0; i < n; ++i) {
      FileInfo& fi = _files[idxs[i]];
      uint32_t nextStart = (i + 1 < n) ? _files[idxs[i + 1]].addr : alignUp(maxEnd, SECTOR_SIZE);
      // Capacity ends at the next file start (or end-of-data head aligned up to sector boundary)
      fi.capEnd = nextStart;
      fi.slotSafe = ((fi.addr % SECTOR_SIZE) == 0) && ((fi.capEnd % SECTOR_SIZE) == 0) && (fi.capEnd > fi.addr);
    }
  }
};