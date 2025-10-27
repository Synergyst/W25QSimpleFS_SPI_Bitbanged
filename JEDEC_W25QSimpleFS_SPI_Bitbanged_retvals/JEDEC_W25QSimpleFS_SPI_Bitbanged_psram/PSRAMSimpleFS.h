/*
  PSRAMSimpleFS.h - simple directory+data FS stored in PSRAM (header-only)
  Updated fixes:
   - Removed unused local variable warning in format()
   - API same as before for compatibility with W25QSimpleFS usage
*/
#ifndef PSRAMSIMPLEFS_H
#define PSRAMSIMPLEFS_H
#include "PSRAMBitbang.h"
#include <stdint.h>
#include <string.h>

class PSRAMSimpleFS {
public:
  static const uint32_t DIR_START = 0x000000UL;
  static const uint32_t DIR_SIZE = 64UL * 1024UL;
  static const uint32_t ENTRY_SIZE = 32;
  static const uint32_t DATA_START = DIR_START + DIR_SIZE;
  static const uint32_t SECTOR_SIZE = 4096;
  static const uint32_t PAGE_SIZE = 256;
  static const size_t MAX_NAME = 32;

  enum class WriteMode : uint8_t {
    ReplaceIfExists,
    FailIfExists
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

  PSRAMSimpleFS(PSRAMBitbang& psram, uint32_t capacityBytes)
    : _psram(psram), _capacity(capacityBytes) {
    _fileCount = 0;
    _dirWriteOffset = 0;
    _nextSeq = 1;
    _dataHead = DATA_START;
  }

  // Mount. If autoFormatIfEmpty true, initialize dir region if appears empty.
  bool mount(bool autoFormatIfEmpty = true) {
    if (_capacity == 0 || _capacity <= DATA_START) return false;
    _fileCount = 0;
    _dirWriteOffset = 0;
    _nextSeq = 1;
    _dataHead = DATA_START;

    uint32_t maxEnd = DATA_START;
    uint32_t maxSeq = 0;
    bool sawAny = false;

    uint32_t entries = DIR_SIZE / ENTRY_SIZE;
    uint8_t buf[ENTRY_SIZE];

    for (uint32_t i = 0; i < entries; ++i) {
      uint32_t addr = DIR_START + i * ENTRY_SIZE;
      _psram.readData03(addr, buf, ENTRY_SIZE);

      if (isAllFF(buf, ENTRY_SIZE)) {
        _dirWriteOffset = i * ENTRY_SIZE;
        break;
      }
      sawAny = true;
      if (buf[0] != 0x57 || buf[1] != 0x46) continue;  // MAGIC 'W' 'F'

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
      if (autoFormatIfEmpty) {
        format();
      }
    }

    _nextSeq = maxSeq + 1;
    if (_nextSeq == 0) _nextSeq = 1;

    _dataHead = maxEnd;
    computeCapacities(_dataHead);
    return true;
  }

  bool format() {
    // Fill directory region with 0xFF in page-chunk writes
    const uint32_t PAGE_CHUNK = 256;
    uint8_t tmp[PAGE_CHUNK];
    memset(tmp, 0xFF, PAGE_CHUNK);
    for (uint32_t i = 0; i < DIR_SIZE; i += PAGE_CHUNK) {
      uint32_t chunk = (i + PAGE_CHUNK <= DIR_SIZE) ? PAGE_CHUNK : (DIR_SIZE - i);
      _psram.writeData02(DIR_START + i, tmp, chunk);
    }
    _fileCount = 0;
    _dirWriteOffset = 0;
    _nextSeq = 1;
    _dataHead = DATA_START;
    computeCapacities(_dataHead);
    return true;
  }

  bool wipeChip() {
    if (_capacity == 0) return false;
    const uint32_t CHUNK = 256;
    uint8_t tmp[CHUNK];
    memset(tmp, 0xFF, CHUNK);
    uint32_t pos = 0;
    while (pos < _capacity) {
      uint32_t n = (pos + CHUNK <= _capacity) ? CHUNK : (_capacity - pos);
      _psram.writeData02(pos, tmp, n);
      pos += n;
    }
    _fileCount = 0;
    _dirWriteOffset = 0;
    _dataHead = DATA_START;
    computeCapacities(_dataHead);
    return true;
  }

  // Append-only write (allocates new space at tail)
  bool writeFile(const char* name, const uint8_t* data, uint32_t size, WriteMode mode = WriteMode::ReplaceIfExists) {
    if (!validName(name) || size > 0xFFFFFFUL) return false;
    if (_dirWriteOffset + ENTRY_SIZE > DIR_SIZE) return false;

    int idxExisting = findIndexByName(name);
    bool exists = (idxExisting >= 0 && !_files[idxExisting].deleted);
    if (exists && mode == WriteMode::FailIfExists) return false;

    uint32_t start = _dataHead;
    if (start < DATA_START) start = DATA_START;
    if (start + size > _capacity) return false;

    if (size > 0) _psram.writeData02(start, data, size);
    if (!appendDirEntry(0x00, name, start, size)) return false;

    upsertFileIndex(name, start, size, false);
    _dataHead = start + size;
    computeCapacities(_dataHead);
    return true;
  }

  bool createFileSlot(const char* name, uint32_t reserveBytes, const uint8_t* initialData = nullptr, uint32_t initialSize = 0) {
    if (!validName(name)) return false;
    if (_dirWriteOffset + ENTRY_SIZE > DIR_SIZE) return false;
    if (initialSize > reserveBytes) return false;
    if (exists(name)) return false;

    uint32_t cap = alignUp((reserveBytes < 1u ? 1u : reserveBytes), SECTOR_SIZE);
    uint32_t start = alignUp(_dataHead, SECTOR_SIZE);
    if (start < DATA_START) start = DATA_START;
    if (start + cap > _capacity) return false;

    uint32_t p = start;
    const uint32_t PAGE_CHUNK = 256;
    uint8_t tmp[PAGE_CHUNK];
    memset(tmp, 0xFF, PAGE_CHUNK);
    while (p < start + cap) {
      uint32_t n = min<uint32_t>(PAGE_CHUNK, start + cap - p);
      _psram.writeData02(p, tmp, n);
      p += n;
    }

    if (initialSize > 0) _psram.writeData02(start, initialData, initialSize);
    if (!appendDirEntry(0x00, name, start, initialSize)) return false;

    upsertFileIndex(name, start, initialSize, false);
    _dataHead = start + cap;
    computeCapacities(_dataHead);
    return true;
  }

  bool writeFileInPlace(const char* name, const uint8_t* data, uint32_t size, bool allowReallocate = false) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;

    FileInfo& fi = _files[idx];
    uint32_t cap = (fi.capEnd > fi.addr) ? (fi.capEnd - fi.addr) : 0;

    if (fi.slotSafe && cap >= size) {
      if (size > 0) _psram.writeData02(fi.addr, data, size);
      if (!appendDirEntry(0x00, name, fi.addr, size)) return false;
      fi.size = size;
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
    _psram.readData03(_files[idx].addr, buf, n);
    return n;
  }

  uint32_t readFileRange(const char* name, uint32_t offset, uint8_t* buf, uint32_t len) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return 0;
    if (offset >= _files[idx].size) return 0;
    uint32_t maxLen = _files[idx].size - offset;
    if (len > maxLen) len = maxLen;
    if (len == 0) return 0;
    _psram.readData03(_files[idx].addr + offset, buf, len);
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

  // New: delete a file by appending a tombstone directory entry
  bool deleteFile(const char* name) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;
    // Append deletion record (flags bit0 = deleted)
    if (!appendDirEntry(0x01, name, 0, 0)) return false;
    // Update in-memory index
    _files[idx].deleted = true;
    _files[idx].addr = 0;
    _files[idx].size = 0;
    // Recompute capacities for remaining files
    computeCapacities(_dataHead);
    return true;
  }

  void listFilesToSerial() {
    Serial.println("Files (PSRAM):");
    for (size_t i = 0; i < _fileCount; ++i) {
      if (_files[i].deleted) continue;
      Serial.printf("- %s  \tsize=%" PRIu32 "  \taddr=0x", _files[i].name, _files[i].size);
      Serial.print(_files[i].addr, HEX);
      uint32_t cap = (_files[i].capEnd > _files[i].addr) ? (_files[i].capEnd - _files[i].addr) : 0;
      Serial.printf("  \tcap=%" PRIu32 "  \tslotSafe=%s\n", cap, _files[i].slotSafe ? "Y" : "N");
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
  PSRAMBitbang& _psram;
  uint32_t _capacity;

  static const size_t MAX_FILES = 2048;
  FileInfo _files[MAX_FILES];
  size_t _fileCount;
  uint32_t _dirWriteOffset;
  uint32_t _dataHead;
  uint32_t _nextSeq;

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
    _files[idx].seq = _nextSeq++;
  }
  bool appendDirEntry(uint8_t flags, const char* name, uint32_t addr, uint32_t size) {
    if (!validName(name)) return false;
    if (_dirWriteOffset + ENTRY_SIZE > DIR_SIZE) return false;
    uint8_t rec[ENTRY_SIZE];
    memset(rec, 0xFF, ENTRY_SIZE);
    rec[0] = 0x57;  // 'W'
    rec[1] = 0x46;  // 'F'
    rec[2] = flags;
    uint8_t nameLen = (uint8_t)min((size_t)MAX_NAME, strlen(name));
    rec[3] = nameLen;
    for (uint8_t i = 0; i < nameLen; ++i) rec[4 + i] = (uint8_t)name[i];
    wr32(&rec[20], addr);
    wr32(&rec[24], size);
    wr32(&rec[28], _nextSeq++);
    _psram.writeData02(DIR_START + _dirWriteOffset, rec, ENTRY_SIZE);
    _dirWriteOffset += ENTRY_SIZE;
    return true;
  }
  void computeCapacities(uint32_t maxEnd) {
    int idxs[MAX_FILES];
    size_t n = 0;
    for (size_t i = 0; i < _fileCount; ++i) {
      if (_files[i].deleted) continue;
      idxs[n++] = (int)i;
    }
    for (size_t i = 1; i < n; ++i) {
      int key = idxs[i];
      size_t j = i;
      while (j > 0 && _files[idxs[j - 1]].addr > _files[key].addr) {
        idxs[j] = idxs[j - 1];
        --j;
      }
      idxs[j] = key;
    }
    for (size_t i = 0; i < n; ++i) {
      FileInfo& fi = _files[idxs[i]];
      uint32_t nextStart = (i + 1 < n) ? _files[idxs[i + 1]].addr : alignUp(maxEnd, SECTOR_SIZE);
      fi.capEnd = nextStart;
      fi.slotSafe = ((fi.addr % SECTOR_SIZE) == 0) && ((fi.capEnd % SECTOR_SIZE) == 0) && (fi.capEnd > fi.addr);
    }
  }
};

#endif  // PSRAMSIMPLEFS_H