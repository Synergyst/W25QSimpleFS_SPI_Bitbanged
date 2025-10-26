/*
  PSRAMMulti.h - Single-header multi-chip PSRAM aggregator + generic FS
  - Aggregates up to PSRAMMULTI_MAX_CHIPS identical PSRAM devices sharing MISO/MOSI/SCK
    with unique CS pins into one linear address space.
  - Reuses PSRAMBitbang's bit-banged SPI engine (transfer and bus timing), but manages CS per bank.
  - Includes a generic FS (PSRAMSimpleFS_Generic<Driver>) compatible with any Driver that
    implements:
       bool readData03(uint32_t addr, uint8_t* buf, size_t len);
       bool writeData02(uint32_t addr, const uint8_t* buf, size_t len, bool needsWriteEnable=false);
       uint32_t capacity() const;
*/

#ifndef PSRAMMULTI_H
#define PSRAMMULTI_H

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "PSRAMBitbang.h"

#ifndef PSRAMMULTI_MAX_CHIPS
#define PSRAMMULTI_MAX_CHIPS 8
#endif

class PSRAMAggregateDevice {
public:
  // Construct with bus pins and per-chip capacity (bytes).
  // Add chips later with addChip(csPin) before begin().
  PSRAMAggregateDevice(uint8_t pin_miso,
                       uint8_t pin_mosi,
                       uint8_t pin_sck,
                       uint32_t perChipCapacityBytes)
    : _bus(/*dummy CS not used*/ 0xFF, pin_miso, pin_mosi, pin_sck),
      _pinMISO(pin_miso),
      _pinMOSI(pin_mosi),
      _pinSCK(pin_sck),
      _chipCount(0),
      _perChipCapacity(perChipCapacityBytes),
      _halfCycleDelayUs(1),
      _useQuad(false),
      _io2(255), _io3(255) {}

  // Construct with an array of CS pins.
  PSRAMAggregateDevice(const uint8_t* csPins, size_t count,
                       uint8_t pin_miso,
                       uint8_t pin_mosi,
                       uint8_t pin_sck,
                       uint32_t perChipCapacityBytes)
    : PSRAMAggregateDevice(pin_miso, pin_mosi, pin_sck, perChipCapacityBytes) {
    for (size_t i = 0; i < count; ++i) addChip(csPins[i]);
  }

#if __cplusplus >= 201103L
  // Construct with initializer_list of CS pins.
  PSRAMAggregateDevice(std::initializer_list<uint8_t> csList,
                       uint8_t pin_miso,
                       uint8_t pin_mosi,
                       uint8_t pin_sck,
                       uint32_t perChipCapacityBytes)
    : PSRAMAggregateDevice(pin_miso, pin_mosi, pin_sck, perChipCapacityBytes) {
    for (uint8_t cs : csList) addChip(cs);
  }
#endif

  // Add a chip by CS pin; call before begin().
  bool addChip(uint8_t csPin) {
    if (_chipCount >= PSRAMMULTI_MAX_CHIPS) return false;
    _csPins[_chipCount++] = csPin;
    return true;
  }

  void begin() {
    // Configure shared bus pins (we do not call _bus.begin() to avoid touching a dummy CS)
    pinMode(_pinMOSI, OUTPUT);
    pinMode(_pinSCK, OUTPUT);
    pinMode(_pinMISO, INPUT);
    digitalWrite(_pinSCK, LOW);
    digitalWrite(_pinMOSI, LOW);

    // Configure per-chip CS lines
    for (uint8_t i = 0; i < _chipCount; ++i) {
      pinMode(_csPins[i], OUTPUT);
      digitalWrite(_csPins[i], HIGH);
    }

    // Timing and optional quad config pass-through to the engine (IO2/IO3 INPUT is fine here)
    _bus.setClockDelayUs(_halfCycleDelayUs);
    if (_io2 != 255 || _io3 != 255) {
      _bus.setExtraDataPins(_io2, _io3);
      _bus.setModeQuad(_useQuad);
    }
  }

  void setExtraDataPins(uint8_t io2, uint8_t io3) {
    _io2 = io2;
    _io3 = io3;
  }

  void setModeQuad(bool enable) {
    _useQuad = enable;
    _bus.setModeQuad(enable);
  }

  void setClockDelayUs(uint8_t halfCycleDelayUs) {
    _halfCycleDelayUs = halfCycleDelayUs;
    _bus.setClockDelayUs(halfCycleDelayUs);
  }

  // Total linear capacity across all banks
  uint32_t capacity() const {
    return _perChipCapacity * _chipCount;
  }

  uint32_t perChipCapacity() const {
    return _perChipCapacity;
  }
  uint8_t chipCount() const {
    return _chipCount;
  }

  // Read JEDEC for a specific bank (0..chipCount-1)
  void readJEDEC(uint8_t bank, uint8_t* out, size_t len) {
    if (bank >= _chipCount || !out || len == 0) return;
    uint8_t cmd = PSRAM_CMD_READ_JEDEC;
    csLow(bank);
    _bus.transfer(cmd);
    if (len) _bus.transfer(nullptr, out, len);
    csHigh(bank);
  }

  // Flat address space read (0x03)
  bool readData03(uint32_t addr, uint8_t* buf, size_t len) {
    if (len == 0) return true;
    if (!buf) return false;
    if (_chipCount == 0) return false;
    uint32_t total = capacity();
    if (addr >= total) return false;

    size_t remaining = len;
    uint32_t cur = addr;
    uint8_t* p = buf;

    while (remaining > 0) {
      uint8_t bank;
      uint32_t off;
      size_t chunk;
      if (!mapAddress(cur, remaining, bank, off, chunk)) return false;
      if (!bankRead(bank, off, p, chunk)) return false;
      cur += chunk;
      p += chunk;
      remaining -= chunk;
    }
    return true;
  }

  // Flat address space write (0x02)
  bool writeData02(uint32_t addr, const uint8_t* buf, size_t len, bool needsWriteEnable = false) {
    if (len == 0) return true;
    if (!buf) return false;
    if (_chipCount == 0) return false;
    uint32_t total = capacity();
    if (addr >= total) return false;

    size_t remaining = len;
    uint32_t cur = addr;
    const uint8_t* p = buf;

    while (remaining > 0) {
      uint8_t bank;
      uint32_t off;
      size_t chunk;
      if (!mapAddress(cur, remaining, bank, off, chunk)) return false;
      if (!bankWrite(bank, off, p, chunk, needsWriteEnable)) return false;
      cur += chunk;
      p += chunk;
      remaining -= chunk;
    }
    return true;
  }

  // Optional helper: raw MISO scan on a bank
  void rawMisoScan(uint8_t bank, uint8_t* out, size_t len) {
    if (bank >= _chipCount || !out || len == 0) return;
    csLow(bank);
    for (size_t i = 0; i < len; ++i) out[i] = _bus.transfer(0x00);
    csHigh(bank);
  }

  // Paste inside PSRAMAggregateDevice (public:)
  bool printCapacityReport(Stream& out = Serial) {
    out.println(F("PSRAM capacity report:"));
    out.print(F("  Banks: "));
    out.println(_chipCount);
    out.print(F("  Per-chip: "));
    out.print(_perChipCapacity);
    out.print(F(" bytes ("));
    out.print(_perChipCapacity / (1024UL * 1024UL));
    out.println(F(" MB)"));
    const uint32_t total = capacity();
    out.print(F("  Total: "));
    out.print(total);
    out.print(F(" bytes ("));
    out.print(total / (1024UL * 1024UL));
    out.println(F(" MB)"));

    uint8_t okCount = 0;
    for (uint8_t i = 0; i < _chipCount; ++i) {
      uint8_t id[6] = { 0 };
      readJEDEC(i, id, sizeof(id));
      bool allFF = true, all00 = true;
      for (size_t k = 0; k < sizeof(id); ++k) {
        if (id[k] != 0xFF) allFF = false;
        if (id[k] != 0x00) all00 = false;
      }
      const bool valid = !(allFF || all00);
      if (valid) ++okCount;

      out.print(F("  Bank "));
      out.print(i);
      out.print(F(" (CS="));
      out.print(_csPins[i]);
      out.print(F(") JEDEC: "));
      for (size_t k = 0; k < sizeof(id); ++k) {
        if (k) out.print(' ');
        if (id[k] < 16) out.print('0');
        out.print(id[k], HEX);
      }
      out.println(valid ? F("  [OK]") : F("  [NO RESP]"));
    }

    out.print(F("Probe result: "));
    out.print(okCount);
    out.print(F("/"));
    out.print(_chipCount);
    out.println(F(" banks responded"));
    return (okCount == _chipCount);
  }

private:
  inline void csLow(uint8_t bank) {
    digitalWrite(_csPins[bank], LOW);
  }
  inline void csHigh(uint8_t bank) {
    digitalWrite(_csPins[bank], HIGH);
  }

  bool mapAddress(uint32_t addr, size_t reqLen,
                  uint8_t& bankOut, uint32_t& offOut, size_t& chunkOut) const {
    if (_perChipCapacity == 0) return false;
    uint8_t bank = (uint8_t)(addr / _perChipCapacity);
    if (bank >= _chipCount) return false;
    uint32_t off = addr - ((uint32_t)bank * _perChipCapacity);
    uint32_t space = _perChipCapacity - off;
    size_t chunk = (reqLen < (size_t)space) ? reqLen : (size_t)space;
    bankOut = bank;
    offOut = off;
    chunkOut = chunk;
    return true;
  }

  bool bankRead(uint8_t bank, uint32_t off, uint8_t* buf, size_t len) {
    uint8_t cmd[4];
    cmd[0] = PSRAM_CMD_READ_03;
    cmd[1] = (uint8_t)((off >> 16) & 0xFF);
    cmd[2] = (uint8_t)((off >> 8) & 0xFF);
    cmd[3] = (uint8_t)(off & 0xFF);
    csLow(bank);
    _bus.transfer(cmd, nullptr, 4);
    _bus.transfer(nullptr, buf, len);
    csHigh(bank);
    return true;
  }

  bool bankWrite(uint8_t bank, uint32_t off, const uint8_t* buf, size_t len, bool needsWriteEnable) {
    if (!buf || len == 0) return true;
    if (needsWriteEnable) {
      uint8_t we = PSRAM_CMD_WRITE_ENABLE;
      csLow(bank);
      _bus.transfer(we);
      csHigh(bank);
    }
    uint8_t cmd[4];
    cmd[0] = PSRAM_CMD_WRITE_02;
    cmd[1] = (uint8_t)((off >> 16) & 0xFF);
    cmd[2] = (uint8_t)((off >> 8) & 0xFF);
    cmd[3] = (uint8_t)(off & 0xFF);
    csLow(bank);
    _bus.transfer(cmd, nullptr, 4);
    _bus.transfer(buf, nullptr, len);
    csHigh(bank);
    return true;
  }

  // Members: order matches initialization to avoid -Wreorder
  PSRAMBitbang _bus;
  uint8_t _pinMISO, _pinMOSI, _pinSCK;
  uint8_t _csPins[PSRAMMULTI_MAX_CHIPS];
  uint8_t _chipCount;
  uint32_t _perChipCapacity;
  uint8_t _halfCycleDelayUs;
  bool _useQuad;
  uint8_t _io2, _io3;
};

// -------------------------
// Generic, header-only FS
// -------------------------
template<typename Driver>
class PSRAMSimpleFS_Generic {
public:
  static const uint32_t DIR_START = 0x000000UL;
  static const uint32_t DIR_SIZE = 64UL * 1024UL;
  static const uint32_t ENTRY_SIZE = 32;
  static const uint32_t DATA_START = DIR_START + DIR_SIZE;
  static const uint32_t SECTOR_SIZE = 4096;
  static const uint32_t PAGE_SIZE = 256;
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

  PSRAMSimpleFS_Generic(Driver& dev, uint32_t capacityBytes)
    : _dev(dev), _capacity(capacityBytes) {
    _fileCount = 0;
    _dirWriteOffset = 0;
    _nextSeq = 1;
    _dataHead = DATA_START;
  }

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
      _dev.readData03(addr, buf, ENTRY_SIZE);
      if (isAllFF(buf, ENTRY_SIZE)) {
        _dirWriteOffset = i * ENTRY_SIZE;
        break;
      }
      sawAny = true;
      if (buf[0] != 0x57 || buf[1] != 0x46) continue;  // 'W' 'F'
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
    const uint32_t PAGE_CHUNK = 256;
    uint8_t tmp[PAGE_CHUNK];
    memset(tmp, 0xFF, PAGE_CHUNK);
    for (uint32_t i = 0; i < DIR_SIZE; i += PAGE_CHUNK) {
      uint32_t chunk = (i + PAGE_CHUNK <= DIR_SIZE) ? PAGE_CHUNK : (DIR_SIZE - i);
      _dev.writeData02(DIR_START + i, tmp, chunk);
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
      _dev.writeData02(pos, tmp, n);
      pos += n;
    }
    _fileCount = 0;
    _dirWriteOffset = 0;
    _dataHead = DATA_START;
    computeCapacities(_dataHead);
    return true;
  }

  // Primary API with our enum
  bool writeFile(const char* name, const uint8_t* data, uint32_t size, WriteMode mode = WriteMode::ReplaceIfExists) {
    if (!validName(name) || size > 0xFFFFFFUL) return false;
    if (_dirWriteOffset + ENTRY_SIZE > DIR_SIZE) return false;
    int idxExisting = findIndexByName(name);
    bool exists = (idxExisting >= 0 && !_files[idxExisting].deleted);
    if (exists && mode == WriteMode::FailIfExists) return false;
    uint32_t start = _dataHead;
    if (start < DATA_START) start = DATA_START;
    if (start + size > _capacity) return false;
    if (size > 0) _dev.writeData02(start, data, size);
    if (!appendDirEntry(0x00, name, start, size)) return false;
    upsertFileIndex(name, start, size, false);
    _dataHead = start + size;
    computeCapacities(_dataHead);
    return true;
  }

  // Compatibility overload: accept int mode (0=ReplaceIfExists, 1=FailIfExists)
  bool writeFile(const char* name, const uint8_t* data, uint32_t size, int modeInt) {
    WriteMode m = (modeInt == (int)WriteMode::FailIfExists) ? WriteMode::FailIfExists : WriteMode::ReplaceIfExists;
    return writeFile(name, data, size, m);
  }

  // Compatibility overload: accept any enum-like type (e.g., PSRAMSimpleFS::WriteMode)
  template<typename ModeT>
  bool writeFile(const char* name, const uint8_t* data, uint32_t size, ModeT modeOther) {
    return writeFile(name, data, size, (int)modeOther);
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
      _dev.writeData02(p, tmp, n);
      p += n;
    }
    if (initialSize > 0) _dev.writeData02(start, initialData, initialSize);
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
      if (size > 0) _dev.writeData02(fi.addr, data, size);
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
    _dev.readData03(_files[idx].addr, buf, n);
    return n;
  }

  uint32_t readFileRange(const char* name, uint32_t offset, uint8_t* buf, uint32_t len) {
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return 0;
    if (offset >= _files[idx].size) return 0;
    uint32_t maxLen = _files[idx].size - offset;
    if (len > maxLen) len = maxLen;
    if (len == 0) return 0;
    _dev.readData03(_files[idx].addr + offset, buf, len);
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
    int idx = findIndexByName(name);
    if (idx < 0 || _files[idx].deleted) return false;
    if (!appendDirEntry(0x01, name, 0, 0)) return false;
    _files[idx].deleted = true;
    _files[idx].addr = 0;
    _files[idx].size = 0;
    computeCapacities(_dataHead);
    return true;
  }

  void listFilesToSerial() {
    Serial.println("Files (PSRAM Multi):");
    for (size_t i = 0; i < _fileCount; ++i) {
      if (_files[i].deleted) continue;
      Serial.printf("- %s  \tsize=%u  \taddr=0x", _files[i].name, (unsigned)_files[i].size);
      Serial.print(_files[i].addr, HEX);
      uint32_t cap = (_files[i].capEnd > _files[i].addr) ? (_files[i].capEnd - _files[i].addr) : 0;
      Serial.printf("  \tcap=%u  \tslotSafe=%s\n", (unsigned)cap, _files[i].slotSafe ? "Y" : "N");
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
    _dev.writeData02(DIR_START + _dirWriteOffset, rec, ENTRY_SIZE);
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

// Convenience alias
using PSRAMSimpleFS_Multi = PSRAMSimpleFS_Generic<PSRAMAggregateDevice>;

#endif  // PSRAMMULTI_H