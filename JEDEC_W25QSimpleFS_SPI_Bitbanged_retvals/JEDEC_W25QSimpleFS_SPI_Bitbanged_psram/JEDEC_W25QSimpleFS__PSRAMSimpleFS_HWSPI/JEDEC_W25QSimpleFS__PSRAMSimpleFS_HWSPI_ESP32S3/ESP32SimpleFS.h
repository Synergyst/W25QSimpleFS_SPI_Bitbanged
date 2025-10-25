// ========== Internal flash -> W25QSimpleFS driver ==========
class ESP32W25QPartitionDriver {
public:
  ESP32W25QPartitionDriver(const char* label = nullptr)
    : _label(label), _part(nullptr) {}

  // Call in setup(); if label is null, uses the first SPIFFS partition.
  bool begin(const char* preferredLabel = nullptr) {
    if (preferredLabel) _label = preferredLabel;
    // Try explicit label first (if provided)
    if (_label && *_label) {
      _part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                       ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
                                       _label);
    }
    // Fallback to any SPIFFS data partition
    if (!_part) {
      _part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                       ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
                                       nullptr);
    }
    return _part != nullptr;
  }

  // SimpleFS capacity is taken from return value of readJEDEC
  uint32_t readJEDEC(uint8_t& m, uint8_t& t, uint8_t& c) {
    if (!_part) {
      m = t = c = 0;
      return 0;
    }
    // Fake a Winbond-like ID; capacity is returned separately as bytes
    m = 0xEF;            // Winbond
    t = 0x40;            // family/type
    c = 0x18;            // density (W25Q128-like) — informational only
    return _part->size;  // actual usable capacity in bytes
  }

  void readData(uint32_t addr, uint8_t* buf, uint32_t len) {
    if (!_part || !buf || (addr + len) > _part->size) return;
    esp_partition_read(_part, addr, buf, len);
  }

  // Emulate page-program with safe chunking (<=256 bytes, no cross-page wrap)
  bool pageProgram(uint32_t addr, const uint8_t* data, uint32_t len) {
    if (!_part || !data || (addr + len) > _part->size) return false;
    const uint32_t PAGE = 256;
    uint32_t off = 0;
    while (off < len) {
      uint32_t a = addr + off;
      uint32_t room = PAGE - (a % PAGE);
      uint32_t n = (len - off < room) ? (len - off) : room;
      if (esp_partition_write(_part, a, data + off, n) != ESP_OK) return false;
      off += n;
      yield();
    }
    return true;
  }

  bool sectorErase4K(uint32_t addr) {
    if (!_part) return false;
    const uint32_t SEC = 4096;
    uint32_t base = (addr / SEC) * SEC;
    if (base >= _part->size) return false;
    return esp_partition_erase_range(_part, base, SEC) == ESP_OK;
  }

  bool chipErase() {
    if (!_part) return false;
    // Partition size is a multiple of 4K; one call suffices
    return esp_partition_erase_range(_part, 0, _part->size) == ESP_OK;
  }

private:
  const char* _label;
  const esp_partition_t* _part;
};

// ========== Internal PSRAM -> PSRAMSimpleFS driver ==========
class ESP32PSRAMLinearDriver {
public:
  ESP32PSRAMLinearDriver()
    : _buf(nullptr), _cap(0) {}

  // Allocate a contiguous PSRAM buffer. Fill with 0xFF to mimic erased flash.
  bool begin(uint32_t requestedBytes) {
    size_t total = ESP.getPsramSize();
    if (total == 0) return false;
    uint32_t want = requestedBytes ? requestedBytes : (uint32_t)total;
    if (want > total) want = (uint32_t)total;
    _buf = (uint8_t*)ps_malloc(want);
    if (!_buf) return false;
    _cap = want;
    memset(_buf, 0xFF, _cap);
    return true;
  }

  void readData03(uint32_t addr, uint8_t* buf, uint32_t len) {
    if (!_buf || !buf || (addr + len) > _cap) return;
    memcpy(buf, _buf + addr, len);
  }

  void writeData02(uint32_t addr, const uint8_t* buf, uint32_t len) {
    if (!_buf || !buf || (addr + len) > _cap) return;
    memcpy(_buf + addr, buf, len);
  }

  uint32_t capacity() const {
    return _cap;
  }

private:
  uint8_t* _buf;
  uint32_t _cap;
};