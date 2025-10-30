// Helper to supply Exec with current FS function pointers
static void updateExecFsTable() {
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
  allOk &= ensureBlobIfMissing(FILE_ADD2, blob_add2, blob_add2_len);
  allOk &= ensureBlobIfMissing(FILE_PWMC, blob_pwmc, blob_pwmc_len);
  allOk &= ensureBlobIfMissing(FILE_PWMC2350, blob_pwmc2350, blob_pwmc2350_len);
  allOk &= ensureBlobIfMissing(FILE_RETMIN, blob_retmin, blob_retmin_len);
  allOk &= ensureBlobIfMissing(FILE_BLINKSCRIPT, blob_blinkscript, blob_blinkscript_len);
  allOk &= ensureBlobIfMissing(FILE_ONSCRIPT, blob_onscript, blob_onscript_len);
  Console.print("Autogen:  ");
  Console.println(allOk ? "OK" : "some failures");
}

// ========== PSRAM smoke test ==========
static bool psramSafeSmokeTest() {
  if (g_storage != StorageBackend::PSRAM_BACKEND) {
    Console.println("psramSafeSmokeTest: active storage is not PSRAM — aborting");
    return false;
  }
  const uint32_t UPDATE_STEP_PERCENT = 5;
  const uint32_t MAX_SLOTS = 2044;
  const uint32_t PAGE = 256;
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
  uint32_t fsNext = 0, fsDataStart = 0;
  if (activeFs.nextDataAddr) fsNext = activeFs.nextDataAddr();
  if (activeFs.dataRegionStart) fsDataStart = activeFs.dataRegionStart();
  auto alignUp = [](uint32_t v, uint32_t a) -> uint32_t {
    return (v + (a - 1)) & ~(a - 1);
  };
  const uint32_t fsAligned = alignUp(fsNext, SECT);
  Console.printf("\nPSRAM multi-slot smoke: chips=%u per_chip=%u total=%u fsNext=%u fsAligned=%u\n",
                 (unsigned)chips, (unsigned)perChip, (unsigned)totalCap, (unsigned)fsNext, (unsigned)fsAligned);
  const uint32_t approxFree = (totalCap > fsAligned) ? (totalCap - fsAligned) : 0;
  Console.printf("Plan: allocate approx %u bytes of PSRAM free space\n", (unsigned)approxFree);
  if (approxFree == 0) {
    Console.println("PSRAM: no free space beyond FS; smoke test aborted");
    return false;
  }
  uint32_t stepsDesired = 100u / UPDATE_STEP_PERCENT;
  if (stepsDesired > MAX_SLOTS) {
    Console.printf("NOTE: stepsDesired=%u > MAX_SLOTS=%u, capping steps to %u\n",
                   (unsigned)stepsDesired, (unsigned)MAX_SLOTS, (unsigned)MAX_SLOTS);
    stepsDesired = MAX_SLOTS;
  }
  uint32_t numSlots = stepsDesired;
  uint32_t baseReserve = (approxFree + numSlots - 1) / numSlots;
  baseReserve = ((baseReserve + (SECT - 1)) & ~(SECT - 1));
  while ((uint64_t)baseReserve * numSlots > approxFree && numSlots > 1) {
    --numSlots;
    baseReserve = (approxFree + numSlots - 1) / numSlots;
    baseReserve = ((baseReserve + (SECT - 1)) & ~(SECT - 1));
  }
  if ((uint64_t)baseReserve * numSlots > approxFree) {
    Console.printf("Not enough free space for %u slots (need %u bytes, have %u). Aborting.\n",
                   (unsigned)numSlots, (unsigned)((uint64_t)baseReserve * numSlots), (unsigned)approxFree);
    return false;
  }
  Console.printf("Allocating %u slots of ~%u bytes each (aligned to %u)\n", (unsigned)numSlots, (unsigned)baseReserve, (unsigned)SECT);
  char slotName[32];
  uint32_t allocatedBytes = 0;
  uint32_t percentNextAlloc = UPDATE_STEP_PERCENT;
  uint32_t createdSlots = 0;
  Console.println("Stage: Allocate slots");
  for (uint32_t i = 0; i < numSlots; ++i) {
    snprintf(slotName, sizeof(slotName), ".span_part_%03u", (unsigned)i);
    if (activeFs.exists && activeFs.exists(slotName)) {
      if (activeFs.deleteFile) activeFs.deleteFile(slotName);
    }
    uint32_t reserve = baseReserve;
    uint32_t remainNeeded = (allocatedBytes >= approxFree) ? 0 : (approxFree - allocatedBytes);
    if (reserve > remainNeeded && i == numSlots - 1) {
      reserve = ((remainNeeded ? remainNeeded : SECT) + (SECT - 1)) & ~(SECT - 1);
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
  if (percentNextAlloc <= 100) {
    while (percentNextAlloc <= 100) {
      Console.printf("Alloc progress: %u%%\n", (unsigned)percentNextAlloc);
      percentNextAlloc += UPDATE_STEP_PERCENT;
    }
  }
  Console.println("Allocation complete");
  uint8_t page[PAGE];
  uint8_t readbuf[PAGE];
  uint64_t totalToWrite = approxFree;
  uint64_t totalWritten = 0;
  uint32_t globalNextPct = UPDATE_STEP_PERCENT;
  Console.println("Stage: Writing data into slots");
  for (uint32_t i = 0; i < createdSlots; ++i) {
    snprintf(slotName, sizeof(slotName), ".span_part_%03u", (unsigned)i);
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
      for (uint32_t k = 0; k < n; ++k) {
        page[k] = (uint8_t)(0xA5 ^ (uint8_t)i ^ (uint8_t)((slotWritten + k) & 0xFF));
      }
      if (!psramBB.writeData02(addr + slotWritten, page, n)) {
        Console.printf("  write failed for slot %u at offset %u\n", (unsigned)i, (unsigned)slotWritten);
        return false;
      }
      memset(readbuf, 0, sizeof(readbuf));
      if (!psramBB.readData03(addr + slotWritten, readbuf, n)) {
        Console.printf("  readback failed for slot %u at offset %u\n", (unsigned)i, (unsigned)slotWritten);
        return false;
      }
      if (memcmp(readbuf, page, n) != 0) {
        Console.printf("  verify mismatch slot %u offset %u\n", (unsigned)i, (unsigned)slotWritten);
        return false;
      }
      slotWritten += n;
      totalWritten += n;
      uint32_t slotPct = (uint32_t)(((uint64_t)slotWritten * 100u) / (uint64_t)toWrite);
      if (slotPct > 100) slotPct = 100;
      uint32_t globalPct = (uint32_t)(((uint64_t)totalWritten * 100u) / (uint64_t)totalToWrite);
      if (globalPct > 100) globalPct = 100;
      Console.printf("Global: %u%%, Slot %03u: %u%%, offset=%u, written=%u\n",
                     (unsigned)globalPct, (unsigned)i, (unsigned)slotPct, (unsigned)slotWritten, (unsigned)slotWritten);
      if ((slotWritten % SECT) == 0) yield();
    }
    Console.printf("Slot %03u complete: written=%u\n", (unsigned)i, (unsigned)toWrite);
  }
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