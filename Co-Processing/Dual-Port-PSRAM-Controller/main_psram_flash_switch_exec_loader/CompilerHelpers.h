// ========== Compile Tiny-C source file -> raw Thumb binary on FS ==========
static void printCompileErrorContext(const char* src, size_t srcLen, size_t pos) {
  const size_t CONTEXT = 40;
  if (!src || srcLen == 0) return;
  size_t start = (pos > CONTEXT) ? (pos - CONTEXT) : 0;
  size_t end = (pos + CONTEXT < srcLen) ? (pos + CONTEXT) : srcLen;
  Console.println("----- context -----");
  for (size_t i = start; i < end; ++i) {
    char c = src[i];
    if (c == '\r') c = ' ';
    Console.print(c);
  }
  Console.println();
  size_t caret = pos - start;
  for (size_t i = 0; i < caret; ++i) Console.print(' ');
  Console.println("^");
  Console.println("-------------------");
}
static bool compileTinyCFileToFile(const char* srcName, const char* dstName) {
  if (!checkNameLen(srcName) || !checkNameLen(dstName)) return false;
  if (!activeFs.exists(srcName)) {
    Console.print("compile: source not found: ");
    Console.println(srcName);
    return false;
  }
  uint32_t srcSize = 0;
  if (!activeFs.getFileSize(srcName, srcSize) || srcSize == 0) {
    Console.println("compile: getFileSize failed or empty source");
    return false;
  }
  // Read source
  char* srcBuf = (char*)malloc(srcSize + 1);
  if (!srcBuf) {
    Console.println("compile: malloc src failed");
    return false;
  }
  uint32_t got = activeFs.readFile(srcName, (uint8_t*)srcBuf, srcSize);
  if (got != srcSize) {
    Console.println("compile: readFile failed");
    free(srcBuf);
    return false;
  }
  srcBuf[srcSize] = 0;
  // Prepare output buffer
  uint32_t outCap = (srcSize * 12u) + 256u;
  if (outCap < 512u) outCap = 512u;
  uint8_t* outBuf = (uint8_t*)malloc(outCap);
  if (!outBuf) {
    Console.println("compile: malloc out failed");
    free(srcBuf);
    return false;
  }
  MCCompiler comp;
  size_t outSize = 0;
  MCCompiler::Result r = comp.compile(srcBuf, srcSize, outBuf, outCap, &outSize);
  if (!r.ok) {
    Console.print("compile: error at pos ");
    Console.print((uint32_t)r.errorPos);
    Console.print(": ");
    Console.println(r.errorMsg ? r.errorMsg : "unknown");
    printCompileErrorContext(srcBuf, srcSize, r.errorPos);
    free(outBuf);
    free(srcBuf);
    return false;
  }
  // Write to destination file
  bool ok = writeBinaryToFS(dstName, outBuf, (uint32_t)outSize);
  if (ok) {
    Console.print("compile: OK -> ");
    Console.print(dstName);
    Console.print(" (");
    Console.print((uint32_t)outSize);
    Console.println(" bytes)");
    if (outSize & 1u) {
      Console.println("note: odd-sized output; for Thumb execution, even size is recommended.");
    }
  } else {
    Console.println("compile: write failed");
  }
  free(outBuf);
  free(srcBuf);
  return ok;
}