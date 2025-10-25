class ConsolePrint : public Print {
public:
  void begin() {}
  int printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int ret = vprintf_impl(fmt, ap);
    va_end(ap);
    return ret;
  }
  virtual size_t write(uint8_t b) override {
    return Serial.write(b);
  }
  virtual size_t write(const uint8_t* buffer, size_t size) override {
    if (!buffer || !size) return 0;
    return Serial.write(buffer, size);
  }
private:
  int vprintf_impl(const char* fmt, va_list ap) {
    if (!fmt) return 0;
    char buf[256];
    va_list aq;
    va_copy(aq, ap);
    int n = vsnprintf(buf, sizeof(buf), fmt, aq);
    va_end(aq);
    if (n < 0) return n;
    if ((size_t)n < sizeof(buf)) {
      write((const uint8_t*)buf, (size_t)n);
      return n;
    } else {
      char* big = (char*)malloc(n + 1);
      if (!big) {
        write((const uint8_t*)buf, sizeof(buf) - 1);
        return (int)(sizeof(buf) - 1);
      }
      vsnprintf(big, n + 1, fmt, ap);
      write((const uint8_t*)big, (size_t)n);
      free(big);
      return n;
    }
  }
};