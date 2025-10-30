#ifndef TEXTEDITOR_H_
#define TEXTEDITOR_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>

// Lightweight terminal-style text and hex editor
// Works over stdio by default; you can provide your own IO callbacks.

class TextEditorLib {
public:
  struct IO {
    // Optional file I/O overrides. If null, stdio-backed defaults are used.
    bool (*loadText)(const char* path, char** buffer, size_t* len_out);
    bool (*saveText)(const char* path, const char* buffer, size_t len);
    bool (*loadBin)(const char* path, uint8_t** buffer, size_t* len_out);
    bool (*saveBin)(const char* path, const uint8_t* buffer, size_t len);
    bool (*exists)(const char* path);
    bool (*remove)(const char* path);
    // Optional printing override (if null, uses printf/fputs)
    void (*print)(const char* s);
  };

  TextEditorLib(const IO* io = nullptr) {
    if (io) _io = *io;
    else setupDefaultIO();
  }

  // Open a text file and start an ED-like editing loop.
  bool editTextFile(const char* path) {
    if (!loadTextFromPath(path)) return false;
    _textPath = path;
    return runTextEditorLoop();
  }

  // Open a binary file and start a simple hex-editing loop.
  bool editBinaryFileHex(const char* path) {
    if (!loadBinFromPath(path)) return false;
    _binPath = path;
    return runHexEditorLoop();
  }

  // Dump current text buffer with 1-based line numbers.
  void printCurrentTextBuffer() {
    for (int i = 0; i < _textLineCount; ++i) {
      char line[256];
      int n = snprintf(line, sizeof(line), "%d: %s\n", i + 1, _textLines[i]);
      (void)n;
      if (_io.print) _io.print(line);
      else fputs(line, stdout);
    }
  }

  const char* currentTextPath() const {
    return _textPath;
  }
  const char* currentBinPath() const {
    return _binPath;
  }

private:
  // Tunables
  static const int MAX_TEXT_LINES = 512;
  static const int MAX_LINE_LEN = 128;
  static const int MAX_BIN_BYTES = 4096;

  // Text buffer
  char _textLines[MAX_TEXT_LINES][MAX_LINE_LEN];
  int _textLineCount = 0;
  const char* _textPath = nullptr;

  // Binary buffer
  uint8_t _binBuffer[MAX_BIN_BYTES];
  size_t _binLen = 0;
  const char* _binPath = nullptr;

  IO _io;

  // Default IO setup
  void setupDefaultIO() {
    _io.loadText = defaultLoadText;
    _io.saveText = defaultSaveText;
    _io.loadBin = defaultLoadBin;
    _io.saveBin = defaultSaveBin;
    _io.exists = defaultExists;
    _io.remove = defaultRemove;
    _io.print = defaultPrint;
  }

  // stdio-backed defaults
  static bool defaultLoadText(const char* path, char** buffer, size_t* len_out) {
    FILE* f = fopen(path, "r");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 0) {
      fclose(f);
      return false;
    }
    char* buf = (char*)malloc((size_t)sz + 1);
    if (!buf) {
      fclose(f);
      return false;
    }
    size_t rd = fread(buf, 1, (size_t)sz, f);
    buf[rd] = '\0';
    fclose(f);
    *buffer = buf;
    *len_out = rd;
    return true;
  }
  static bool defaultSaveText(const char* path, const char* buffer, size_t len) {
    FILE* f = fopen(path, "w");
    if (!f) return false;
    size_t w = fwrite(buffer, 1, len, f);
    fclose(f);
    return w == len;
  }
  static bool defaultLoadBin(const char* path, uint8_t** buffer, size_t* len_out) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 0) {
      fclose(f);
      return false;
    }
    uint8_t* buf = (uint8_t*)malloc((size_t)sz);
    if (!buf) {
      fclose(f);
      return false;
    }
    size_t rd = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    *buffer = buf;
    *len_out = rd;
    return true;
  }
  static bool defaultSaveBin(const char* path, const uint8_t* buffer, size_t len) {
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    size_t w = fwrite(buffer, 1, len, f);
    fclose(f);
    return w == len;
  }
  static bool defaultExists(const char* path) {
    FILE* f = fopen(path, "r");
    if (f) {
      fclose(f);
      return true;
    }
    return false;
  }
  static bool defaultRemove(const char* path) {
    return (::remove(path) == 0);
  }
  static void defaultPrint(const char* s) {
    if (s) fputs(s, stdout);
  }

  // Local file loaders using stdio (bounded buffers)
  bool loadTextFromPath(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return false;
    _textLineCount = 0;
    char tmp[MAX_LINE_LEN];
    while (fgets(tmp, sizeof(tmp), f) && _textLineCount < MAX_TEXT_LINES) {
      size_t len = strlen(tmp);
      // trim newline(s)
      while (len && (tmp[len - 1] == '\n' || tmp[len - 1] == '\r')) {
        tmp[--len] = '\0';
      }
      strncpy(_textLines[_textLineCount], tmp, MAX_LINE_LEN - 1);
      _textLines[_textLineCount][MAX_LINE_LEN - 1] = '\0';
      _textLineCount++;
    }
    fclose(f);
    return true;
  }

  bool saveTextToPath(const char* path) {
    FILE* f = fopen(path, "w");
    if (!f) return false;
    for (int i = 0; i < _textLineCount; ++i) {
      fprintf(f, "%s\n", _textLines[i]);
    }
    fclose(f);
    return true;
  }

  bool loadBinFromPath(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    _binLen = fread(_binBuffer, 1, MAX_BIN_BYTES, f);
    fclose(f);
    return true;
  }

  bool saveBinToPath(const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    size_t w = fwrite(_binBuffer, 1, _binLen, f);
    fclose(f);
    return w == _binLen;
  }

  // Read a line from stdin (or override by wrapping this method)
  int readLine(char* buf, int max) {
    if (!fgets(buf, max, stdin)) return 0;
    int len = (int)strlen(buf);
    // Strip trailing newline/CR
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
      buf[--len] = '\0';
    }
    return len;
  }

  // Text editor loop (ED/nano-ish minimal)
  bool runTextEditorLoop() {
    print(": text editor. Commands: :q, :w <path>, :p, :a, :i N, :d N, :h\n");
    const int CMD_BUF = 256;
    char linebuf[CMD_BUF];
    while (true) {
      print("ed> ");
      int len = readLine(linebuf, CMD_BUF);
      if (len <= 0) continue;

      if (linebuf[0] == ':') {
        const char* cmd = linebuf + 1;
        if (!strcmp(cmd, "q") || !strcmp(cmd, "quit")) {
          print("Exiting text editor.\n");
          return true;
        } else if (!strcmp(cmd, "h") || !strcmp(cmd, "help")) {
          print(
            "Text Editor commands:\n"
            "  :q or :quit        - exit editor (without saving)\n"
            "  :w <path>          - save text to path\n"
            "  :p                 - print current content\n"
            "  :a                 - append lines until '.' on a line\n"
            "  :i N               - insert after line N; then lines until '.'\n"
            "  :d N               - delete line N (1-based)\n"
            "  :h or :help        - show help\n");
        } else if (!strncmp(cmd, "w ", 2)) {
          const char* path = cmd + 2;
          bool ok = saveTextToPath(path);
          print(ok ? "Saved text to " : "Failed to save text to ");
          print(path);
          print("\n");
        } else if (!strcmp(cmd, "p")) {
          printCurrentTextBuffer();
        } else if (!strcmp(cmd, "a")) {
          print("Enter text to append. End with a single '.' on a line.\n");
          while (true) {
            print("> ");
            int r = readLine(linebuf, CMD_BUF);
            if (r <= 0) continue;
            if (!strcmp(linebuf, ".")) break;
            if (_textLineCount >= MAX_TEXT_LINES) {
              print("Text buffer full; cannot append more lines.\n");
              break;
            }
            strncpy(_textLines[_textLineCount], linebuf, MAX_LINE_LEN - 1);
            _textLines[_textLineCount][MAX_LINE_LEN - 1] = '\0';
            _textLineCount++;
          }
        } else if (!strncmp(cmd, "i ", 2)) {
          int after = atoi(cmd + 2);
          if (after < 0) after = 0;
          if (after > _textLineCount) after = _textLineCount;
          print("Enter lines to insert; end with '.' on a line.\n");

          char temp[MAX_TEXT_LINES][MAX_LINE_LEN];
          int addCnt = 0;
          while (true) {
            print("> ");
            int r = readLine(linebuf, CMD_BUF);
            if (r <= 0) continue;
            if (!strcmp(linebuf, ".")) break;
            if (addCnt >= MAX_TEXT_LINES) {
              print("Insert buffer full; stopping.\n");
              break;
            }
            strncpy(temp[addCnt], linebuf, MAX_LINE_LEN - 1);
            temp[addCnt][MAX_LINE_LEN - 1] = '\0';
            addCnt++;
          }

          if (addCnt > 0) {
            if (_textLineCount + addCnt > MAX_TEXT_LINES) {
              print("Not enough space to insert lines.\n");
            } else {
              int insertPos = after;  // 0-based
              for (int k = _textLineCount - 1; k >= insertPos; --k) {
                strncpy(_textLines[k + addCnt], _textLines[k], MAX_LINE_LEN);
                _textLines[k + addCnt][MAX_LINE_LEN - 1] = '\0';
              }
              for (int j = 0; j < addCnt; ++j) {
                strncpy(_textLines[insertPos + j], temp[j], MAX_LINE_LEN);
                _textLines[insertPos + j][MAX_LINE_LEN - 1] = '\0';
              }
              _textLineCount += addCnt;
            }
          }
        } else if (!strncmp(cmd, "d ", 2)) {
          int del = atoi(cmd + 2);
          if (del >= 1 && del <= _textLineCount) {
            int idx = del - 1;
            for (int t = idx; t < _textLineCount - 1; ++t) {
              strncpy(_textLines[t], _textLines[t + 1], MAX_LINE_LEN);
              _textLines[t][MAX_LINE_LEN - 1] = '\0';
            }
            _textLineCount--;
            _textLines[_textLineCount][0] = '\0';
          } else {
            print("Invalid line number to delete.\n");
          }
        } else {
          print("Unknown command. Type :h for help.\n");
        }
      } else {
        if (_textLineCount >= MAX_TEXT_LINES) {
          print("Text buffer full; cannot add more lines.\n");
        } else {
          strncpy(_textLines[_textLineCount], linebuf, MAX_LINE_LEN - 1);
          _textLines[_textLineCount][MAX_LINE_LEN - 1] = '\0';
          _textLineCount++;
        }
      }
    }
    // Unreachable
    // return true;
  }

  // Hex editor loop (basic)
  bool runHexEditorLoop() {
    print(": hex editor. Commands: show, set, fill, save <path>, q\n");
    const int CMD_BUF = 128;
    char linebuf[CMD_BUF];
    while (true) {
      print("hex> ");
      int len = readLine(linebuf, CMD_BUF);
      if (len <= 0) continue;

      if (!strcmp(linebuf, "q") || !strcmp(linebuf, "quit") || !strcmp(linebuf, "exit")) {
        print("Exiting hex editor.\n");
        return true;
      }
      if (!strcmp(linebuf, "help")) {
        print(
          "Hex Editor commands:\n"
          "  show                 - dump buffer (16 bytes/line)\n"
          "  set <off> <hex>      - set byte at offset (dec) to hex value (e.g., 1A)\n"
          "  fill <s> <n> <hex>   - fill [s, s+n) with value hex\n"
          "  save <path>          - write binary buffer to path\n"
          "  q                    - quit (no auto-save)\n");
        continue;
      }
      if (!strcmp(linebuf, "show")) {
        for (size_t i = 0; i < _binLen; i += 16) {
          char l[96];
          int p = snprintf(l, sizeof(l), "%06u: ", (unsigned)i);
          for (size_t j = 0; j < 16 && (i + j) < _binLen; ++j) {
            if (p < (int)sizeof(l)) p += snprintf(l + p, sizeof(l) - p, "%02X ", (unsigned)_binBuffer[i + j]);
          }
          if (_io.print) _io.print(l);
          else fputs(l, stdout);
          print("\n");
        }
        continue;
      }
      if (!strncmp(linebuf, "set ", 4)) {
        int off = 0;
        unsigned v = 0;
        if (sscanf(linebuf + 4, "%d %x", &off, &v) != 2 || v > 0xFF || off < 0 || (size_t)off >= _binLen) {
          print("Usage: set <offset> <hex>\n");
          continue;
        }
        _binBuffer[off] = (uint8_t)v;
        print("OK\n");
        continue;
      }
      if (!strncmp(linebuf, "fill ", 5)) {
        int s = 0, n = 0;
        unsigned v = 0;
        if (sscanf(linebuf + 5, "%d %d %x", &s, &n, &v) != 3 || v > 0xFF || s < 0 || n < 0 || (size_t)(s + n) > _binLen) {
          print("Usage: fill <start> <len> <hex>\n");
          continue;
        }
        for (int i = 0; i < n; ++i) _binBuffer[s + i] = (uint8_t)v;
        print("OK\n");
        continue;
      }
      if (!strncmp(linebuf, "save ", 5)) {
        const char* path = linebuf + 5;
        bool ok = saveBinToPath(path);
        print(ok ? "Saved hex buffer to " : "Failed to save to ");
        print(path);
        print("\n");
        continue;
      }
      print("Unknown command. Type 'help'.\n");
    }
  }

  // Helpers
  void print(const char* s) {
    if (_io.print) _io.print(s);
    else fputs(s, stdout);
  }
};

#endif  // TEXTEDITOR_H_