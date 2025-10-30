#ifndef TEXTEDITOR_H_
#define TEXTEDITOR_H_

// ----------------- Nano-like editor integration -----------------
namespace Term {
static inline void write(const char* s) {
  if (s) Serial.print(s);
}
static inline void writeChar(char c) {
  Serial.write((uint8_t)c);
}
static inline void clear() {
  write("\x1b[2J\x1b[H");
}
static inline void hideCursor() {
  write("\x1b[?25l");
}
static inline void showCursor() {
  write("\x1b[?25h");
}
static inline void move(int row1, int col1) {
  char buf[24];
  snprintf(buf, sizeof(buf), "\x1b[%d;%dH", row1, col1);
  write(buf);
}
static inline void clrEol() {
  write("\x1b[K");
}
static inline void invertOn() {
  write("\x1b[7m");
}
static inline void invertOff() {
  write("\x1b[0m");
}
}

struct NanoEditor {
  static const int MAX_LINES = 512;
  static const int MAX_COLS = 256;    // allow longer lines than console view
  static const int SCREEN_ROWS = 48;  // we could also assume a 80x24 terminal, edit as desired
  static const int SCREEN_COLS = 200;

  char lines[MAX_LINES][MAX_COLS];
  int lineCount = 0;
  int cx = 0;       // cursor x in current line (column, 0-based)
  int cy = 0;       // cursor y (line index, 0-based)
  int rowOff = 0;   // topmost visible line
  int colOff = 0;   // leftmost visible column
  int prefCol = 0;  // preferred column for up/down
  bool modified = false;
  char filename[ActiveFS::MAX_NAME + 1];

  // Load file (UTF-8 treated as bytes)
  bool load(const char* path) {
    filename[0] = 0;
    if (path) {
      strncpy(filename, path, sizeof(filename) - 1);
      filename[sizeof(filename) - 1] = 0;
    }
    lineCount = 0;
    if (!path || !activeFs.exists(path)) {
      lineCount = 1;
      lines[0][0] = 0;
      cx = cy = rowOff = colOff = prefCol = 0;
      modified = false;
      return true;
    }
    uint32_t sz = 0;
    if (!activeFs.getFileSize(path, sz)) return false;
    char* tmp = (char*)malloc(sz + 1);
    if (!tmp) return false;
    uint32_t got = activeFs.readFile(path, (uint8_t*)tmp, sz);
    if (got != sz) {
      free(tmp);
      return false;
    }
    tmp[sz] = 0;
    char* p = tmp;
    while (*p && lineCount < MAX_LINES) {
      char* e = p;
      while (*e && *e != '\n' && *e != '\r') ++e;
      int L = (int)(e - p);
      if (L >= MAX_COLS) L = MAX_COLS - 1;
      memcpy(lines[lineCount], p, L);
      lines[lineCount][L] = 0;
      lineCount++;
      while (*e == '\n' || *e == '\r') ++e;
      p = e;
    }
    if (lineCount == 0) {
      lineCount = 1;
      lines[0][0] = 0;
    }
    free(tmp);
    cx = cy = rowOff = colOff = prefCol = 0;
    modified = false;
    return true;
  }

  // Replace the existing NanoEditor::saveAs with this slot-safe version
  bool saveAs(const char* path) {
    if (!path || !checkNameLen(path)) return false;

    // Join buffer into a single blob with '\n'
    uint32_t total = 0;
    for (int i = 0; i < lineCount; ++i) total += (uint32_t)strlen(lines[i]) + 1;

    uint8_t* out = (uint8_t*)malloc(total ? total : 1);
    if (!out) return false;

    uint32_t off = 0;
    for (int i = 0; i < lineCount; ++i) {
      size_t L = strlen(lines[i]);
      memcpy(out + off, lines[i], L);
      off += (uint32_t)L;
      out[off++] = '\n';
    }

    // Sector-align the reserve to keep slotSafe
    auto alignUp = [](uint32_t v, uint32_t a) -> uint32_t {
      return (v + (a - 1)) & ~(a - 1);
    };
    uint32_t reserve = alignUp(off, ActiveFS::SECTOR_SIZE);
    if (reserve == 0) reserve = ActiveFS::SECTOR_SIZE;

    // Delete old file (if any) and create a fresh slot with initial contents
    if (activeFs.exists(path)) {
      (void)activeFs.deleteFile(path);
    }
    bool ok = activeFs.createFileSlot(path, reserve, out, off);
    free(out);

    if (ok) {
      strncpy(filename, path, sizeof(filename) - 1);
      filename[sizeof(filename) - 1] = 0;
      modified = false;
    }
    return ok;
  }

  // UI
  void drawStatus(const char* msg) {
    Term::move(SCREEN_ROWS - 1, 1);
    Term::invertOn();
    // status line
    char status[128];
    snprintf(status, sizeof(status), " %s %s  Ln %d, Col %d  %s",
             "Nano-like Editor",
             filename[0] ? filename : "[No Name]",
             cy + 1, cx + 1,
             modified ? "(modified)" : "");
    Serial.print(status);
    Term::clrEol();
    Term::invertOff();
    // help bar
    Term::move(SCREEN_ROWS, 1);
    Serial.print("^X Exit  ^C Pos  Arrows Move  Backspace/Delete Edit  Enter Newline");
    Term::clrEol();
    // message (overlay above help bar if provided)
    if (msg && *msg) {
      Term::move(SCREEN_ROWS - 2, 1);
      Serial.print(msg);
      Term::clrEol();
    } else {
      Term::move(SCREEN_ROWS - 2, 1);
      Term::clrEol();
    }
  }
  void drawRows() {
    // Draw from row 1 to SCREEN_ROWS-2 (reserve bottom 2 rows for bars)
    int usable = SCREEN_ROWS - 2;
    for (int i = 0; i < usable; ++i) {
      int fileRow = rowOff + i;
      Term::move(1 + i, 1);
      Term::clrEol();
      if (fileRow >= lineCount) {
        Serial.print("~");
      } else {
        const char* s = lines[fileRow];
        int len = (int)strlen(s);
        if (len > colOff) {
          const char* p = s + colOff;
          int toShow = len - colOff;
          if (toShow < 0) toShow = 0;
          if (toShow > SCREEN_COLS) toShow = SCREEN_COLS;
          Serial.write((const uint8_t*)p, toShow);
        }
      }
    }
  }
  void refresh(const char* msg = nullptr) {
    Term::hideCursor();
    Term::move(1, 1);
    drawRows();
    drawStatus(msg);
    // place cursor
    int crow = cy - rowOff + 1;
    int ccol = cx - colOff + 1;
    if (crow < 1) crow = 1;
    if (ccol < 1) ccol = 1;
    if (crow > SCREEN_ROWS - 2) crow = SCREEN_ROWS - 2;
    if (ccol > SCREEN_COLS) ccol = SCREEN_COLS;
    Term::move(crow, ccol);
    Term::showCursor();
  }

  // Editing actions
  void moveLeft() {
    if (cx > 0) {
      cx--;
      prefCol = cx;
      return;
    }
    if (cy > 0) {
      cy--;
      cx = (int)strlen(lines[cy]);
      prefCol = cx;
      if (rowOff > cy) rowOff = cy;
      ensureCursorVisible();
    }
  }
  void moveRight() {
    int L = (int)strlen(lines[cy]);
    if (cx < L) {
      cx++;
      prefCol = cx;
      ensureCursorVisible();
      return;
    }
    if (cy + 1 < lineCount) {
      cy++;
      cx = 0;
      prefCol = cx;
      ensureCursorVisible();
    }
  }
  void moveUp() {
    if (cy > 0) cy--;
    int L = (int)strlen(lines[cy]);
    if (prefCol > L) cx = L;
    else cx = prefCol;
    ensureCursorVisible();
  }
  void moveDown() {
    if (cy + 1 < lineCount) cy++;
    int L = (int)strlen(lines[cy]);
    if (prefCol > L) cx = L;
    else cx = prefCol;
    ensureCursorVisible();
  }
  void ensureCursorVisible() {
    // Horizontal
    if (cx < colOff) colOff = cx;
    if (cx >= colOff + SCREEN_COLS) colOff = cx - SCREEN_COLS + 1;
    // Vertical
    if (cy < rowOff) rowOff = cy;
    if (cy >= rowOff + (SCREEN_ROWS - 2)) rowOff = cy - (SCREEN_ROWS - 2) + 1;
  }
  void insertChar(char c) {
    char* line = lines[cy];
    int L = (int)strlen(line);
    if (L >= MAX_COLS - 1) return;  // full, ignore
    if (cx > L) cx = L;
    // shift right
    for (int i = L; i >= cx; --i) line[i + 1] = line[i];
    line[cx] = c;
    cx++;
    prefCol = cx;
    modified = true;
    ensureCursorVisible();
  }
  void backspace() {
    if (cx > 0) {
      char* line = lines[cy];
      int L = (int)strlen(line);
      for (int i = cx - 1; i < L; ++i) line[i] = line[i + 1];
      cx--;
      prefCol = cx;
      modified = true;
      return;
    }
    // at start of line: join with previous if any
    if (cy > 0) {
      int prevL = (int)strlen(lines[cy - 1]);
      int curL = (int)strlen(lines[cy]);
      int canCopy = min(MAX_COLS - 1 - prevL, curL);
      // append as much as fits
      memcpy(lines[cy - 1] + prevL, lines[cy], canCopy);
      lines[cy - 1][prevL + canCopy] = 0;
      // remove this line
      for (int i = cy; i < lineCount - 1; ++i) {
        strcpy(lines[i], lines[i + 1]);
      }
      lineCount--;
      cy--;
      cx = prevL;
      prefCol = cx;
      modified = true;
      ensureCursorVisible();
    }
  }
  void delChar() {
    char* line = lines[cy];
    int L = (int)strlen(line);
    if (cx < L) {
      for (int i = cx; i < L; ++i) line[i] = line[i + 1];
      modified = true;
      return;
    }
    // at end of line: join with next line
    if (cy + 1 < lineCount) {
      int curL = (int)strlen(lines[cy]);
      int nextL = (int)strlen(lines[cy + 1]);
      int canCopy = min(MAX_COLS - 1 - curL, nextL);
      memcpy(lines[cy] + curL, lines[cy + 1], canCopy);
      lines[cy][curL + canCopy] = 0;
      // shift lines up
      for (int i = cy + 1; i < lineCount - 1; ++i) {
        strcpy(lines[i], lines[i + 1]);
      }
      lineCount--;
      modified = true;
    }
  }
  void newline() {
    if (lineCount >= MAX_LINES) return;
    char* line = lines[cy];
    int L = (int)strlen(line);
    // split at cx
    char tail[MAX_COLS];
    int tailLen = (cx < L) ? (L - cx) : 0;
    if (tailLen > 0) {
      memcpy(tail, line + cx, tailLen);
    }
    tail[tailLen] = 0;
    line[cx] = 0;  // truncate current
    // shift lines down to insert
    for (int i = lineCount; i > cy + 1; --i) {
      strcpy(lines[i], lines[i - 1]);
    }
    strcpy(lines[cy + 1], tail);
    lineCount++;
    cy++;
    cx = 0;
    prefCol = 0;
    modified = true;
    ensureCursorVisible();
  }

  // read a key, parsing ESC sequences for arrows and delete
  int readKey() {
    for (;;) {
      if (Serial.available() > 0) {
        int ch = Serial.read();
        if (ch == '\r') {
          // swallow optional '\n'
          if (Serial.available() && Serial.peek() == '\n') Serial.read();
          return '\n';
        }
        if (ch == 0x1B) {  // ESC
          // parse CSI
          while (!Serial.available()) {
            Exec.pollBackground();
            tight_loop_contents();
            yield();
          }
          int ch1 = Serial.read();
          if (ch1 == '[') {
            while (!Serial.available()) {
              Exec.pollBackground();
              tight_loop_contents();
              yield();
            }
            int ch2 = Serial.read();
            if (ch2 == 'A') return 1000;  // up
            if (ch2 == 'B') return 1001;  // down
            if (ch2 == 'C') return 1002;  // right
            if (ch2 == 'D') return 1003;  // left
            if (ch2 == '3') {
              // expect ~
              while (!Serial.available()) {
                Exec.pollBackground();
                tight_loop_contents();
                yield();
              }
              int tilde = Serial.read();
              if (tilde == '~') return 1004;  // delete
            }
            // ignore other CSI
            return -1;
          } else {
            // lone ESC or other sequences: ignore
            return -1;
          }
        }
        return ch;
      }
      Exec.pollBackground();
      tight_loop_contents();
      yield();
    }
  }

  // prompt Yes/No, returns 1 for yes, 0 for no, -1 for cancel (Esc)
  int promptYesNo(const char* q) {
    refresh(q);
    for (;;) {
      int k = readKey();
      if (k == 'y' || k == 'Y') return 1;
      if (k == 'n' || k == 'N') return 0;
      if (k == 27) return -1;
    }
  }
  // prompt for filename (default shown), return true if got a name in out
  bool promptFilename(char* out, size_t outCap) {
    char buf[ActiveFS::MAX_NAME + 1];
    buf[0] = 0;
    if (filename[0]) strncpy(buf, filename, sizeof(buf) - 1);
    size_t len = strlen(buf);
    char msg[96];
    snprintf(msg, sizeof(msg), "File Name to Write: %s", buf);
    refresh(msg);
    for (;;) {
      int k = readKey();
      if (k == '\n') {
        if (len == 0) return false;
        strncpy(out, buf, outCap - 1);
        out[outCap - 1] = 0;
        return true;
      } else if (k == 27) {
        return false;
      } else if (k == 0x08 || k == 0x7F) {
        if (len > 0) {
          buf[--len] = 0;
        }
      } else if (k >= 32 && k <= 126) {
        if (len + 1 < sizeof(buf)) {
          buf[len++] = (char)k;
          buf[len] = 0;
        }
      }
      snprintf(msg, sizeof(msg), "File Name to Write: %s", buf);
      refresh(msg);
    }
  }

  bool run() {
    Term::clear();
    refresh("Welcome to Nano-like editor. ^X exit, ^C cursor pos");
    for (;;) {
      int k = readKey();
      if (k == 3) {  // Ctrl-C -> show pos
        char m[64];
        snprintf(m, sizeof(m), "Cursor position: Ln %d, Col %d", cy + 1, cx + 1);
        refresh(m);
        continue;
      }
      if (k == 24) {  // Ctrl-X -> prompt save then exit
        int ans = modified ? promptYesNo("Save modified buffer? (Y/N)") : 0;
        if (ans == 1) {
          char outname[ActiveFS::MAX_NAME + 1];
          if (!promptFilename(outname, sizeof(outname))) {
            refresh("Save canceled.");
            continue;  // cancel save, stay
          }
          if (!checkNameLen(outname)) {
            refresh("Error: filename too long.");
            continue;
          }
          if (saveAs(outname)) {
            refresh("Wrote file. Exiting.");
          } else {
            refresh("Write failed! Press any key to continue.");
            (void)readKey();
          }
        }
        // if ans == 0 (No) or not modified, exit without saving
        Term::clear();
        Term::showCursor();
        return true;
      }
      if (k == '\n') {
        newline();
        refresh(nullptr);
        continue;
      }
      if (k == 1000) {
        moveUp();
        refresh(nullptr);
        continue;
      }
      if (k == 1001) {
        moveDown();
        refresh(nullptr);
        continue;
      }
      if (k == 1002) {
        moveRight();
        refresh(nullptr);
        continue;
      }
      if (k == 1003) {
        moveLeft();
        refresh(nullptr);
        continue;
      }
      if (k == 1004) {
        delChar();
        refresh(nullptr);
        continue;
      }
      if (k == 0x08 || k == 0x7F) {
        backspace();
        refresh(nullptr);
        continue;
      }
      if (k >= 32 && k <= 126) {
        insertChar((char)k);
        refresh(nullptr);
        continue;
      }
      // ignore other keys
    }
  }
};

static bool runNanoEditor(const char* path) {
  NanoEditor ed;
  if (!ed.load(path)) {
    Console.println("nano: load failed");
    return false;
  }
  bool ok = ed.run();
  return ok;
}
// ----------------- End Nano-like editor -----------------

#endif  // TEXTEDITOR_H_