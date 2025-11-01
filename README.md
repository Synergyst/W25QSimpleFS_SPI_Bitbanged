# 🤖 toyOS + Fun Libraries
## 🌟 Overview
This repository is a practical embedded “lab” for RP2040/RP2350-based systems that combines:
- Unified SPI memory with lightweight file systems over:
  - SPI NOR Flash (W25Q)
  - Multi-bank PSRAM
  - SPI-NAND (MX35)
- A framed, CRC-protected RPC link to a co-processor over software serial
- A dual-path execution model: raw binary “blob” executor + a small script language (CoProcLang)
- A modernized serial console:
  - In-line editing with arrow/Home/End/Delete keys
  - History in SRAM (arrow-up/down)
  - Rich file/folder commands (mv/cp/fscp/ls/df/…)
  - Built-in Nano-like text editor with ANSI UI
- Background job control with timeouts, cancellation, and a shared mailbox
- Hardware glue for multi-bank PSRAM using a 74HC138 decoder or 74HC138 + 74HC595, plus in-system programming (ISP) support

It’s equal parts “toy OS” exploration and a collection of reusable embedded utilities that solve real constraints in memory, transport, and execution on small MCUs.

---

## ✨ Core Components
### 1) Storage and File Systems
- UnifiedSpiMem + SimpleFS facades
  - W25QUnifiedSimpleFS (SPI NOR Flash)
  - PSRAMUnifiedSimpleFS (volatile)
  - MX35UnifiedSimpleFS (SPI-NAND)
- FS features
  - Slot-based layout with sector/page alignment
  - Replace-in-place where possible; reserve sizing aligned to device erase size
  - Simple folder emulation using “marker” entries (path strings with ‘/’)
  - Tools and introspection for directory tables (ls, lsraw, lsdebug)
- CLI commands (highlights)
  - files, info, dump, mkSlot, writeblob, autogen
  - df (device/FS usage), ls, mkdir, rmdir [-r], touch
  - mv, cp [-f] (intra-FS), fscp [-f] (cross-FS), del/rm
- PSRAM helpers
  - Capacity reporting and a safe, non-destructive smoke test
  - Multi-bank aggregation with 74HC138/74HC595 glue

Hardware chip select options:
- Direct chip select
- 74HC138-based CS (configurable pins)
- 74HC138 + 74HC595-based CS (configurable pins)
- Optional clock settle delays for reliable bus transactions

### 2) Execution and Control
- ExecHost (Main CPU)
  - Manages local FS, stages binaries/scripts, and drives the co-processor via RPC
  - Foreground and background execution flows with a configurable timeout override
  - Shared mailbox buffer for small result/status passing
- CoProcExec + CoProcLang (Co-Processor)
  - Binary blob executor (Thumb) on core1 with argument passing
  - Script engine (CoProcLang) with I/O primitives (delays, pin ops, etc.)
  - Cancellation via mailbox flag; execution state reporting (idle, loaded, running, done)
  - Named function dispatch (CMD_FUNC) with strict argc checking

### 3) Transport: Framed RPC over Software Serial
- CoProcProto
  - 24-byte header + payload + CRC32(payload)
  - Versioned, CRC-protected, command-based request/response
  - Commands include:
    - HELLO, INFO
    - LOAD_BEGIN/DATA/END, EXEC
    - STATUS, MAILBOX_RD, CANCEL, RESET
    - SCRIPT_BEGIN/DATA/END, SCRIPT_EXEC (ASCII arg mode supported)
    - ISP_ENTER, ISP_EXIT
    - FUNC (named function dispatch on co-processor)
- Defaults
  - UART over two GPIOs using SoftwareSerial
  - 8N1 framing, default 230400 baud (RPC link)

### 4) Console & Editor
- Console UX (main serial at 115200 baud)
  - In-line editing: Left/Right, Home/End, Delete, Backspace
  - History in SRAM (16 entries, Up/Down with preservation of in-progress input)
  - Extras: Ctrl+A (home), Ctrl+E (end), Ctrl+K (kill to end), Ctrl+U (clear), Ctrl+L (redraw)
- Cross-filesystem copy
  - fscp <srcFS:path> <dstFS:path|folder/> [-f], where FS ∈ {flash, psram, nand}
- Nano-like text editor
  - ANSI UI with auto screen-size detection (ESC[6n), fallback to configurable defaults
  - Line numbers (toggle: Ctrl+N), status and help bars, percent-through-file
  - Navigation: arrows, Home/End, PageUp/PageDown, word left/right (Ctrl+←/→)
  - Editing: insert/backspace/delete, newline with auto-indent from previous line
  - Search (Ctrl+F), goto line (Ctrl+G), save (Ctrl+S), exit (Ctrl+X with save prompt)
  - Visual cursor alignment fixed in line-number mode

---

## 🧠 Architecture (at a glance)
- Main CPU
  - Manages Flash/PSRAM/SPI-NAND FS
  - Presents a rich serial console (help text, file ops, blob/script staging, diagnostics, editor)
  - Drives the co-processor RPC (frame building, CRC, responses)
  - Background job manager (submit, query, cancel)
- Co-Processor (RP2040/2350)
  - Core0: framed serial RPC loop + handlers
  - Core1: execution worker (blob/script)
  - Shared mailbox and cancel flag
  - Optional ISP service loop while idle in transport

---

## 🔌 Hardware Notes (defaults)
- Flash/PSRAM SPI (shared bus):
  - SCK=GP10, MOSI=GP11, MISO=GP12
  - Example CS pins: FLASH=GP9, PSRAM banks {GP14, GP15, GP26, GP27}, NAND=GP28
- PSRAM banking:
  - Direct CS, 74HC138, or 74HC138+74HC595 (configurable pins)
- Co-Processor soft-serial (both sides):
  - RX=GP0, TX=GP1
- Baud
  - Console: 115200 (main)
  - RPC: 230400 (configurable)

---

## 🛠️ Getting Started
### Requirements
- RP2040 or RP2350 boards (Earle Philhower Arduino core recommended)
- Arduino IDE (tested with 2.3.2) + Philhower core/toolchain
- Hardware:
  - SPI NOR Flash (W25Q series or compatible) for non-volatile FS
  - Optional PSRAM (single/multi-bank) and optional SPI-NAND (MX35) for larger capacities

### Build and Flash
- Co-Processor firmware
  - Sketch: main_coproc_softserial.ino
  - Board: RP2040/RP2350 via Earle Philhower core
  - Upload to the co-processor board
- Main CPU firmware
  - Sketch: main_psram_flash_switch_exec_loader.ino
  - Choose active storage at runtime (storage flash|psram|nand)
  - Upload to the main board
- Console
  - Open a 115200 baud ANSI-capable terminal to the main CPU
  - Type `help` to see supported commands

Tip: ANSI-capable terminals (Linux/macOS Terminal, Windows Terminal, etc.) unlock full editor UI and line-editing. If ANSI responses aren’t available, the editor falls back to predefined dimensions.

---

## 🧪 Typical Workflows
### 1) Put a blob into storage and execute it on the co-processor
- Upload to active FS:
  - putb64 <file> <base64> or puthex <file> <hex>
- Execute:
  - `exec <file> [args] [&]` (local background/foreground)
  - `coproc exec <file> [args]` (stage via RPC then run)

### 2) Send and run a script on the co-processor
- Place script in FS:
  - `coproc sexec <file> [args]`

### 3) Call a named function on the co-processor
- From the main CPU console:
  - `coproc func <name> [a0..aN]` → returns status + int32 result

### 4) Copy across filesystems
- Cross-FS:
  - `fscp flash:/app nand:/backup/ -f`
- Intra-FS:
  - `cp src dst [-f]`, `mv src dst`

### 5) Edit files on-device
- Open editor:
  - `nano <file>` or `edit <file>`
- Key highlights:
  - Save: Ctrl+S
  - Exit: Ctrl+X (prompts to save)
  - Line numbers toggle: Ctrl+N
  - Find: Ctrl+F, Goto: Ctrl+G
  - Navigation: arrows, Home/End, PgUp/PgDn, Ctrl+←/→ for words

---

## 📦 Protocol Essentials (CoProcProto)
- Header: 24 bytes (MAGIC ‘CPR0’, VERSION, CMD, SEQ, LEN, CRC32)
- Payload CRC: IEEE 802.3 polynomial
- Response CMD = request CMD | 0x80
- Status-first payloads (int32 status followed by any result fields)
- Notable commands:
  - EXEC / SCRIPT_EXEC: argc + args, plus timeout; optional ASCII arg encoding
  - FUNC: uint32 name_len + bytes[name] + uint32 argc + argv
    - Response: int32 status, int32 result

---

## 🧰 Console Highlights (Main CPU)
Examples (115200 baud):
- File operations:
  - `files`, `info <file>`, `dump <file> <n>`, `mkSlot <file> <reserve>`, `del <file>`
  - `writeblob <file> <blobId>`, `autogen`
  - `mv <src> <dst|folder/>`, `cp <src> <dst|folder/> [-f]`, `fscp <sFS:path> <dFS:path|folder/> [-f]`
  - Folders: `pwd`, `cd`, `mkdir`, `ls [path]`, `rmdir <path> [-r]`, `touch <path|folder/>`
  - `df` (device + FS usage)
- Execution:
  - `exec <file> [a0..aN] [&]`
  - `bg status|query|kill|cancel`
- Co-Processor:
  - `coproc ping | info | status | cancel | reset`
  - `coproc exec <file> [a0..aN]`
  - `coproc sexec <file> [a0..aN]`
  - `coproc func <name> [a0..aN]`
  - `coproc mbox [n]`
  - `coproc isp enter|exit`
- System:
  - `storage [flash|psram|nand]`, `format`, `wipe`, `wipereboot`, `wipebootloader`
  - `timeout [ms]`, `meminfo`, `psramsmoketest`, `reboot`
- Editor:
  - `nano <file>` / `edit <file>`

Prebuilt blobs/scripts are included and can be auto-created in storage with `autogen` to simplify initial testing.

---

## ⚠️ Notes and Caveats
- SimpleFS path limit: full path (including folders) must be ≤ 32 chars
- Blob execution expects even byte length for Thumb entry alignment
- Only one background job is supported at a time
- Cancellation is cooperative (jobs should poll the cancel flag)
- Editor renders tab characters as-is; terminals show them at tab stops (usually 8 columns). This can visually offset the cursor on lines containing tabs. Use spaces or we can add optional tab expansion if needed.

---

## 🤝 Contribution
Issues and PRs are welcome. If you extend storage backends, add script opcodes, improve protocol robustness, or enhance the console/editor, please share the changes. Clear repro steps and hardware notes are appreciated to keep the tooling reproducible.
