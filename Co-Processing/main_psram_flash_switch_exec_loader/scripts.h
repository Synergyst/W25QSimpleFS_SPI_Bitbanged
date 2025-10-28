DECLARE_ASCII_SCRIPT(blob_blinkscript, R"(# Blink GP2 three times, 200ms on, 150ms off, return 7
PINMODE 2 OUT
LET R0 0
LOOP:
DWRITE 2 1
DELAY 3000
DWRITE 2 0
DELAY 3500
DWRITE 2 0
ADD R0 1
DWRITE 2 0
IF R0 < 3 GOTO LOOP
MBCLR
MBAPP "Blink done"
RET 7
)");

DECLARE_ASCII_SCRIPT(blob_onscript, R"(# Toggle GP2 on, return 7
PINMODE 2 OUT
DWRITE 2 1
MBCLR
MBAPP "Toggled GP2 on"
RET 7
)");