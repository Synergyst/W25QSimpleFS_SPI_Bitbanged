DECLARE_ASCII_SCRIPT(blob_blinkscript, R"(# Blink GP2 three times, 200ms on, 150ms off, return 7
PINMODE 2 OUT
LET R0 0
LOOP:
DWRITE 2 1
DELAY 20
DWRITE 2 0
DELAY 20
ADD R0 1
IF R0 < 20 GOTO LOOP
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