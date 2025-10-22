// gpio_pwm_cycles.cpp
#include <stdint.h>
#include <stddef.h>

#define IO_BANK0_GPIO0_CTRL   0x40014004u  // + (pin << 3)
#define SIO_GPIO_OUT_SET      0xD0000014u
#define SIO_GPIO_OUT_CLR      0xD0000018u
#define SIO_GPIO_OE_SET       0xD0000024u
#define TIMERAWL_ADDR         0x40054028u  // 1 MHz free-running timer

#ifndef EXIT_OK
#define EXIT_OK 0
#endif
#ifndef EXIT_INVALID_PIN
#define EXIT_INVALID_PIN (-1)
#endif

#ifndef INFO_ADDR
#define INFO_ADDR 0x20041000u  // default: Scratch Y base
#endif
#ifndef INFO_MAX
#define INFO_MAX 512u          // reserved bytes in mailbox
#endif

static inline __attribute__((always_inline)) void delay_ms(int32_t ms) {
    if (ms <= 0) return;
    volatile uint32_t* const timerawl = (volatile uint32_t*)TIMERAWL_ADDR;
    const uint32_t us_per_ms = 1000u;
    while (ms-- > 0) {
        uint32_t start = *timerawl;
        while ((uint32_t)(*timerawl - start) < us_per_ms) { }
    }
}

// Copy an embedded info string to the fixed mailbox; return INFO_ADDR
static inline __attribute__((always_inline)) uintptr_t provide_info_ptr() {
    const char* src;
    // Place ADR before the string (forward); align string to 4 for ADR T1 encoding on M0+
    asm volatile(
        "adr %0, 1f\n"
        "b 2f\n"
        ".balign 4\n"
        "1:\n"
        // Keep this short enough to fit INFO_MAX-1; NUL-terminated
        ".ascii \"gpio_pwm_cycles: entry(pin,low_ms,high_ms,cycles). Starts+ends LOW. pin[0..29].\\0\"\n"
        ".balign 2\n"
        "2:\n"
        : "=r"(src)
        :
        : "cc"
    );

    volatile char* dst = (volatile char*)(uintptr_t)(INFO_ADDR);
    uint32_t cap = (INFO_MAX ? (INFO_MAX - 1u) : 0u);
    uint32_t i = 0;
    while (cap && src[i]) { dst[i] = src[i]; ++i; --cap; }
    dst[i] = '\0';
    return (uintptr_t)(INFO_ADDR);
}

extern "C" int32_t entry(int32_t pin, int32_t low_ms, int32_t high_ms, int32_t cycles) {
    // No-params => write info and return pointer to mailbox
    if ((pin | low_ms | high_ms | cycles) == 0) {
        return (int32_t)(uintptr_t)provide_info_ptr();
    }

    // Validate pin
    if (pin < 0 || pin > 29) return EXIT_INVALID_PIN;

    // Configure FUNCSEL=5 (SIO) for this pin
    volatile uint32_t* const gpio_ctrl =
        (volatile uint32_t*)(IO_BANK0_GPIO0_CTRL + ((uint32_t)pin << 3));
    *gpio_ctrl = 5u;

    const uint32_t mask = (1u << (uint32_t)pin);

    // Enable output and start LOW
    *(volatile uint32_t*)SIO_GPIO_OE_SET  = mask;
    *(volatile uint32_t*)SIO_GPIO_OUT_CLR = mask;

    if (cycles < 0) cycles = 0;

    while (cycles-- > 0) {
        if (low_ms != 0) {
            *(volatile uint32_t*)SIO_GPIO_OUT_CLR = mask;
            delay_ms(low_ms);
        }
        if (high_ms != 0) {
            *(volatile uint32_t*)SIO_GPIO_OUT_SET = mask;
            delay_ms(high_ms);
        }
    }

    // Ensure final LOW
    *(volatile uint32_t*)SIO_GPIO_OUT_CLR = mask;

    // Also write status/info to mailbox on normal runs
    (void)provide_info_ptr();

    return EXIT_OK;
}
