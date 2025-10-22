// gpio_pwm_cycles.c or .cpp
#include <stdint.h>

#define IO_BANK0_GPIO0_CTRL   0x40014004u  // + (pin << 3)
#define SIO_GPIO_OUT_SET      0xD0000014u
#define SIO_GPIO_OUT_CLR      0xD0000018u
#define SIO_GPIO_OE_SET       0xD0000024u
#define TIMERAWL_ADDR         0x40054028u  // 1 MHz free-running timer

static inline __attribute__((always_inline)) void delay_ms(int32_t ms) {
    if (ms <= 0) return;
    volatile uint32_t* const timerawl = (volatile uint32_t*)TIMERAWL_ADDR;
    const uint32_t us_per_ms = 1000u;
    while (ms-- > 0) {
        uint32_t start = *timerawl;
        while ((uint32_t)(*timerawl - start) < us_per_ms) {
            // busy-wait
        }
    }
}

#ifdef __cplusplus
extern "C" {
#endif

// entry(pin, low_ms, high_ms, cycles)
// Returns 0 on success, -1 if pin not in [0..29]
int32_t entry(int32_t pin, int32_t low_ms, int32_t high_ms, int32_t cycles) {
    if (pin < 0 || pin > 29) return -1;

    // FUNCSEL = 5 (SIO)
    volatile uint32_t* const gpio_ctrl =
        (volatile uint32_t*)(IO_BANK0_GPIO0_CTRL + ((uint32_t)pin << 3));
    *gpio_ctrl = 5u;

    const uint32_t mask = (1u << (uint32_t)pin);

    // OE_SET
    *(volatile uint32_t*)SIO_GPIO_OE_SET = mask;

    // Start LOW
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

    // Ensure LOW at end
    *(volatile uint32_t*)SIO_GPIO_OUT_CLR = mask;
    return 0;
}

#ifdef __cplusplus
}
#endif
