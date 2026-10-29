#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "sync_period.pio.h"
#include "csync_sep.pio.h"
#include "board.h"
#include "source.h"
#include "sync.h"

// pio0 (base 0): HSYNC/VSYNC are GPIO 26/27, within 0-31.
static PIO pio = pio0;
static uint sm_h, sm_v, prog_off;

// Composite-sync sources: a long sync pulse is vertical. Line syncs are ~4 us
// and the Electron's vertical pulse ~160 us, so 32 us (half a line) splits
// them with a wide margin either way.
#define CSYNC_VSYNC_MIN_US 32u

// The separator runs on pio1, beside DVI: libdvi claims SMs 0-2 and 2
// instructions there (the pixel clock is a PWM slice), so SM 3 and the rest of
// instruction memory are free. pio0 (capture + sync_period, 24 instructions)
// and pio2 (IR decoder, 30) are too full. Call after video_init().
// PIN_VSYNC becomes a pio1 output, inside pio1's GPIO window on both boards
// (base 0, or 16 on the PiZero). Input readers (the sync_period SM on pio0,
// capture's gpio_get) still see it: GPIO input does not depend on FUNCSEL.
static void csync_sep_start(void)
{
    const PIO p = pio1;
    const uint off = pio_add_program(p, &csync_sep_program);
    const uint sm = pio_claim_unused_sm(p, true);
    pio_sm_config c = csync_sep_program_get_default_config(off);
    sm_config_set_in_pins(&c, PIN_HSYNC);       // `wait pin 0`
    sm_config_set_jmp_pin(&c, PIN_HSYNC);
    sm_config_set_set_pins(&c, PIN_VSYNC, 1);
    pio_gpio_init(p, PIN_VSYNC);
    pio_sm_set_consecutive_pindirs(p, sm, PIN_VSYNC, 1, true);
    pio_sm_init(p, sm, off, &c);
    pio_sm_put(p, sm, clock_get_hz(clk_sys) / 1000000u * CSYNC_VSYNC_MIN_US / 2u);
    pio_sm_set_enabled(p, sm, true);
}

static void sm_setup(uint sm, uint pin)
{
    pio_gpio_init(pio, pin);
    gpio_set_input_hysteresis_enabled(pin, true);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false); // input
    pio_sm_config c = sync_period_program_get_default_config(prog_off);
    sm_config_set_in_pins(&c, pin); // `wait pin 0`
    sm_config_set_jmp_pin(&c, pin); // counting-loop pin test
    pio_sm_init(pio, sm, prog_off, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void sync_init(void)
{
    prog_off = pio_add_program(pio, &sync_period_program);
    sm_h = pio_claim_unused_sm(pio, true);
    sm_v = pio_claim_unused_sm(pio, true);
    sm_setup(sm_h, PIN_HSYNC);
    sm_setup(sm_v, PIN_VSYNC);

    if (source_active()->csync) {
        // Active-low CSYNC, inverted in the pad so capture.pio's rising edge
        // is the start of each sync pulse and sync_period measures the pulse
        // itself. After sm_setup: gpio_set_function rewrites CTRL, which
        // would clear the override.
        gpio_set_inover(PIN_HSYNC, GPIO_OVERRIDE_INVERT);
        csync_sep_start();
    }
}

// Only for a failed measure (stuck at `wait` with no edges).
static void sm_restart(uint sm)
{
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(prog_off));
    pio_sm_set_enabled(pio, sm, true);
}

static bool rx_get(uint sm, uint32_t *v)
{
    absolute_time_t deadline = make_timeout_time_ms(250);
    while (pio_sm_is_rx_fifo_empty(pio, sm))
        if (time_reached(deadline))
            return false;
    *v = pio_sm_get(pio, sm);
    return true;
}

static uint32_t measure(uint sm, uint32_t preload, uint32_t *pulse)
{
    pio_sm_put_blocking(pio, sm, preload);
    uint32_t pr, gr;
    if (!rx_get(sm, &pr) || !rx_get(sm, &gr))
    {
        sm_restart(sm);
        if (pulse) *pulse = 0;
        return 0;
    }
    if (pr <= 1 || gr <= 1) // countdown ran out -> no edge
    {
        if (pulse) *pulse = 0;
        return 0;
    }
    uint32_t high = (preload - pr) * 2;
    uint32_t low  = (preload - gr) * 2;
    if (pulse) *pulse = high;
    return high + low;
}

uint32_t sync_hsync_period(void) { return measure(sm_h, 100000, NULL); }
uint32_t sync_vsync_period(void) { return measure(sm_v, 6000000, NULL); }
uint32_t sync_hsync(uint32_t *pulse) { return measure(sm_h, 100000, pulse); }
