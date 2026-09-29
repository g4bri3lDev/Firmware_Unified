/* What the imported Firmware_NRF52 panel drivers need from this target, in place of the nRF SDK. */
#ifndef EPD_PORT_H
#define EPD_PORT_H

#include <stdbool.h>
#include <stdint.h>

void epd_port_delay_ms(uint32_t ms);

/* Pins in tlsr_port.h numbering; 0xFF = not connected. */
struct epd_io_pins {
    uint8_t mosi, sclk, cs, dc, rst, busy, pwr;
};

void epd_io_configure(const struct epd_io_pins *pins);

/* Drive a panel power enable to its off level (0xFF: no-op). Called whenever a config is loaded,
 * so the supply is not left floating between boot and the first transfer. */
void epd_io_park_power(uint8_t pin);

/* True when the last EPD_WaitBusy() ran out its timeout rather than seeing the busy line clear. */
bool epd_io_busy_timed_out(void);

#endif /* EPD_PORT_H */
