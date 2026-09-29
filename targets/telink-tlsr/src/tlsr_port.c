/* SDK-side implementation of tlsr_port.h: timer, flash, AES engine, noise, GPIO, breadcrumbs. */
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"

#include "tlsr_port.h"

uint32_t tlsr_port_ticks(void)
{
    return clock_time();
}

void tlsr_port_delay_us(uint32_t us)
{
    sleep_us(us);
}

void tlsr_port_flash_read(uint32_t addr, uint32_t len, uint8_t *buf)
{
    flash_read_page(addr, len, buf);
}

void tlsr_port_flash_program(uint32_t addr, uint32_t len, const uint8_t *buf)
{
    flash_write_page(addr, len, (u8 *)buf);
}

void tlsr_port_flash_erase_4k(uint32_t addr)
{
    flash_erase_sector(addr);
}

void tlsr_port_aes_block(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
    u8 r = irq_disable();
    aes_encrypt((u8 *)key, (u8 *)in, out);
    irq_restore(r);
}

uint32_t tlsr_port_noise32(void)
{
    return rand();
}

void tlsr_port_reboot(void)
{
    start_reboot();
}

static GPIO_PinTypeDef pin_of(uint8_t pin)
{
    return (GPIO_PinTypeDef)(((u32)(pin >> 3) << 8) | BIT(pin & 7u));
}

static bool pin_valid(uint8_t pin)
{
    return pin != TLSR_PORT_PIN_NONE && (pin >> 3) <= 4u;
}

void tlsr_port_gpio_output(uint8_t pin, bool level)
{
    if (!pin_valid(pin)) return;
    gpio_set_func(pin_of(pin), AS_GPIO);
    gpio_write(pin_of(pin), level);
    gpio_set_input_en(pin_of(pin), 0);
    gpio_set_output_en(pin_of(pin), 1);
}

void tlsr_port_gpio_input(uint8_t pin, uint8_t pull)
{
    if (!pin_valid(pin)) return;
    gpio_set_func(pin_of(pin), AS_GPIO);
    gpio_set_output_en(pin_of(pin), 0);
    gpio_set_input_en(pin_of(pin), 1);
    gpio_setup_up_down_resistor(pin_of(pin), pull == TLSR_PORT_PULL_UP   ? PM_PIN_PULLUP_10K
                                           : pull == TLSR_PORT_PULL_DOWN ? PM_PIN_PULLDOWN_100K
                                                                         : PM_PIN_UP_DOWN_FLOAT);
}

void tlsr_port_gpio_write(uint8_t pin, bool level)
{
    if (!pin_valid(pin)) return;
    gpio_write(pin_of(pin), level);
}

bool tlsr_port_gpio_read(uint8_t pin)
{
    return pin_valid(pin) && gpio_read(pin_of(pin)) != 0;
}

void tlsr_port_gpio_release(uint8_t pin)
{
    if (!pin_valid(pin)) return;
    gpio_set_output_en(pin_of(pin), 0);
    gpio_set_input_en(pin_of(pin), 0);
    gpio_setup_up_down_resistor(pin_of(pin), PM_PIN_UP_DOWN_FLOAT);
}

void tlsr_port_service_stack(void)
{
    wd_clear();                        /* long waits (panel refresh) must not trip the watchdog */
    blt_sdk_main_loop();
}

/* DEEP_ANA_REG1/2 (0x3b/0x3c): reset only by a power cycle, so they survive the watchdog. */
#define CRUMB_STEP_REG   0x3b
#define CRUMB_RESET_REG  0x3c

void tlsr_port_crumb(uint8_t step)
{
    analog_write(CRUMB_STEP_REG, step);
}

uint8_t tlsr_port_crumb_boot(uint8_t *resets)
{
    uint8_t last = analog_read(CRUMB_STEP_REG);
    uint8_t count = analog_read(CRUMB_RESET_REG);

    if (last != 0u && count != 0xFFu) {
        count++;
        analog_write(CRUMB_RESET_REG, count);
    }
    analog_write(CRUMB_STEP_REG, 0u);
    *resets = count;
    return last;
}
