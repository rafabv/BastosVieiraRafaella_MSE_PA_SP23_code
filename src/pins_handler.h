#ifndef PINS_HANDLER_H
#define PINS_HANDLER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>


/*PINS BOOT AND RESET*/
#define BOOT_NODE DT_ALIAS(controlpin0)
#define RESET_NODE DT_ALIAS(controlpin1)

static const struct gpio_dt_spec bootpin = GPIO_DT_SPEC_GET(BOOT_NODE, gpios);
static const struct gpio_dt_spec resetpin = GPIO_DT_SPEC_GET(RESET_NODE, gpios);


void pins_init();
void boot_activation(bool onoff);
void reset_stm();
#endif