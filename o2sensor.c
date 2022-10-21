/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "oled_i2c.h"
/* #include "raspberry26x32.h" */

int main() {
    stdio_init_all();
    printf("ADC Example, measuring GPIO26\n");

    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);

    // I2C is "open drain", pull ups to keep signal high when no data is being
    // sent
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // run through the complete initialization process
    oled_init();
    oled_clear_display();

    while (1) {
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        const float conversion_factor = 3.3f / (1 << 12);
        uint16_t result = adc_read();
        char display_str[100] = "\n";

        // Send the ADC measurement to the serial port.
        printf("My raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);

        // Send the ADC measurement to the OLED.
        sprintf(display_str, "ADC reading: 0x%03x.", result);
        oled_render_string(0,0,display_str);
        sprintf(display_str, "Voltage: %f V", result*conversion_factor);
        oled_render_string(0,1,display_str);
        sleep_ms(1000);
    }
}
