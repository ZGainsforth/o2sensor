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

    // initialize render area for entire frame (128 pixels by 4 pages)
    struct render_area frame_area = {start_col: 0, end_col : OLED_WIDTH - 1, start_page : 0, end_page : OLED_NUM_PAGES - 1};
    calc_render_area_buflen(&frame_area);

    // zero the entire display
    uint8_t buf[OLED_BUF_LEN];
    fill(buf, 0x00);
    render(buf, &frame_area);

    /* struct render_area area = {start_col: 0, end_col : IMG_WIDTH - 1, start_page : 0, end_page : OLED_NUM_PAGES - 1}; */
    /* calc_render_area_buflen(&area); */
    /* render(raspberry26x32, &area); */

    // The render areas are inclusive, and zero based.  i.e. 0-2 is three pixels.
    struct render_area letter_area = {start_col: 4, end_col : 6, start_page : 1, end_page : 2};
    calc_render_area_buflen(&letter_area);
    static uint8_t lettera[] = { 0xAA, 0xff, 0xff, 0xff, 0xff, 0xAA};
    /* static uint8_t lettera[] = { 0x00, 0xFF, 0xFF, 0xFF, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; */
    /* render(lettera, &letter_area); */

    while (1) {
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        const float conversion_factor = 3.3f / (1 << 12);
        uint16_t result = adc_read();
        printf("My raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
        render(buf, &frame_area);
        sleep_ms(1000);
        /* oled_send_cmd(0xA5); // ignore RAM, all pixels on */
        render(lettera, &letter_area);
        oled_render_letter(0,0,'0');
        oled_render_letter(1,0,'1');
        oled_render_letter(2,0,'2');
        oled_render_letter(3,0,'3');
        oled_render_letter(4,0,'4');
        oled_render_letter(5,0,'5');
        oled_render_letter(6,0,'6');
        oled_render_letter(7,0,'7');
        oled_render_letter(8,0,'8');
        oled_render_letter(9,0,'9');
        sleep_ms(500);
        /* render(raspberry26x32, &area); */
        /* printf("Rendering %d bytes.\n", area.buflen); */
        /* oled_send_cmd(0xA4); // go back to following RAM */
    }
}
