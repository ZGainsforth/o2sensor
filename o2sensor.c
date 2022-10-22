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

#define XISTOR_PIN 6 
#define ZEROSEQUENCELENGTH 100
#define VOLTSPERPERCENTO2 0.05  //This is the default for a SGX-40X -- tune the pot to make it perfect.

uint16_t mean(uint16_t* x, uint16_t lenx){
    uint16_t i;
    uint16_t meanval = 0;
    for(i=0; i<lenx; i++){
        meanval += x[i];
    }
    meanval = meanval/lenx;
    return meanval;
}

int main() {
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversion_factor = 3.3f / (1 << 12);
    uint16_t result = 0;
    uint16_t zerolevel = 0;
    uint16_t zerolevelsequence[ZEROSEQUENCELENGTH];
    char display_str[100] = "\n";
    int i;
    float voltage = 0;
    float O2pct = 0;


    stdio_init_all();
    printf("O2 sensor booting.\n");
    printf("ADC measuring on GPIO26 (board pin 31).\n");
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);
    
    printf("Setting GPIO6 (board pin 9) to control sensor connect transistor.\n");
    gpio_init(XISTOR_PIN);
    gpio_set_dir(XISTOR_PIN, GPIO_OUT);
    gpio_pull_down(XISTOR_PIN); // Default to transistor off = sensor disconnected.

    const uint32_t baudrate = 400000;
    printf("Initializing I2C on GPIOs SDA=%d, SCL=%d, %d baud.\n", PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, baudrate);
    // I2C is "open drain", pull ups to keep signal high when no data is being
    // sent
    i2c_init(i2c_default, baudrate);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    printf("Initializing OLED display across I2C\n");
    // run through the complete initialization process
    oled_init();
    oled_clear_display();

    // Get the zero voltage offset.
    printf("Reading zero voltage level on ADC.  Wait 10 secs...\n");
    sprintf(display_str, "Cal ADC zero level.");
    oled_render_string(0,0,display_str);
    sprintf(display_str, "Wait 10 seconds...");
    oled_render_string(0,1,display_str);
    // Turn the transistor low. (It should be anyway at turn on.)
    gpio_put(XISTOR_PIN, 0);
    // Read 10 values of zero reading.
    for(i=0; i < ZEROSEQUENCELENGTH; i++){
        zerolevelsequence[i] = adc_read();
        printf("Zero level raw value %d of %d: 0x%03x\n", i+1, ZEROSEQUENCELENGTH, zerolevelsequence[i]);
        sleep_ms(100);
    }
    zerolevel = mean(zerolevelsequence, ZEROSEQUENCELENGTH);
    printf("Mean zerolevel is 0x%03x.\n", zerolevel);
    // Turn the transistor high. (It should be anyway at turn on.)
    gpio_put(XISTOR_PIN, 1);

    oled_clear_display();

    while (1) {

        // Read the ADC.
        result = adc_read();
        voltage = (result-zerolevel) * conversion_factor;
        O2pct = voltage / VOLTSPERPERCENTO2;
        // Send the ADC measurement to the serial port.
        printf("ADC reading: 0x%03x, Voltage: %0.3f V, O2 = %0.2f%%\n", result, voltage, O2pct);

        // Send the ADC measurement to the OLED.
        sprintf(display_str, "ADC reading: 0x%03x", result);
        oled_render_string(0,0,display_str);
        sprintf(display_str, "Voltage: %0.3f V", voltage);
        oled_render_string(0,1,display_str);
        sprintf(display_str, "O2: %0.2f%%", O2pct);
        oled_render_string(0,2,display_str);

        // We read every second.
        sleep_ms(1000);

    }
}
