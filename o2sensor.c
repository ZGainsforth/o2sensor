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
#include "bmp280_i2c.h"
#include <string.h>
#include <math.h>

#define XISTOR_PIN 6 
#define ADCSEQUENCELENGTH 60 
#define VOLTSPERPERCENTO2 0.05  //This is the default for a SGX-40X -- tune the pot to make it perfect.

float mean(uint16_t* x, uint16_t lenx){
    uint16_t i;
    /* uint16_t meanval = 0; */
    float meanval=0;
    for(i=0; i<lenx; i++){
        meanval += (float)x[i];
    }
    meanval = meanval/lenx;
    return meanval;
}

int main() {
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversion_factor = 3.3f / (1 << 12);
    // Readout from ADC from O2 sensor.
    float result = 0;
    // Readout from ADC when O2 sensor disconnected by MOSFET.
    float zerolevel = 0;
    // Readouts are the aveage of adcsequence measurements
    uint16_t adcsequence[ADCSEQUENCELENGTH];
    // It's a ring buffer.
    uint16_t adcposition;
    // voltage is calculated from result and zerolevel.
    float voltage = 0;
    // O2Pct is calculated from voltage.
    float O2pct = 0;
    // tempcorrection is used to calculate the difference in the O2 reading as a function of temp.
    float tempcorrection = 0;
    // To hold strings to output to the oled.
    char display_str[100] = "\n";
    // For loops.
    int i;


    // Initialize 
    stdio_init_all();
    printf("O2 sensor booting.\n");
    
    // Init ADC to measure O2 sensor.
    printf("ADC measuring on GPIO26 (board pin 31).\n");
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);
    
    // Init GPIO for the MOSFET used to connect/disconnect the O2 sensor.
    printf("Setting GPIO6 (board pin 9) to control sensor connect transistor.\n");
    gpio_init(XISTOR_PIN);
    gpio_set_dir(XISTOR_PIN, GPIO_OUT);
    gpio_pull_down(XISTOR_PIN); // Default to transistor off = sensor disconnected.

    // Init the I2C which goes to the OLED and BME280.
    const uint32_t baudrate = 400000;
    printf("Initializing I2C on GPIOs SDA=%d, SCL=%d, %d baud.\n", PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, baudrate);
    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, baudrate);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Now initialize the OLED display.
    printf("Initializing OLED display across I2C\n");
    // run through the complete initialization process
    oled_init();
    oled_clear_display();

    // Get a reading off the opamp when the O2 sensor is disconnected.  This gives us our "zero" voltage reading.
    // Get the zero voltage offset.
    // Turn the transistor low which disconnects the O2 sesnsor. (It should be anyway at turn on.)
    gpio_put(XISTOR_PIN, 0);
    // Print friendly messages.
    printf("Reading zero voltage level on ADC.  Wait 10 secs...\n");
    sprintf(display_str, "Cal ADC zero level.");
    oled_render_string(0,0,display_str);
    sprintf(display_str, "Wait 10 seconds...");
    oled_render_string(0,1,display_str);
    // Read a whole buffer of ADC readings.
    for(i=0; i < ADCSEQUENCELENGTH; i++){
        adcsequence[i] = adc_read();
        printf("Zero level raw value %d of %d: 0x%03x\n", i+1, ADCSEQUENCELENGTH, adcsequence[i]);
        sleep_ms(100);
    }
    // Average the buffer to get the average zero level.
    zerolevel = mean(adcsequence, ADCSEQUENCELENGTH);
    printf("Mean zerolevel is %f.\n", zerolevel);
    // Turn the transistor high to reconnect the O2 sensor again.
    gpio_put(XISTOR_PIN, 1);

    // Refresh the display.
    oled_clear_display();

    // configure BME280
    bmp280_init();

    // retrieve fixed compensation params from the BME280 which it needs to make temp adjustments and the like.
    struct bmp280_calib_param params;
    bmp280_get_calib_params(&params);
    int32_t raw_temperature;
    int32_t raw_pressure;
    int32_t temperature;
    int32_t pressure;

    sleep_ms(250); // sleep so that data polling and register update don't collide

    uint8_t current_display_line = 0;

    // We take one reading from the O2 sensor and then fill the buffer with that value averaging has sensible values from the start.
    adcposition = 0;
    result = adc_read();
    // We start with the first reading filling the buffer.  As we get more readings the average will get better.
    for(adcposition=0; adcposition < ADCSEQUENCELENGTH; adcposition++){
        adcsequence[adcposition] = result;
    }

    while (1) {
        // Reset our display to the first line.
        printf("\n");
        current_display_line = 0;

        // read the temperature and pressure from the BME280.
        bmp280_read_raw(&raw_temperature, &raw_pressure); 
        // Temperature is an int recording hundredths of a degree.
        temperature = bmp280_convert_temp(raw_temperature, &params); 
        // Pressure is an int recording Pa.
        pressure = bmp280_convert_pressure(raw_pressure, raw_temperature, &params);

        // Print them out to the serial and OLED.
        printf("Pressure = %.3f kPa\n", pressure / 1000.f);
        printf("Temp. = %.2f C\n", temperature / 100.f);
        sprintf(display_str, "Pressure = %.3f kPa\n", pressure / 1000.f);
        oled_render_string(0,current_display_line++,display_str);
        sprintf(display_str, "Temp. = %.2f C\n", temperature / 100.f);
        oled_render_string(0,current_display_line++,display_str);

        // Read the O2 sensor.
        result = adc_read();
        // Add to the ring buffer.
        adcsequence[adcposition++] = result;
        if(adcposition >= ADCSEQUENCELENGTH){
            adcposition=0;
        }
        // Average the buffer to get the current ADC result.
        result = mean(adcsequence, ADCSEQUENCELENGTH);
        // Convert it to a voltage.
        voltage = (result-zerolevel) * conversion_factor;
        // Convert the voltage to an O2 percentage.
        O2pct = voltage / VOLTSPERPERCENTO2;
        // Calculate how many degrees from 20C. e.g. 23 -> 3.
        tempcorrection = ((temperature/100.f)-20);
        // The value changes 3% every 10 degrees so /10.
        tempcorrection /= 10;
        // tempcorrection is now a percentage change which needs to be added to the O2 reading.
        O2pct *= 1 + (tempcorrection/100);
        
        // Send the ADC measurement to the serial port.
        printf("ADC reading: %f, Voltage: %0.3f V, O2 = %0.2f%%\n", result, voltage, O2pct);

        // Send the ADC measurement to the OLED.
        sprintf(display_str, "Voltage: %0.3f V", voltage);
        oled_render_string(0,current_display_line++,display_str);
        sprintf(display_str, "O2: %0.2f%%", O2pct);
        oled_render_string(0,current_display_line++,display_str);
        // We read every second.
        sleep_ms(1000);

    }
}
