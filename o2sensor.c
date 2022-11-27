/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "oled_i2c.h"
#include "bme280_i2c.h"
#include <string.h>
#include <math.h>

#define XISTOR_PIN 6 
#define ADCSEQUENCELENGTH 60 
#define VOLTSPERPERCENTO2 0.05  //This is the default for a SGX-40X -- tune the pot to make it perfect.
#define BOOTUPSLEEP 0 // If 1 then we have several seconds of sleep time to get sensor stability when booting.  But 0 is better for debugging.
#define PWMFREQ 400000 // Frequency for the PWM audio output.

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

void pwm_set_freq_duty(uint slice, uint channel, uint32_t freq, uint16_t duty){
    // slice = the PWM slice.
    // channel = the PWM channel.
    // freq = the desired PWM frequency in Hz.
    // duty = The desired duty cycle in per mil (1/1000)

    // This is the default clock speed on pi pico, div/2 because we have two clocks per PWM cycle.
    uint32_t clock = 125000000/2; 

    // For now the minimum frequency is 1 kHz.  We can do lower by dividing the sys clock but don't need that now.
    if(freq < 1000){
        freq = 1000;
    }
    // Can't have frequency higher than the clock.
    if(freq > clock){
        freq = clock;
    }

    // Maximum duty is 1000 per mil.
    if(duty > 1000){
        duty = 1000;
    }

    // The wrap is the number of clock ticks in one period.  We round to the nearest integer.
    uint16_t wrap = (uint16_t)round(clock / freq);
    /* printf("Wrap value: %d", wrap); */

    uint16_t dutyticks = (uint16_t)round(wrap*duty/1000);
    /* printf("Duty value: %d", dutyticks); */

    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, channel,dutyticks);

    return;
}

/* #define STEPMICROSECS 100 */
/* #define WAITMICROSECS 78 */

#define STEPMICROSECS 25
#define WAITMICROSECS 3
#define DIODEDROP 0.212

/* // When outputting audio, it goes from 0-3.3 V which means the middle of the sine wave is 1.5V.  However, an audio alarm is rare. */
/* // To save power, we leave the PWM off when not using the audio */ 
/* void generate_ramp(uint slice, uint channel, uint32_t pwm_freq, uint32_t ms, uint8_t polarity){ */
/*     uint32_t num_steps = ms*1000/STEPMICROSECS; */
/*     uint32_t i = 0; // loop counter. */
/*     float A = 0; // Amplitude of the ramp. */
/* } */

void generate_tone(uint slice, uint channel, uint32_t pwm_freq, uint32_t audio_freq, uint32_t ms, uint8_t end_int_wavelength){
    // We assume the PWM frequency is >> than the audio_frequency.
    // It takes about 200 us for this function to run.
    uint32_t num_steps = ms*1000/STEPMICROSECS;
    /* printf("num_steps: %d\n", num_steps); */
    uint32_t i = 0; // loop counter.
    float t = 0; // time in seconds at current counter in loop.
    float A = 0; // Amplitude of the sine wave.
    float lastA = 0; // Amplitude of the sine wave at the last sample (checks for sample crossing).

    for(i=0; 1; i++){
        // convert to an absolute time.
        t = (float)(i * STEPMICROSECS) / 1000000;
        // Calculate the amplitude of the sine wave at the current time.
        A = sin(2*3.14159*audio_freq*t);
        // We want unity swing
        A /= 2;
        // Also reduce amplitude by one diode drop to accomodate the transistor in the amp on the output.
        /* A *= (1-DIODEDROP*2); */
        // And now shift it so it is at the "midpoint" between DIODEDROP and 1.
        A += 0.5;// + DIODEDROP;
        /* A = (sin(2*3.14159*audio_freq*t)*(1-DIODEDROP) + 1)/2; */
        pwm_set_freq_duty(slice, channel, PWMFREQ, A*1000);
        // Wait until the time slice is finished.
        sleep_us(WAITMICROSECS);

        // Loop end conditions.
        if(i>=num_steps){
            // Either wait to an end of a wavelength (sin phase = 2 pi = crossing the x axis in the positive direction.)
            if(end_int_wavelength==1){
                if((lastA < (0.5)) && (A >= (0.5))){
                    break;
                }
            // Or just end when the time is up.
            }else{
                break;
            }
        }
        lastA=A;
    }
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

    // Init PWM used for audio out when O2 goes out of range.
    // GPIO 9 is Pi Pico pin 12.
    gpio_set_function(7, GPIO_FUNC_PWM);
    // Get the slice and channel for the pwm on this gpio.
    uint slice = pwm_gpio_to_slice_num(7);
    uint channel = pwm_gpio_to_channel(7);
    // Set the frequency of the PWM to a value >> audio, and the duty cycle to 0 per mil.
    pwm_set_freq_duty(slice, channel, PWMFREQ, 0);
    pwm_set_enabled(slice, true);

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
        if (BOOTUPSLEEP==1){
            sleep_ms(100);
        }
    }
    // Average the buffer to get the average zero level.
    zerolevel = mean(adcsequence, ADCSEQUENCELENGTH);
    printf("Mean zerolevel is %f.\n", zerolevel);
    // Turn the transistor high to reconnect the O2 sensor again.
    gpio_put(XISTOR_PIN, 1);
    // sleep so the sensor has a chance to recover from open circuit.
    if (BOOTUPSLEEP==1){
        printf("Wait 4 seconds for O2 sensor stability after MOSFET switch closed.");
        sleep_ms(4000); 
    }

    // Refresh the display.
    oled_clear_display();

    // configure BME280
    bme280_init();

    // retrieve fixed compensation params from the BME280 which it needs to make temp adjustments and the like.
    struct bme280_calib_param params;
    bme280_get_calib_params(&params);
    int32_t raw_temperature;
    int32_t raw_pressure;
    int32_t raw_humidity;
    int32_t temperature;
    int32_t pressure;
    float humidity;

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
        bme280_read_raw(&raw_temperature, &raw_pressure, &raw_humidity); 
        /* printf("rawT, rawP, rawH = %d %d %d\n", raw_temperature, raw_pressure, raw_humidity); */
        // Temperature is an int recording hundredths of a degree.
        temperature = bme280_convert_temp(raw_temperature, &params); 
        // Pressure is an int recording Pa.
        pressure = bme280_convert_pressure(raw_pressure, raw_temperature, &params);
        // Humidity is an int recording RH% in fractional .
        humidity = bme280_convert_humidity(raw_humidity, raw_temperature, &params);

        // Print them out to the serial and OLED.
        printf("Pressure = %.3f kPa\n", pressure / 1000.f);
        printf("Temp. = %.2f \xB0""C\n", temperature / 100.f);
        printf("RH = %.2f%%\n", humidity);
        sprintf(display_str, "Pressure = %.3f kPa\n", pressure / 1000.f);
        oled_render_string(0,current_display_line++,display_str);
        sprintf(display_str, "Temp. = %.2f C\n", temperature / 100.f);
        oled_render_string(0,current_display_line++,display_str);
        sprintf(display_str, "RH = %.2f%%\n", humidity);
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
        // tempcorrection is now a change which needs to be removed to the O2 reading.
        // the chemical reactions in the cell increase as temperature increases.  So we need to subtract in order to maintain steady values.
        O2pct = O2pct*(1 - 0.03*tempcorrection);
        
        // Send the ADC measurement to the serial port.
        printf("ADC reading: %f, Voltage: %0.3f V, O2 = %0.2f%%\n", result, voltage, O2pct);

        // Send the ADC measurement to the OLED.
        sprintf(display_str, "O2: %0.2f%%", O2pct);
        oled_render_string(0,current_display_line++,display_str);


        // If the O2 sensor is out of bound then set off an alarm.
        if(O2pct < 20.0){
            for(i=0;i<100;i++){
                generate_tone(slice, channel, PWMFREQ, 50+(i*10), 5, 1);
            }
            pwm_set_freq_duty(slice, channel, PWMFREQ, 500);
            // We read every second.
            sleep_ms(500);
        }else{
            // We read every second.
            sleep_ms(1000);
            // Deactivate the audio out.
            pwm_set_freq_duty(slice, channel, PWMFREQ, 0);
        }

    }
}
