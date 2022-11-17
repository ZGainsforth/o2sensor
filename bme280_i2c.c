/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 2022-11-15 Upgraded BMP280->BME280, Zack Gainsforth.
 **/

#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "bme280_i2c.h"

 /* Example code to talk to a BMP280 temperature and pressure sensor

    NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
    GPIO (and therefore I2C) cannot be used at 5v.

    You will need to use a level shifter on the I2C lines if you want to run the
    board at 5v.

    Connections on Raspberry Pi Pico board, other boards may vary.

    GPIO PICO_DEFAULT_I2C_SDA_PIN (on Pico this is GP4 (pin 6)) -> SDA on BMP280
    board
    GPIO PICO_DEFAULT_I2C_SCK_PIN (on Pico this is GP5 (pin 7)) -> SCL on
    BMP280 board
    3.3v (pin 36) -> VCC on BMP280 board
    GND (pin 38)  -> GND on BMP280 board
 */

#ifdef i2c_default
void bme280_init() {
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];

    // 500ms sampling time, x16 filter
    const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;

    // send register number followed by its corresponding value
    buf[0] = REG_CONFIG;
    buf[1] = reg_config_val;
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);

    // osrs_h x1
    const uint8_t reg_ctrl_hum_val = 0x07;
    buf[0] = REG_CTRL_HUM;
    buf[1] = reg_ctrl_hum_val;
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
    
    // osrs_t x1, osrs_p x4, normal mode operation
    const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);
    buf[0] = REG_CTRL_MEAS;
    buf[1] = reg_ctrl_meas_val;
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);

}

void bme280_read_raw(int32_t* temp, int32_t* pressure, int32_t* humidity) {
    // BMP280 data registers are auto-incrementing and we have 3 temperature and
    // pressure registers each, so we start at 0xF7 and read 6 bytes to 0xFC
    // note: normal mode does not require further ctrl_meas and config register writes

    uint8_t buf[8];
    uint8_t reg = REG_PRESSURE_MSB;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, true);  // true to keep master control of bus
    i2c_read_blocking(i2c_default, ADDR, buf, 8, false);  // false - finished with bus

    // store the 20 bit read in a 32 bit signed integer for conversion
    *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    *humidity = (buf[6] << 8) | (buf[7]);
}

void bme280_reset() {
    // reset the device with the power-on-reset procedure
    uint8_t buf[2] = { REG_RESET, 0xB6 };
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
}

// intermediate function that calculates the fine resolution temperature
// used for both pressure and temperature conversions
int32_t bme280_convert(int32_t temp, struct bme280_calib_param* params) {
    // use the 32-bit fixed point compensation implementation given in the
    // datasheet
    
    int32_t var1, var2;
    var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) * ((int32_t)params->dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) * ((temp >> 4) - ((int32_t)params->dig_t1))) >> 12) * ((int32_t)params->dig_t3)) >> 14;
    return var1 + var2;
}

int32_t bme280_convert_temp(int32_t temp, struct bme280_calib_param* params) {
    // uses the BMP280 calibration parameters to compensate the temperature value read from its registers
    int32_t t_fine = bme280_convert(temp, params);
    return (t_fine * 5 + 128) >> 8;
}

int32_t bme280_convert_pressure(int32_t pressure, int32_t temp, struct bme280_calib_param* params) {
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

    int32_t t_fine = bme280_convert(temp, params);

    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)params->dig_p6);
    var2 += ((var1 * ((int32_t)params->dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)params->dig_p4) << 16);
    var1 = (((params->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)params->dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)params->dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)params->dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)params->dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + params->dig_p7) >> 4));
    return converted;
}

float bme280_convert_humidity(int32_t humidity, int32_t temp, struct bme280_calib_param* params) {
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

    int32_t t_fine = bme280_convert(temp, params);
    int32_t v_x1_u32r = (t_fine - (int32_t)76800);

    // Algorithm from the datasheet for BME280.
    /* printf("v_x = %d\n", v_x1_u32r); */
    /* printf("hmm = %d\n", humidity<<14); */
    v_x1_u32r = (((((humidity << 14) - (((int32_t)params->dig_h4) << 20) - (((int32_t)params->dig_h5) *  v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)params->dig_h6)) >> 10) * (((v_x1_u32r * ((int32_t)params->dig_h3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)params->dig_h2) + 8192) >> 14));
    /* printf("v_x = %d\n", v_x1_u32r); */
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)params->dig_h1)) >> 4));
    /* printf("v_x = %d\n", v_x1_u32r); */
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    /* printf("v_x = %d\n", v_x1_u32r); */
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    /* printf("v_x = %d\n", v_x1_u32r); */
    /* return (uint32_t)(v_x1_u32r>>12); */
    v_x1_u32r = (uint32_t)(v_x1_u32r>>12);

    // That gets us to Q22.10 format with 22 integer bits and 10 fractional bits.
    // We can convert to a float by dividing by 1024 to get RH% like RH=20.323%.
    return ((float)v_x1_u32r)/1024.f;
 }

void bme280_get_calib_params(struct bme280_calib_param* params) {
    // raw temp, pressure and humidity values need to be calibrated according to
    // parameters generated during the manufacturing of the sensor
    // there are 3 temperature params, 9 pressure params, each with a LSB
    // and MSB register, total 24 registers.
    // The humidity regs are smaller so we will reuse the buffer for that later.

    uint8_t buf[NUM_CALIB_PARAMS] = { 0 };
    uint8_t reg = REG_DIG_T1_LSB;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, true);  // true to keep master control of bus
    // read in one go as register addresses auto-increment
    i2c_read_blocking(i2c_default, ADDR, buf, NUM_CALIB_PARAMS, false);  // false, we're done reading

    // store these in a struct for later use
    params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];

    reg = REG_DIG_H1;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, true);  // true to keep master control of bus
    // read in one go as register addresses auto-increment
    i2c_read_blocking(i2c_default, ADDR, buf, 1, false);  // false, we're done reading

    params->dig_h1 = (uint8_t)buf[0];

    reg = REG_DIG_H2;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, true);  // true to keep master control of bus
    // read in one go as register addresses auto-increment
    i2c_read_blocking(i2c_default, ADDR, buf, 6, false);  // false, we're done reading

    params->dig_h2 = (int16_t)(buf[1]<<8) | buf[0];
    params->dig_h3 = (uint8_t)buf[2];
    params->dig_h4 = (int16_t)((buf[3] << 4) | (buf[4] & 0x0F));
    params->dig_h5 = (int16_t)((buf[4] >> 4) | (buf[5]<<4));
    params->dig_h6 = (int8_t)buf[6];
}

#endif

/* int main() { */
/*     stdio_init_all(); */

/* #if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN) */
/*     #warning i2c / bme280_i2c example requires a board with I2C pins */
/*         puts("Default I2C pins were not defined"); */
/* #else */
/*     // useful information for picotool */
/*     bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C)); */
/*     bi_decl(bi_program_description("BMP280 I2C example for the Raspberry Pi Pico")); */

/*     printf("Hello, BMP280! Reading temperaure and pressure values from sensor...\n"); */

/*     // I2C is "open drain", pull ups to keep signal high when no data is being sent */
/*     i2c_init(i2c_default, 100 * 1000); */
/*     gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C); */
/*     gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C); */
/*     gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN); */
/*     gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN); */

/*     // configure BMP280 */
/*     bme280_init(); */

/*     // retrieve fixed compensation params */
/*     struct bme280_calib_param params; */
/*     bme280_get_calib_params(&params); */

/*     int32_t raw_temperature; */
/*     int32_t raw_pressure; */

/*     sleep_ms(250); // sleep so that data polling and register update don't collide */
/*     while (1) { */
/*         bme280_read_raw(&raw_temperature, &raw_pressure); */
/*         int32_t temperature = bme280_convert_temp(raw_temperature, &params); */
/*         int32_t pressure = bme280_convert_pressure(raw_pressure, raw_temperature, &params); */
/*         printf("Pressure = %.3f kPa\n", pressure / 1000.f); */
/*         printf("Temp. = %.2f C\n", temperature / 100.f); */
/*         // poll every 500ms */
/*         sleep_ms(500); */
/*     } */

/* #endif */
/*     return 0; */
/* } */
