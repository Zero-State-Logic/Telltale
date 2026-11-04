/*
 * telltale_ina219.h - INA219 register maths (portable C99, no I/O).
 *
 * The firmware reads the shunt-voltage register directly over I2C instead of
 * using the calibration/current registers, so the conversion is one multiply
 * and the ADC settings are fully under our control:
 *   - motion: 9-bit shunt + 9-bit bus conversions (~84 us each), read at 1 kHz;
 *   - idle:   12-bit shunt with 16-sample averaging (8.5 ms), read at 100 Hz.
 *
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#ifndef TELLTALE_INA219_H
#define TELLTALE_INA219_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TT_INA219_ADDR          0x40u
#define TT_INA219_REG_CONFIG    0x00u
#define TT_INA219_REG_SHUNT     0x01u
#define TT_INA219_REG_BUS       0x02u

#define TT_INA219_PGA_40MV      0u   /* /1 */
#define TT_INA219_PGA_80MV      1u   /* /2 : 0.8 A with a 0.1 ohm shunt */
#define TT_INA219_PGA_160MV     2u   /* /4 */
#define TT_INA219_PGA_320MV     3u   /* /8 (power-on default) */

#define TT_INA219_ADC_9BIT      0x0u /* 84 us */
#define TT_INA219_ADC_10BIT     0x1u
#define TT_INA219_ADC_11BIT     0x2u
#define TT_INA219_ADC_12BIT     0x3u /* 532 us */
#define TT_INA219_ADC_AVG16     0xCu /* 12-bit, 16 samples, 8.51 ms */

#define TT_INA219_MODE_SHUNT_BUS_CONT 0x7u

#define TT_INA219_SHUNT_LSB_UV  10.0f  /* shunt register LSB, microvolts */
#define TT_INA219_BUS_LSB_MV    4.0f

uint16_t tt_ina219_config_word(uint32_t bus_32v, uint32_t pga, uint32_t badc,
                               uint32_t sadc, uint32_t mode);
/* Fast config used during motion and the averaged config used while idle. */
uint16_t tt_ina219_config_motion(void);
uint16_t tt_ina219_config_idle(void);
/* Shunt register (two's complement, 10 uV LSB) to milliamps. */
float tt_ina219_shunt_ma(int16_t raw, float shunt_ohm);
/* Bus register to millivolts (bits 15..3, 4 mV LSB). */
float tt_ina219_bus_mv(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* TELLTALE_INA219_H */
