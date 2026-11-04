/*
 * telltale_ina219.c - INA219 register maths (portable C99).
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#include "telltale_ina219.h"

uint16_t tt_ina219_config_word(uint32_t bus_32v, uint32_t pga, uint32_t badc,
                               uint32_t sadc, uint32_t mode)
{
    uint32_t w = 0u;
    w |= (bus_32v ? 1u : 0u) << 13;
    w |= (pga & 0x3u) << 11;
    w |= (badc & 0xFu) << 7;
    w |= (sadc & 0xFu) << 3;
    w |= (mode & 0x7u);
    return (uint16_t)w;
}

uint16_t tt_ina219_config_motion(void)
{
    return tt_ina219_config_word(0u, TT_INA219_PGA_80MV, TT_INA219_ADC_9BIT,
                                 TT_INA219_ADC_9BIT, TT_INA219_MODE_SHUNT_BUS_CONT);
}

uint16_t tt_ina219_config_idle(void)
{
    return tt_ina219_config_word(0u, TT_INA219_PGA_80MV, TT_INA219_ADC_12BIT,
                                 TT_INA219_ADC_AVG16, TT_INA219_MODE_SHUNT_BUS_CONT);
}

float tt_ina219_shunt_ma(int16_t raw, float shunt_ohm)
{
    if (!(shunt_ohm > 0.0f)) {
        return 0.0f;
    }
    /* raw * 10 uV / R[ohm] = raw * 0.01 mV / R = mA */
    return (float)raw * (TT_INA219_SHUNT_LSB_UV * 1.0e-3f) / shunt_ohm;
}

float tt_ina219_bus_mv(uint16_t raw)
{
    return (float)(raw >> 3) * TT_INA219_BUS_LSB_MV;
}
