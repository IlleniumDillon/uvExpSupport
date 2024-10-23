/*
 * fpn.c
 *
 *  Created on: Sep 29, 2024
 *      Author: 14769
 */

#include "fpn.h"

drift64 drift64_from_double(double value)
{
    drift64 result;
    result.memory = value;
    return result;
}
double drift64_to_double(drift64 value)
{
    return value.memory;
}

drift32 drift32_from_float(float value)
{
    drift32 result;
    result.memory = value;
    return result;
}
float drift32_to_float(drift32 value)
{
    return value.memory;
}

drift16 drift16_from_float(float value)
{
    drift32 temp;
    temp.memory = value;
    drift16 result;
    result.sign = temp.sign;
    int8_t exponent = temp.exponent - 127;
    // if exponent is less than -15, then the value is too small to be represented in drift16
    if (exponent < -15)
    {
        result.exponent = 0x0;
        result.mantissa = 0x0;
        return result;
    }
    // if exponent is greater than 15, then the value is too large to be represented in drift16
    if (exponent > 15)
    {
        result.exponent = 0x1F;
        result.mantissa = 0x0;
        return result;
    }
    result.exponent = exponent + 15;
    result.mantissa = temp.mantissa >> 13;
    return result;
}
float drift16_to_float(drift16 value)
{
    drift32 result;
    result.sign = value.sign;
    int8_t exponent = value.exponent - 15;
    result.exponent = exponent + 127;
    result.mantissa = value.mantissa << 13;
    return result.memory;
}

drift8 drift8_from_float(float value)
{
    drift32 temp;
    temp.memory = value;
    drift8 result;
    result.sign = temp.sign;
    int8_t exponent = temp.exponent - 127;
    // if exponent is less than -3, then the value is too small to be represented in drift8
    if (exponent < -3)
    {
        result.exponent = 0x0;
        result.mantissa = 0x0;
        return result;
    }
    // if exponent is greater than 4, then the value is too large to be represented in drift8
    if (exponent > 3)
    {
        result.exponent = 0x7;
        result.mantissa = 0x0;
        return result;
    }
    result.exponent = exponent + 3;
    result.mantissa = temp.mantissa >> 19;
    return result;
}
float drift8_to_float(drift8 value)
{
    drift32 result;
    result.sign = value.sign;
    int8_t exponent = value.exponent - 3;
    result.exponent = exponent + 127;
    result.mantissa = value.mantissa << 19;
    return result.memory;
}

udrift8 udrift8_from_float(float value)
{
    drift32 temp;
    temp.memory = value;
    udrift8 result;
    if (temp.sign)
    {
        result.exponent = 0x0;
        result.mantissa = 0x0;
        return result;
    }
    int8_t exponent = temp.exponent - 127;
    // if exponent is less than -3, then the value is too small to be represented in drift8
    if (exponent < -3)
    {
        result.exponent = 0x0;
        result.mantissa = 0x0;
        return result;
    }
    // if exponent is greater than 4, then the value is too large to be represented in drift8
    if (exponent > 4)
    {
        result.exponent = 0x7;
        result.mantissa = 0x0;
        return result;
    }
    result.exponent = exponent + 3;
    result.mantissa = temp.mantissa >> 18;
    return result;
}

float udrift8_to_float(udrift8 value)
{
    drift32 result;
    result.sign = 0;
    int8_t exponent = value.exponent - 3;
    result.exponent = exponent + 127;
    result.mantissa = value.mantissa << 18;
    return result.memory;
}
