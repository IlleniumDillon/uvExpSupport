/*
 * fpn.h
 *
 *  Created on: Sep 29, 2024
 *      Author: 14769
 */

#ifndef PROTOCOL_FPN_H_
#define PROTOCOL_FPN_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef union
{
	double memory;
	struct
	{
		uint64_t mantissa	:	52;
		uint64_t exponent	:	11;
		uint64_t sign		:	1;
	};
}drift64;

typedef union
{
	float memory;
	struct
	{
		uint32_t mantissa	:	23;
		uint32_t exponent	:	8;
		uint32_t sign		:	1;
	};
}drift32;

typedef union
{
	uint16_t memory;
	struct
	{
		uint16_t mantissa	:	10;
		uint16_t exponent	:	5;
		uint16_t sign		:	1;
	};
}drift16;

typedef union
{
	uint8_t memory;
	struct
	{
		uint8_t mantissa	:	4;
		uint8_t exponent	:	3;
		uint8_t sign		:	1;
	};
}drift8;

typedef union
{
	uint8_t memory;
	struct
	{
		uint8_t mantissa	:	5;
		uint8_t exponent	:	3;
	};
}udrift8;

#define DRIFT64_NAN		0x7FF8000000000000
#define DRIFT32_NAN		0x7FC00000
#define DRIFT16_NAN		0x7E00
#define DRIFT8_NAN		0x78

#define DRIFT64_INF		0x7FF0000000000000
#define DRIFT32_INF		0x7F800000
#define DRIFT16_INF		0x7C00
#define DRIFT8_INF		0x70

drift64 drift64_from_double(double value);
double drift64_to_double(drift64 value);

drift32 drift32_from_float(float value);
float drift32_to_float(drift32 value);

drift16 drift16_from_float(float value);
float drift16_to_float(drift16 value);

drift8 drift8_from_float(float value);
float drift8_to_float(drift8 value);

udrift8 udrift8_from_float(float value);
float udrift8_to_float(udrift8 value);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_FPN_H_ */
