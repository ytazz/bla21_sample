#pragma once

#include <stdint.h>

/*
 * simple implementation of half-precision float
 */

struct Float16{
	static float Decode(uint16_t val){
		if(val == 0)
			return 0.0f;

		int sign     = ((val >> 15) ? -1 : 1);
		int exponent = ((val >> 10) & 0x1f);
		int fraction = (val & 0x3ff);

		return (float)(sign*pow(2.0, exponent - 15)*(1.0f + fraction/1024.0f));
	}
	static uint16_t Encode(float val){
		if(val == 0.0f)
			return 0;

		int sign;
		if(val < 0.0f){
			sign = 1;
			val = -val;
		}
		else{
			sign = 0;
		}
		int exponent = (int)floor(log(val)/log(2.0));
		int fraction = (int)(1024.0f*(val/pow(2.0, exponent) - 1.0f));

		return (sign << 15) | (((exponent + 15) & 0x1f) << 10) | (fraction & 0x3ff);
	}
				
};
