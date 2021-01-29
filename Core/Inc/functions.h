/*
 * functions.h
 *
 *  Created on: Dec 12, 2020
 *      Author: root
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include "main.h"
#include "lcd.h"

void tempADC(float *value, u32 adc_data);
float tempADC2(u32 adc_data);
void strtoINT(uint32_t number, int lenght);
void ftoa(float n, char *res, int afterpoint);
int intToStr(int x, char *str, int d);
void reverse(char *str,int len);


#endif /* INC_FUNCTIONS_H_ */
