/*
 * functions.c
 *
 *  Created on: Dec 12, 2020
 *      Author: root
 */
#include "functions.h"

void tempADC(float *value, u32 adc_data){
	float u25 = 1.43;
	float AVG_slope = 0.0043;
	float Usense = (3.3/4096)*adc_data;
	//float Us = Usense*adc_data;

	float T = ((u25-Usense)/AVG_slope+25.0);

	*value = T;
}

float tempADC2(u32 adc_data){

	return ((((3.3/4096)*adc_data)-1.43)/(4.3/1000.0)+25);
}

void strtoINT(uint32_t number, int lenght){
	uch string[lenght];
	sprintf(string,"%d",number);

	LCD_string(string);
}

void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int x, char *str, int d) {
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';

    return i;
}

void ftoa(float n, char* res, int afterpoint) {
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
