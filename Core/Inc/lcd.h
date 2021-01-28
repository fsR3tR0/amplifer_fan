/*
 * lcd.h
 *
 *  Created on: Dec 10, 2020
 *      Author: root
 */

#ifndef LCD_H_
#define LCD_H_

#include "main.h"

#define LCD_E 2
#define LCD_curzor 1
#define LCD_blink 0
#define CLEAR 0x01
#define HOME 0x02

void LCD_test(void);
void LCD_cmd(u8 cmd);
void LCD_enable(void);
void LCD_data(u8 data);
void LCD_init(bool curzor, bool blink);
void LCD_string(char *str);
void LCD_goto(u8 row, u8 col);

#endif /* LCD_H_ */
