/*
 * lcd.c
 *
 *  Created on: Dec 10, 2020
 *      Author: root
 */
//I use a LCD display with 2 row and 16 cool, with Hitachi basic
//i use it in 4 data_bit mode
#include "lcd.h"

void LCD_test(){
	LCD_goto(0,0);
	LCD_string("#+++Elso-sor+++#");
	LCD_goto(1,0);
	LCD_string("#+Masodik--sor+#");
	HAL_Delay(1000);
	LCD_cmd(CLEAR);
}

void LCD_string(char *str){
	while(*str){
		LCD_data(*str);
		str++;
	}
}

void LCD_goto(u8 row, u8 col){
	switch(row){
		case 0:
			LCD_cmd(0x80);
			for(u8 i = col; i>0; i--){
				LCD_cmd(0x14);
			}
		 break;
		case 1:
			LCD_cmd(0xC0);
			for(u8 i = col; i>0; i--){
				LCD_cmd(0x14);
			}
		 break;
	}
}

void LCD_enable(){
	//need a square  signal _| |_ for LCD enable Port
	//we need a 0-1-0 signalflow, transition
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, RESET);	//make _
	HAL_Delay(5);		//wait
	HAL_GPIO_WritePin(LCD_E_GPIO_Port,LCD_E_Pin, SET);	//_|
	HAL_Delay(5);		//wait
	HAL_GPIO_WritePin(LCD_E_GPIO_Port,LCD_E_Pin, RESET);	//	|_
	HAL_Delay(5);
}

void LCD_cmd(u8 cmd){
	//we need reset RS port, why we send commands for the LCD display
		//the R/W pin is already on GNDm because we only write the display
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin, RESET);
	//we write the data with 2 cycle, first the upper Bits
		//we use PORT MASK just in case
	HAL_GPIO_WritePin(LCD_DATA_7_GPIO_Port,LCD_DATA_7_Pin,(cmd>>7) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_6_GPIO_Port,LCD_DATA_6_Pin,(cmd>>6) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_5_GPIO_Port,LCD_DATA_5_Pin,(cmd>>5) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_4_GPIO_Port,LCD_DATA_4_Pin,(cmd>>4) & 0x01);

	//we write this for LCD with LCD_enable, we send the datas
	LCD_enable();

	//now we write the four lower Bit
	HAL_GPIO_WritePin(LCD_DATA_7_GPIO_Port,LCD_DATA_7_Pin,(cmd>>3) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_6_GPIO_Port,LCD_DATA_6_Pin,(cmd>>2) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_5_GPIO_Port,LCD_DATA_5_Pin,(cmd>>1) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_4_GPIO_Port,LCD_DATA_4_Pin,(cmd>>0) & 0x01);		//here truly we don`t need the MASK,just in case

	//again write for LCD
	LCD_enable();
}

void LCD_data(u8 data){
	//we set the RS Pin, because we send now data
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,SET);
	//send data again with 2 cycles
	HAL_GPIO_WritePin(LCD_DATA_7_GPIO_Port,LCD_DATA_7_Pin, (data>>7) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_6_GPIO_Port,LCD_DATA_6_Pin, (data>>6) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_5_GPIO_Port,LCD_DATA_5_Pin, (data>>5) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_4_GPIO_Port,LCD_DATA_4_Pin, (data>>4) & 0x01);
	//we send the data for display
	LCD_enable();

	//now we send the low 4 bits
	HAL_GPIO_WritePin(LCD_DATA_7_GPIO_Port,LCD_DATA_7_Pin, (data>>3) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_6_GPIO_Port,LCD_DATA_6_Pin, (data>>2) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_5_GPIO_Port,LCD_DATA_5_Pin, (data>>1) & 0x01);
	HAL_GPIO_WritePin(LCD_DATA_4_GPIO_Port,LCD_DATA_4_Pin, (data>>0) & 0x01);

	LCD_enable();
}

void LCD_init(bool curzor, bool blink){
	//we before init
	HAL_Delay(15);
	//we use it in 4bit mode with 2 row - 5*8 pixel
	LCD_cmd(0x20);

	//just in case we send for Enable PORT 3 cycle
	LCD_enable();
	LCD_enable();
	LCD_enable();

	//this hexacodes is from datasheet (LCD 1602, with hitachi base)
	LCD_cmd(0x28);		//set the mode 3 time
	LCD_cmd(0x28);		//we don`t know before the init function wich one mode is the display
	LCD_cmd(0x28);		//when we set the mode only 2 times, the display cant work well

	LCD_cmd(CLEAR);			//lcd clear
	LCD_cmd(HOME);			//send the curzor to 0.row 0.cool
	//set the curzor blink and the curzor visibility
	LCD_cmd(0x08 | (1<<LCD_E) | (curzor<<LCD_curzor) | (blink<<LCD_blink));
}
