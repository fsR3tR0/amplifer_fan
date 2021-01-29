/*
 * ledrow.c
 *
 *  Created on: Jan 29, 2021
 *      Author: root
 */
#include "ledrow.h"

void ledrow_max(){
	HAL_GPIO_WritePin(ledrow0_GPIO_Port, ledrow0_Pin, SET);
	HAL_GPIO_WritePin(ledrow1_GPIO_Port, ledrow1_Pin, SET);
	HAL_GPIO_WritePin(ledrow2_GPIO_Port, ledrow2_Pin, SET);
	HAL_GPIO_WritePin(ledrow3_GPIO_Port, ledrow3_Pin, SET);
	HAL_GPIO_WritePin(ledrow4_GPIO_Port, ledrow4_Pin, SET);
	HAL_GPIO_WritePin(ledrow5_GPIO_Port, ledrow5_Pin, SET);
	HAL_GPIO_WritePin(ledrow6_GPIO_Port, ledrow6_Pin, SET);
	HAL_GPIO_WritePin(ledrow7_GPIO_Port, ledrow7_Pin, SET);
	HAL_GPIO_WritePin(ledrow8_GPIO_Port, ledrow8_Pin, SET);
	HAL_GPIO_WritePin(ledrow9_GPIO_Port, ledrow9_Pin, SET);
}

void ledrow_half(void){
	HAL_GPIO_WritePin(ledrow0_GPIO_Port, ledrow0_Pin, SET);
	HAL_GPIO_WritePin(ledrow1_GPIO_Port, ledrow1_Pin, SET);
	HAL_GPIO_WritePin(ledrow2_GPIO_Port, ledrow2_Pin, SET);
	HAL_GPIO_WritePin(ledrow3_GPIO_Port, ledrow3_Pin, SET);
	HAL_GPIO_WritePin(ledrow4_GPIO_Port, ledrow4_Pin, SET);
	HAL_GPIO_WritePin(ledrow5_GPIO_Port, ledrow5_Pin, RESET);
	HAL_GPIO_WritePin(ledrow6_GPIO_Port, ledrow6_Pin, RESET);
	HAL_GPIO_WritePin(ledrow7_GPIO_Port, ledrow7_Pin, RESET);
	HAL_GPIO_WritePin(ledrow8_GPIO_Port, ledrow8_Pin, RESET);
	HAL_GPIO_WritePin(ledrow9_GPIO_Port, ledrow9_Pin, RESET);
}

void ledrow_clear(){
	HAL_GPIO_WritePin(ledrow0_GPIO_Port, ledrow0_Pin, RESET);
	HAL_GPIO_WritePin(ledrow1_GPIO_Port, ledrow1_Pin, RESET);
	HAL_GPIO_WritePin(ledrow2_GPIO_Port, ledrow2_Pin, RESET);
	HAL_GPIO_WritePin(ledrow3_GPIO_Port, ledrow3_Pin, RESET);
	HAL_GPIO_WritePin(ledrow4_GPIO_Port, ledrow4_Pin, RESET);
	HAL_GPIO_WritePin(ledrow5_GPIO_Port, ledrow5_Pin, RESET);
	HAL_GPIO_WritePin(ledrow6_GPIO_Port, ledrow6_Pin, RESET);
	HAL_GPIO_WritePin(ledrow7_GPIO_Port, ledrow7_Pin, RESET);
	HAL_GPIO_WritePin(ledrow8_GPIO_Port, ledrow8_Pin, RESET);
	HAL_GPIO_WritePin(ledrow9_GPIO_Port, ledrow9_Pin, RESET);
}
