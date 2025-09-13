/*
 * power_manage.cpp
 *
 *  Created on: Mar 11, 2025
 *      Author: 19325
 */

#include "power_manage.h"
#include "main.h"

void controlPower1State(const U8 state){
	if(state == 0U){
		HAL_GPIO_WritePin(Power1_EN_GPIO_Port, Power1_EN_Pin, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(Power1_EN_GPIO_Port, Power1_EN_Pin, GPIO_PIN_SET);
	}
}

void controlPower2State(const U8 state){
	if(state == 0U){
		HAL_GPIO_WritePin(Power2_EN_GPIO_Port, Power2_EN_Pin, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(Power2_EN_GPIO_Port, Power2_EN_Pin, GPIO_PIN_SET);
	}
}

void controlPower5VState(const U8 state){
	if(state == 0U){
		HAL_GPIO_WritePin(Power5V_EN_GPIO_Port, Power5V_EN_Pin, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(Power5V_EN_GPIO_Port, Power5V_EN_Pin, GPIO_PIN_SET);
	}
}


