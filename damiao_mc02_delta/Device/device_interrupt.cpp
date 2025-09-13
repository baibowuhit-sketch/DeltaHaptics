/*
 * device_interrupt.cpp
 *
 *  Created on: Mar 11, 2025
 *      Author: 19325
 */

#include "main.h"
#include "gm6020_comm.h"

static volatile U32 timer_1ms_tick_cnt = 0U;
volatile U8 sr::timer_1ms_tick_flag = 0U;
volatile U8 sr::timer_2ms_tick_flag = 0U;
volatile U8 sr::timer_5ms_tick_flag = 0U;
volatile U8 sr::timer_10ms_tick_flag = 0U;
volatile U8 sr::timer_100ms_tick_flag = 0U;
volatile U8 sr::timer_1000ms_tick_flag = 0U;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM6){
		sr::timer_1ms_tick_flag = 1U;

		timer_1ms_tick_cnt++;
		if(timer_1ms_tick_cnt % 2U == 0U){
			sr::timer_2ms_tick_flag = 1U;
		}
		if(timer_1ms_tick_cnt % 5U == 0U){
			sr::timer_5ms_tick_flag = 1U;
		}
		if(timer_1ms_tick_cnt % 10U == 0U){
			sr::timer_10ms_tick_flag = 1U;
		}
		if(timer_1ms_tick_cnt % 100U == 0U){
			sr::timer_100ms_tick_flag = 1U;
		}
		if(timer_1ms_tick_cnt % 1000U == 0U){
			timer_1ms_tick_cnt = 0U;

			sr::timer_1000ms_tick_flag = 1U;
		}
	}
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if((hfdcan->Instance == FDCAN1) &&
	   ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)){
		FDCAN_RxHeaderTypeDef rx_header;
		sr::CANDataFrame frame;

		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, frame.data) == HAL_OK){
			frame.id = (U16)rx_header.Identifier;
			frame.dlc = (U8)rx_header.DataLength;
			gm6020_comm.can_recv_queue.push(frame);
		}
	}
}


