/*
 * init.cpp
 *
 *  Created on: Mar 10, 2025
 *      Author: 19325
 */

#include "init.h"
#include "w2812_comm.h"
#include "power_manage.h"
#include "delta_robot.h"

void init(){
	controlPower1State(1U);
	controlPower2State(0U);
	controlPower5VState(0U);

	w2812_comm.construction();
	w2812_comm.ctrlColor(0, 100, 0);

	delta_robot.construction();
}

void startUpTask(){
	/* init */
	init();

	/* loop */
	while(1){
		if(sr::timer_1ms_tick_flag == 1U){
			sr::timer_1ms_tick_flag = 0U;

			/* do something every 1ms */
			delta_robot.handleEvent();

			delta_robot.run1msLoop();
		}

		if(sr::timer_2ms_tick_flag == 1U){
			sr::timer_2ms_tick_flag = 0U;

			/* do something every 2ms */
		}

		if(sr::timer_5ms_tick_flag == 1U){
			sr::timer_5ms_tick_flag = 0U;

			/* do something every 5ms */
		}

		if(sr::timer_10ms_tick_flag == 1U){
			sr::timer_10ms_tick_flag = 0U;

			/* do something every 10ms */
		}

		if(sr::timer_100ms_tick_flag == 1U){
			sr::timer_100ms_tick_flag = 0U;

			/* do something every 100ms */
		}

		if(sr::timer_1000ms_tick_flag == 1U){
			sr::timer_1000ms_tick_flag = 0U;

			/* do something every 1000ms */
		}
	}
}

