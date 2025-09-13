/*
 * delta_robot.cpp
 *
 *  Created on: Mar 17, 2025
 *      Author: 19325
 */

#include "delta_robot.h"
#include "gm6020_comm.h"
#include "main.h"
#include <string.h>

DeltaRobot &DeltaRobot::construction(void) {
    static DeltaRobot obj;
    return obj;
}

DeltaRobot::DeltaRobot(){
	gm6020_comm.construction();

	can_recv_queue.setBuff(can_rx_queue_buf,
    		(S16)sizeof(can_rx_queue_buf) / (S16)sizeof(can_rx_queue_buf[0]));

    run_status = sr::DeltaRunStatus::IDLE_STATUS;
    control_period_s = 0.001F;
    delta_paras.rf = 0.150F;	//unit: m
    delta_paras.re = 0.200F;	//unit: m
    delta_paras.t = 0.0306F;	//unit: m

    for(U16 i = 0U; i < sr::DELTA_JOINT_NUM; i++){
    	joint_angle[i] = 0.0F;
    	joint_velocity[i] = 0.0F;
    	joint_acceleration[i] = 0.0F;
    	joint_torque[i] = 0.0F;
    }

    memset(end_position, 0, 3 * sizeof(F32));
    memset(end_velocity, 0, 3 * sizeof(F32));
    memset(end_acceleration, 0, 3 * sizeof(F32));
    memset(end_force, 0, 3 * sizeof(F32));
}

void DeltaRobot::configParams(const sr::DeltaStructureParas dt_paras){
	memcpy((void*)&delta_paras, (void*)&dt_paras, sizeof(sr::DeltaStructureParas));
}

void DeltaRobot::handleEvent()
{

	gm6020_comm.handleEvent();

}

void DeltaRobot::run1msLoop(){
	sr::MotorCtrlMode ctrl_mode[GM6020_MOTOR_NUM] = {sr::NO_OUTPUT, sr::NO_OUTPUT, sr::NO_OUTPUT};
	F32 targ_torq[GM6020_MOTOR_NUM] = {0};
	F32 targ_ang[GM6020_MOTOR_NUM] = {0};
	F32 targ_vel[GM6020_MOTOR_NUM] = {0};

	switch(run_status){
	case sr::DeltaRunStatus::IDLE_STATUS:
	{
		gm6020_comm.sendTargetMotionCmd(ctrl_mode, targ_torq, targ_ang, targ_vel);
		break;
	}
	default:
	{
		break;
	}
	}
}

void DeltaRobot::getMotionInfo(F32 (&end_pos)[3], F32 (&end_vel)[3],
                   	   	   	   F32 (&end_acc)[3], F32 (&end_ft)[3]){

}

