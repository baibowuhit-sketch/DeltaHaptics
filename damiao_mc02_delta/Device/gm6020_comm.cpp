/*
 * gm6020_comm.cpp
 *
 *  Created on: Mar 11, 2025
 *      Author: 19325
 */

#include "gm6020_comm.h"
#include "main.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

static const U16 GM6020_START_CANID			= 0x205U;
static const U16 GM6020_SEND_CANID			= 0x1FEU;

static const F32 GM6020_TORQUE_COEFF		= 0.1105F;
static const U16 GM6020_MIN_RECV_COUNT		= 10U;
static const U16 GM6020_ANG_RESOLUTION		= 8192U;
static const U16 GM6020_ANG_RESOLUTION_F	= (F32)GM6020_ANG_RESOLUTION;
static const S16 GM6020_CONTROL_VALUE_MAX   = 15000;
//namespace GM6020
//{

static FDCAN_TxHeaderTypeDef tx_header;

static S16 calcCircleTimeByRange(const U16 count, const sr::JointMotorParas para){
	const S16 try_circle_times[5] = {-2, -1, 0, 1, 2};

	S16 best_circle_time = 0;
	F32 min_range_err = 1.0e6F;

	for(U16 i = 0U; i < 5U; i++){
		S16 circle_time_tmp = try_circle_times[i];
		F32 angle_tmp = (F32)((F32)count / GM6020_ANG_RESOLUTION_F + (F32)circle_time_tmp)
							  * (sr::_2_PI) / para.gear_rate + para.zero_angle;

		F32 range_err = 0.0F;
		if(angle_tmp < para.min_angle){
			range_err = para.min_angle - angle_tmp;
		}
		else if(angle_tmp > para.max_angle){
			range_err = angle_tmp - para.max_angle;
		}
		else{}

		if(range_err < min_range_err){
			min_range_err = range_err;
			best_circle_time = circle_time_tmp;
		}
	}

	return best_circle_time;
}

Gm6020Comm &Gm6020Comm::construction(void) {
    static Gm6020Comm obj;
    return obj;
}

Gm6020Comm::Gm6020Comm(){
    can_recv_queue.setBuff(can_rx_queue_buf,
    		(S16)sizeof(can_rx_queue_buf) / (S16)sizeof(can_rx_queue_buf[0]));

    F32 gear_rate_cali[GM6020_MOTOR_NUM] = {-1.0F, 1.0F, 1.0F};
    F32 zero_angle_cali[GM6020_MOTOR_NUM] = {4.753F, 0.5074F, 0.45383F};
    for(U16 i = 0U; i < GM6020_MOTOR_NUM; i++){
    	joint_paras[i].gear_rate = gear_rate_cali[i];
    	joint_paras[i].zero_angle = zero_angle_cali[i];
    	joint_paras[i].min_angle = -M_PI / 3.0F;
    	joint_paras[i].max_angle = 2.0F * M_PI / 3.0F;

    	recv_frame_count[i] = 0U;
    	circle_time[i] = 0;
    	angle[i] = 0.0F;
    	velocity[i] = 0.0F;
    	acceleration[i] = 0.0F;
    	torque[i] = 0.0F;
    	temperature[i] = 0;
    	control_value[i] = 0;

    	pos_pid[i].param.k_p = 10000.0F;
    	pos_pid[i].param.k_d = 100000.0F;
    	pos_pid[i].param.k_i = 10.0F;
    	pos_pid[i].param.d_alpha = 1.0F;
    	pos_pid[i].inte_limit = 10000.0F;
    	pos_pid[i].param.out_limit = 12000.0F;
    	pos_pid[i].param.e_deadband = 0.0F;
    	pidClear(&(pos_pid[i]));

    	vel_pid[i].param.k_p = 1000.0F;
    	vel_pid[i].param.k_d = 10000.0F;
    	vel_pid[i].param.k_i = 1.0F;
    	vel_pid[i].param.d_alpha = 1.0F;
    	vel_pid[i].inte_limit = 10000.0F;
    	vel_pid[i].param.out_limit = 12000.0F;
    	vel_pid[i].param.e_deadband = 0.0F;
    	pidClear(&(vel_pid[i]));
    }
    control_period_s = 0.001F;

    tx_header.Identifier = GM6020_SEND_CANID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
}


void Gm6020Comm::configParams(const sr::JointMotorParas (&jm_paras)[GM6020_MOTOR_NUM],
				  	  	  	  const pid::PidParam (&pid_pos_paras)[GM6020_MOTOR_NUM],
							  const pid::PidParam (&pid_vel_paras)[GM6020_MOTOR_NUM]){
	for(U16 i = 0U; i < GM6020_MOTOR_NUM; i++){
		joint_paras[i] = jm_paras[i];

		pidSetParam(&(pos_pid[i]), pid_pos_paras[i]);
		pidSetParam(&(vel_pid[i]), pid_vel_paras[i]);
	}
}

void Gm6020Comm::handleEvent(){
	while (SR_FALSE == can_recv_queue.isEmpty()) {
		sr::CANDataFrame frame = can_recv_queue.pop();
        if(((frame.id >= GM6020_START_CANID) &&
        	(frame.id < GM6020_START_CANID + GM6020_MOTOR_NUM)) &&
        	(frame.dlc == 8U)){
        	U16 id = (U16)(frame.id - GM6020_START_CANID);

        	if(id < GM6020_MOTOR_NUM){
				U16 count = (U16)((frame.data[0] << 8U) | (frame.data[1]));
				S16 speed = (S16)((frame.data[2] << 8U) | (frame.data[3]));
				S16 current = (S16)((frame.data[4] << 8U) | (frame.data[5]));
				temperature[id] = (S16)frame.data[6];

				if(count > last_count[id] + GM6020_ANG_RESOLUTION/2){
					circle_time[id]--;
				}
				else if(last_count[id] > count + GM6020_ANG_RESOLUTION/2){
					circle_time[id]++;
				}
				else{}
				last_count[id] = count;

				angle[id] = (F32)((F32)count / GM6020_ANG_RESOLUTION_F + (F32)circle_time[id])
								* (sr::_2_PI) / joint_paras[id].gear_rate + joint_paras[id].zero_angle;
				velocity[id] = (F32)speed * sr::RPM_TO_RADPS;
				torque[id] = (F32)current * GM6020_TORQUE_COEFF;

				if(recv_frame_count[id] < GM6020_MIN_RECV_COUNT){
					recv_frame_count[id]++;

					if(recv_frame_count[id] == GM6020_MIN_RECV_COUNT){
						/* set circle time by range */
						circle_time[id] = calcCircleTimeByRange(count, joint_paras[id]);
					}
				}
        	}

        }
    }
}

void Gm6020Comm::getMotionInfo(F32 (&torq)[GM6020_MOTOR_NUM], F32 (&ang)[GM6020_MOTOR_NUM],
                   	   	   	   F32 (&vel)[GM6020_MOTOR_NUM], F32 (&acc)[GM6020_MOTOR_NUM]){
	memcpy(torq, torque, GM6020_MOTOR_NUM * sizeof(F32));
	memcpy(ang, angle, GM6020_MOTOR_NUM * sizeof(F32));
	memcpy(vel, velocity, GM6020_MOTOR_NUM * sizeof(F32));
	memcpy(acc, acceleration, GM6020_MOTOR_NUM * sizeof(F32));
}

void Gm6020Comm::sendTargetMotionCmd(const sr::MotorCtrlMode (&ctrl_mode)[GM6020_MOTOR_NUM],
									 const F32 (&targ_torq)[GM6020_MOTOR_NUM],
									 const F32 (&targ_ang)[GM6020_MOTOR_NUM],
									 const F32 (&targ_vel)[GM6020_MOTOR_NUM]){
	U8 tx_data[8] = {0};
	for(U16 i = 0U; i < GM6020_MOTOR_NUM; i++){
		if(i > 3){
			break;
		}
		S16 send_val = 0;

        //angle planning
        if(ctrl_mode[i] == sr::POSITION_CONTROL){
            F32 d_angle_thred = fabsf(targ_vel[i]) * control_period_s;
            if(targ_ang[i] - planned_angle[i] > d_angle_thred){
            	planned_angle[i] += d_angle_thred;
            }
            else if(targ_ang[i] - planned_angle[i] < -d_angle_thred){
            	planned_angle[i] -= d_angle_thred;
            }
            else{
            	planned_angle[i] = targ_ang[i];
            }
        }
        else{
        	planned_angle[i] = angle[i];
        }

		switch(ctrl_mode[i]){
			case sr::NO_OUTPUT:
			{
				send_val = 0;
				break;
			}
			case sr::TORQUE_CONTROL:
			{
				send_val = (S16)roundf(targ_torq[i] / GM6020_TORQUE_COEFF / joint_paras[i].gear_rate);
				break;
			}
			case sr::VELOCITY_CONTROL:
			{
				F32 pid_out = pidCalc(&(vel_pid[i]), velocity[i] * joint_paras[i].gear_rate,
									    targ_vel[i] * joint_paras[i].gear_rate);
				send_val = (S16)roundf(pid_out);
				break;
			}
			case sr::POSITION_CONTROL:
			{
				F32 pid_out = pidCalc(&(pos_pid[i]), angle[i] * joint_paras[i].gear_rate,
										planned_angle[i] * joint_paras[i].gear_rate);
				send_val = (S16)roundf(pid_out);
				break;
			}
			default:
			{
				send_val = 0;
				break;
			}
		}

		send_val = _constrain(send_val, -GM6020_CONTROL_VALUE_MAX, GM6020_CONTROL_VALUE_MAX);
		tx_data[2*i + 0] = (U8)((send_val >> 8) & 0xFF);
		tx_data[2*i + 1] = (U8)(send_val & 0xFF);
	}

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) != HAL_OK)
    {
//    	printf("CAN Send Error !!! \r\n");
    }
}

//}/* end of GM6020 namespace */
