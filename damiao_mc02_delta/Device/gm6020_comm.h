/*
 * gm6020.h
 *
 *  Created on: Mar 11, 2025
 *      Author: 19325
 */

#ifndef GM6020_COMM_H_
#define GM6020_COMM_H_

#include "global_msg_def.h"
#include "ring_queue.h"
#include "motor_pid.h"

//namespace GM6020
//{

#define GM6020_MOTOR_NUM	(sr::DELTA_JOINT_NUM)

class Gm6020Comm {
public:
    static Gm6020Comm &construction(void);

    RingQueue<sr::CANDataFrame> can_recv_queue;

    void configParams(const sr::JointMotorParas (&jm_paras)[GM6020_MOTOR_NUM],
    				  const pid::PidParam (&pid_pos_paras)[GM6020_MOTOR_NUM],
    				  const pid::PidParam (&pid_vel_paras)[GM6020_MOTOR_NUM]);

    void handleEvent();
    void getMotionInfo(F32 (&torq)[GM6020_MOTOR_NUM], F32 (&ang)[GM6020_MOTOR_NUM],
                       F32 (&vel)[GM6020_MOTOR_NUM], F32 (&acc)[GM6020_MOTOR_NUM]);

    void sendTargetMotionCmd(const sr::MotorCtrlMode (&ctrl_mode)[GM6020_MOTOR_NUM],
                             const F32 (&targ_torq)[GM6020_MOTOR_NUM],
                             const F32 (&targ_ang)[GM6020_MOTOR_NUM],
                             const F32 (&targ_vel)[GM6020_MOTOR_NUM]);
private:
    Gm6020Comm();
    Gm6020Comm(const Gm6020Comm&);
    Gm6020Comm &operator=(const Gm6020Comm&);

    sr::CANDataFrame can_rx_queue_buf[GM6020_MOTOR_NUM * 10];
    sr::JointMotorParas joint_paras[GM6020_MOTOR_NUM];
    pid::PidParam pid[GM6020_MOTOR_NUM];

    pid::PidTypeDef pos_pid[GM6020_MOTOR_NUM];
    pid::PidTypeDef vel_pid[GM6020_MOTOR_NUM];
    F32 planned_angle[GM6020_MOTOR_NUM];

    F32 control_period_s;
    U16 recv_frame_count[GM6020_MOTOR_NUM];
    U16 last_count[GM6020_MOTOR_NUM];
    S16 circle_time[GM6020_MOTOR_NUM];
    F32 angle[GM6020_MOTOR_NUM];
    F32 velocity[GM6020_MOTOR_NUM];
    F32 acceleration[GM6020_MOTOR_NUM];
    F32 torque[GM6020_MOTOR_NUM];
    S16 temperature[GM6020_MOTOR_NUM];

    S16 control_value[GM6020_MOTOR_NUM];
};

#define gm6020_comm (Gm6020Comm::construction())

//}/* end of GM6020 namespace */

#endif /* GM6020_COMM_H_ */
