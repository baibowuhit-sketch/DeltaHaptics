/*
 * delta_robot.h
 *
 *  Created on: Mar 17, 2025
 *      Author: 19325
 */

#ifndef DELTA_ROBOT_H_
#define DELTA_ROBOT_H_


#include "global_msg_def.h"
#include "ring_queue.h"

class DeltaRobot {
public:
    static DeltaRobot &construction(void);

    RingQueue<sr::CANDataFrame> can_recv_queue;

    void configParams(const sr::DeltaStructureParas dt_paras);

    void preCalcSomething();
    void handleEvent();
    void run1msLoop();
    void getMotionInfo(F32 (&end_pos)[3], F32 (&end_vel)[3],
                       F32 (&end_acc)[3], F32 (&end_ft)[3]);
private:
    DeltaRobot();
    DeltaRobot(const DeltaRobot&);
    DeltaRobot &operator=(const DeltaRobot&);

    sr::CANDataFrame can_rx_queue_buf[sr::DELTA_JOINT_NUM * 10];

    sr::DeltaRunStatus run_status;
    F32 control_period_s;
    sr::DeltaStructureParas delta_paras;

    F32 joint_angle[sr::DELTA_JOINT_NUM];
    F32 joint_velocity[sr::DELTA_JOINT_NUM];
    F32 joint_acceleration[sr::DELTA_JOINT_NUM];
    F32 joint_torque[sr::DELTA_JOINT_NUM];

    F32 end_position[3];
    F32 end_velocity[3];
    F32 end_acceleration[3];
    F32 end_force[3];
};

#define delta_robot (DeltaRobot::construction())

#endif /* DELTA_ROBOT_H_ */
