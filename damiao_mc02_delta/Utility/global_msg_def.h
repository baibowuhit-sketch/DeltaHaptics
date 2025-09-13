/*
 * global_msg_def.h
 *
 *  Created on: Mar 11, 2025
 *      Author: 19325
 */

#ifndef GLOBAL_MSG_DEF_H_
#define GLOBAL_MSG_DEF_H_

#include "sr_portable.h"
#include "math.h"

namespace sr{

typedef enum : U16 {
    DELTA_JOINT_1 = 0U,
	DELTA_JOINT_2,
	DELTA_JOINT_3,
	DELTA_JOINT_NUM,
} DeltaJointIdx;

struct CANDataFrame {
    U16 id;     /**< CAN message identifier */
    U8 dlc;     /**< Data Length Code (number of bytes in the data field) */
    U8 data[8]; /**< Array to hold CAN message data, maximum of 8 bytes */
};

typedef enum{
    NO_OUTPUT = 0U,
    TORQUE_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL,
    CTRL_MODE_NUMS,
}MotorCtrlMode;

typedef struct{
    F32 gear_rate;
    F32 zero_angle;

    F32 min_angle;
    F32 max_angle;
}JointMotorParas;


typedef struct{
    F32 t;
    F32 rf;
    F32 re;
}DeltaStructureParas;


typedef enum{
    IDLE_STATUS = 0x10U,
    CFG_STATUS = 0x20U,
    RUN_STATUS = 0x30U,
    TUNE_STATUS = 0x40U,
}DeltaRunStatus;

extern volatile U8 timer_1ms_tick_flag;
extern volatile U8 timer_2ms_tick_flag;
extern volatile U8 timer_5ms_tick_flag;
extern volatile U8 timer_10ms_tick_flag;
extern volatile U8 timer_100ms_tick_flag;
extern volatile U8 timer_1000ms_tick_flag;

static const F32 _2_PI = 2.0F * M_PI;
//static const F32 _PI_2 = M_PI / 2.0F;
//static const F32 _PI_3 = M_PI / 3.0F;
//static const F32 _PI_4 = M_PI / 4.0F;
//static const F32 _PI_6 = M_PI / 6.0F;

static const F32 RPM_TO_RADPS				= (_2_PI) / 60.0F;
//static const F32 RADPS_TO_RPM				= 60.0F / (_2_PI);

#define _constrain(amt,low,high) 	((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

inline F32 signVal(F32 valx) {
    return ((valx > 0.0F) ? 1.0F : ((valx < 0.0F) ? -1.0F : 0.0F));
}

inline F32 sqrtVal(F32 valx) {
    return (F32) sqrtf(((valx < 0.0F) ? 0.0F : valx));
}

inline F32 asinVal(F32 valx) {
    return (F32) asinf(_constrain(valx, -1.0F, 1.0F));
}

}

#endif /* GLOBAL_MSG_DEF_H_ */
