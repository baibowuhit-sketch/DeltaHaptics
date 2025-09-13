/*
 * motor_pid.h
 *
 *  Created on: Mar 12, 2025
 *      Author: 19325
 */

#ifndef MOTOR_PID_H_
#define MOTOR_PID_H_

#include "sr_portable.h"

namespace pid{
	typedef struct {
		F32 k_p;
		F32 k_i;
		F32 k_d;

		F32 d_alpha;    // d filter lambda
		F32 out_limit;
		F32 e_deadband;
	} __attribute__((packed)) PidParam;

	typedef struct {
	    F32 s;
	    F32 r;
	    F32 e[2];

	    PidParam param;
	    F32 inte_limit;

	    /*PID output*/
	    F32 p_out;
	    F32 i_out;
	    F32 d_out;

	    F32 last_ud;
	    F32 u;

	}PidTypeDef;

	void pidClear(PidTypeDef *pid);
	void pidSetParam(PidTypeDef *pid, const PidParam param_);
	F32 pidCalc(PidTypeDef *pid, const F32 rel_val, const F32 set_val);
}





#endif /* MOTOR_PID_H_ */
