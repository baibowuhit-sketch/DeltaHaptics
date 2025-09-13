/*
 * motor_pid.cpp
 *
 *  Created on: Mar 12, 2025
 *      Author: 19325
 */

#include "motor_pid.h"
#include "global_msg_def.h"

namespace pid{

void pidClear(PidTypeDef *pid) {
    pid->i_out = 0.0F;
    pid->d_out = 0.0F;
    pid->u = 0.0F;
    pid->s = 0.0F;
    pid->r = 0.0F;
    pid->e[0] = 0.0F;
    pid->e[1] = 0.0F;
}

void pidSetParam(PidTypeDef *pid, const PidParam param_) {
    static const F32 inte_k = 0.85F;

    pid->param = param_;
    pid->inte_limit = param_.out_limit * inte_k;
    pid->param.d_alpha = _constrain(pid->param.d_alpha, 0.0F, 1.0F);
}

F32 pidCalc(PidTypeDef *pid, const F32 rel_val, const F32 set_val) {

    pid->s = set_val;
    pid->r = rel_val;
    pid->e[1] = pid->s - pid->r;
    if(fabsf(pid->e[1]) < pid->param.e_deadband){
        pid->e[1] = 0.0F;
    }

    pid->p_out = pid->param.k_p * pid->e[1];

    pid->i_out += pid->param.k_i * (pid->e[1]);
    pid->i_out = _constrain(pid->i_out, -pid->inte_limit, pid->inte_limit);

    pid->d_out = pid->param.d_alpha * (pid->param.k_d * (pid->e[1] - pid->e[0])) +
                (1.0F - pid->param.d_alpha) * pid->d_out;

    pid->u = pid->p_out + pid->i_out + pid->d_out;
    pid->u = _constrain(pid->u, -pid->param.out_limit, pid->param.out_limit);

    pid->e[0] = pid->e[1];

    return pid->u;
}




}
