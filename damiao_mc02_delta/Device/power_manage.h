/*
 * power_manage.h
 *
 *  Created on: Mar 11, 2025
 *      Author: 19325
 */

#ifndef POWER_MANAGE_H_
#define POWER_MANAGE_H_

#include "sr_portable.h"

void controlPower1State(const U8 state = 0U);

void controlPower2State(const U8 state = 0U);

void controlPower5VState(const U8 state = 0U);

#endif /* POWER_MANAGE_H_ */
