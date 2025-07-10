/*
 * PID.h
 *
 *  Created on: Jul 9, 2025
 *      Author: u033008
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32f4xx_hal.h"
#include <math.h>

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float prev_error;
	float integral;
	uint32_t last_time;
} PID_contriller;

extern PID_contriller PID_AZ;
extern PID_contriller PID_EL;

float PID_compute(PID_contriller *pid, float setpiont, float input);
uint32_t PID_control(PID_contriller *pid, float target, float current);

#endif /* INC_PID_H_ */
