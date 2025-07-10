/*
 * OID.c
 *
 *  Created on: Jul 9, 2025
 *      Author: u033008
 */

#include "PID.h"
#include "Moving_system.h"

PID_contriller PID_AZ =
{
		.Kd = 0.0f,
		.Ki = 0.5f,
		.Kp = 10.0f,
		.integral = 0,
		.prev_error = 0
};

PID_contriller PID_EL =
{
		.Kd = 0.0f,
		.Ki = 0.5f,
		.Kp = 10.0f,
		.integral = 0,
		.prev_error = 0
};

float PID_compute(PID_contriller *pid, float setpoint, float input)
{
	uint32_t now = HAL_GetTick();
	float dt = (now - pid->last_time) / 1000.0f; // В секундах
	pid->last_time = now;

	if (dt <= 0) dt = 0.001f;

	float error = setpoint - input;

	float P = pid->Kp * error;

	pid->integral += error * dt;
	if (pid->integral > 100.0f) pid->integral = 100.0f;
	if (pid->integral < -100.0f) pid->integral = -100.0f;
	float I = pid->Ki * pid->integral;

	float derivative = (error - pid->prev_error) / dt;
	float D = pid->Kd * derivative;
	pid->prev_error = error;

	return P + I + D;
};

uint32_t PID_control(PID_contriller *pid, float target, float current)
{
	float error = target - current;
	float pid_out = PID_compute(pid, 0, error);
	pid_out = CLAMP(pid_out, -1000, 1000);

	return pid_out;
};
