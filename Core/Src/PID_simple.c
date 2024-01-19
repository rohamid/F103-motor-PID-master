/*
 * PID_simple.c
 *
 *  Created on: Jan 1, 2024
 *      Author: rd-dev
 */


#include "PID_simple.h"

static float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* @fn 		pid_init
 * @brief	initialize PID constant
 * @return 	None
 * */
void pid_init( pid_config_t *pid,
			   float set_point,
			   float kp,
			   float ki,
			   float kd,
			   float delta_time,
			   float out_min,
			   float out_max ) {
	if(pid == NULL) return;

	pid->KP = kp;
	pid->KI = ki;
	pid->KD = kd;
	pid->dt = delta_time;

	pid->outMin = out_min;
	pid->outMax = out_max;

	// Save setpoint
	pid->setPoint = set_point;

	// Initial error
	pid->error = pid->errorPrev = 0.0;

	// Error integral initial value
	pid->eIntegral = 0.0;
}

/* @fn 		pid_update
 * @brief	update PID constant. This function should be called before
 * 			pid_compute_xxx() in order update all constant before execute
 * @param 	pid		 	a pointer to pid_config_t structure.
 * 			set_point	PID setpoint value
 * 			kp			Proportional constant
 * 			ki			integral constant
 * 			kd			derivative constant
 * 			delta_time	Dt (delta time) constant. (Must be non zero).
 * @return 	None
 * */
void pid_update( pid_config_t *pid,
				 float set_point,
				 float kp,
				 float ki,
				 float kd,
				 float delta_time ) {

	if(pid == NULL) return;

	pid->KP = kp;
	pid->KI = ki;
	pid->KD = kd;
	pid->dt = delta_time;

	// Save setpoint
	pid->setPoint = set_point;
}

/* @fn 		pid_compute_all
 * @brief	Compute controller using all P, I & D constant
 * @param 	pid		 	a pointer to pid_config_t structure.
 * 			input		measurement input
 * @return 	PID's controller output
 * */
float pid_compute_all(pid_config_t *pid, float input) {
	// Determine the error
	pid->error = (pid->setPoint - input);

	// Calculate error integral
	pid->eIntegral = pid->eIntegral + pid->error * pid->dt;

	// Calculate error derivative
	float dedt = (pid->error - pid->errorPrev) / pid->dt;

	// Compute all formulas
	float output = (pid->KP * pid->error) + (pid->KI * pid->eIntegral) + (pid->KD * dedt);

	// Save last error
	pid->errorPrev = pid->error;

	return output;
}

/* @fn 		pid_compute_PI
 * @brief	Compute controller using only P & I constant
 * @param 	pid		 	a pointer to pid_config_t structure.
 * 			input		measurement input
 * @return 	PID's controller output
 * */
float pid_compute_PI(pid_config_t *pid, float input) {
	// Determine the error
	pid->error = (pid->setPoint - input);

	// Calculate error integral
	pid->eIntegral = pid->eIntegral + pid->error * pid->dt;

	// Compute all formulas
	float output = (pid->KP * pid->error) + (pid->KI * pid->eIntegral);

	// Save last error
	pid->errorPrev = pid->error;

	return output;
}

/* @fn 		pid_compute_PD
 * @brief	Compute controller using only P & D constant
 * @param 	pid		 	a pointer to pid_config_t structure.
 * 			input		measurement input
 * @return 	PID's controller output
 * */
float pid_compute_PD(pid_config_t *pid, float input) {
	// Determine the error
	pid->error = (pid->setPoint - input);

	// Calculate error derivative
	float dedt = (pid->error - pid->errorPrev) / pid->dt;

	// Compute all formulas
	float output = (pid->KP * pid->error) + (pid->KD * dedt);

	// Save last error
	pid->errorPrev = pid->error;

	return output;
}

/* @fn 		pid_set_output_limit
 * @brief	Clamps the PID output
 * @param 	pid		 	a pointer to pid_config_t structure.
 * 			out_min		output minimal value
 * 			out_max		output maximal value
 * */
void pid_set_output_limit(pid_config_t *pid, float out_min, float out_max) {
	pid->outMin = out_min;
	pid->outMax = out_max;
}
