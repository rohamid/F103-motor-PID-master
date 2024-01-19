/*
 * PID_simple.h
 *
 *  Created on: Jan 1, 2024
 *      Author: rd-dev
 */

#ifndef INC_PID_SIMPLE_H_
#define INC_PID_SIMPLE_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

typedef struct pid_config_t {
	float KP;
	float KI;
	float KD;
	float setPoint;
	float error;
	float errorPrev;
	float eIntegral;
	float dt;
	float outMin;
	float outMax;
}pid_config_t;

void pid_init( pid_config_t *pid,
			   float set_point,
			   float kp,
			   float ki,
			   float kd,
			   float delta_time,
			   float out_min,
			   float out_max );

void pid_update( pid_config_t *pid,
				  float set_point,
				  float kp,
				  float ki,
				  float kd,
				  float delta_time );

float pid_compute_all(pid_config_t *pid, float input);
float pid_compute_PI(pid_config_t *pid, float input);
float pid_compute_PD(pid_config_t *pid, float input);
void pid_set_output_limit(pid_config_t *pid, float out_min, float out_max);

#endif /* INC_PID_SIMPLE_H_ */
