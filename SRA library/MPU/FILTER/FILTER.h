/**
	FILTER library for WALL-E 2.0
	This module is used to convert the raw readings
	from the MPU module into usable filtered readings.
	The filter which we have used is complementary.


	For more information visit http://sra.vjti.info/

	This code is in the public domain.

	by Society Of Robotics And Automation, VJTI.
*/

/*Can also use #pragma once*/
#ifndef FILTER_H
#define FILTER_H

#include <MPU.h>

static long	prev_time = 0;                				//	A variable to hold the time elapsed till previous iteration


float calc_angular_velocity(int16_t , float );			//Calculates angular Velocity using Gyro data

float calc_acce_angle(int16_t , int16_t, float);		//Calculates angle using only Accelerometer

float complementary_filter(float, float, float);		//Calculates angle using Gyro data and Acce Data

void  calculatePitchAngle(float &, int16_t, int16_t, int16_t, float, float); 	//Returns appropriate angle according to complementary filter

#endif
