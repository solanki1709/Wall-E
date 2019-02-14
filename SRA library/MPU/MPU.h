/**
	MPU library for WALL-E 2.0
	This module is used to get raw values of
	acceleration components in the direction of the X-Axis, Y-Axis and Z-Axis,
	and the raw values on angular velocities about the X-Axis, Y-Axis and Z-Axis.

	For more information visit http://sra.vjti.info/

	This code is in the public domain.

	by Society Of Robotics And Automation, VJTI.
*/

/*Can also use #pragma once*/
#ifndef MPU_H
#define MPU_H

#include <Wire.h>	//	For I2C Communication


void startMPU(void);					//Wakes up the MPU by clearing the PWR_MGMT_1 register for MPU-6050

void readRawValuesOfMPU(int16_t &, int16_t &, int16_t &, int16_t &, int16_t &, int16_t &, int16_t &);	//Reads raw values from the registers of the MPU

void calibrateMPU(float &, float &);	//Gets initial raw reading from gyroscope and initial angle calculated by accelerometer

#endif