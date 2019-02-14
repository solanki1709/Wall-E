/**
	MPU LIBRARY FOR WALL-E 2.0
	By Society Of Robotics And Automation
*/

#include <MPU.h>
#include <Arduino.h>

#define RAD_TO_DEG 	57.2975                        							// 180 / Pi
#define MPU_ADDR 	0x68

void startMPU(void)
{
	Wire.begin();
	Wire.beginTransmission(MPU_ADDR);										// START signal
	Wire.write(0x6B);           											// PWR_MGMT_1 register
	Wire.write(0);              											// set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);												// STOP signal
}

void readRawValuesOfMPU(int16_t &rawAcceX, int16_t &rawAcceY, int16_t &rawAcceZ, int16_t &rawTemp, int16_t &rawGyroX, int16_t &rawGyroY, int16_t &rawGyroZ)
{
	startMPU();																// Starts MPU to make it Failiure proof
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x3B);														// Starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);											// REPEATED START signal
	Wire.requestFrom(MPU_ADDR,14,true);  									// request a total of 14 registers

	if(Wire.available())
	{
		rawAcceX	=	Wire.read()	<<	8	|	Wire.read();  				// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
		rawAcceY	=	Wire.read()	<<	8	|	Wire.read();  				// 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
		rawAcceZ	=	Wire.read()	<<	8	|	Wire.read();  				// 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
		rawTemp		=	Wire.read()	<<	8	|	Wire.read();  				// 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
		rawGyroX	=	Wire.read()	<<	8	|	Wire.read();  				// 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
		rawGyroY	=	Wire.read()	<<	8	|	Wire.read();  				// 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
		rawGyroZ	=	Wire.read()	<<	8	|	Wire.read();  				// 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	}
}

void calibrateMPU(float &initialAcceAngle, float &initialGyroAngle)
{
	initialAcceAngle	= 0;												// Initialize everything to zero
	initialGyroAngle	= 0;

    int16_t  rawAcceX, rawAcceY, rawAcceZ;
    int16_t rawTemp;
    int16_t rawGyroX, rawGyroY, rawGyroZ;

    static long 	totalAcceX	= 0, avgAcceX = 0;
	static long 	totalAcceZ	= 0, avgAcceZ = 0;
	static long		totalAcceXZ;

    for(int i = 0; i < 2000; i++) {										// Calculate the average initial reading over 2000 readings
        readRawValuesOfMPU(rawAcceX, rawAcceY, rawAcceZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ);
        totalAcceX += rawAcceX;
        totalAcceZ += rawAcceZ;
        initialGyroAngle += rawGyroY;
    }

    avgAcceX = totalAcceX / 2000;
    avgAcceZ = totalAcceZ / 2000;

    totalAcceXZ = sqrt((avgAcceX * avgAcceX) + (avgAcceZ * avgAcceZ));

    initialAcceAngle	= asin(((float)avgAcceX) / ((float)totalAcceXZ)) * RAD_TO_DEG;
    initialGyroAngle	= initialGyroAngle	/ 2000.0;
}
