/**	
	FILTER LIBRARY FOR WALL-E 2.0
	By Society Of Robotics And Automation

*/
#include <Filter.h>
#include <Arduino.h>
#include <avr/eeprom.h>

float calc_angular_velocity(int16_t rawGyroY, float initialGyroAngle)
{
	return ((rawGyroY - initialGyroAngle) / 131);                     // FS_SEL = 0, so 131 raw reading corrusponds to 1 deg / s BY default FS_SEL = 0
}

float calc_acce_angle(int16_t rawAcceX, int16_t rawAcceZ, float initialAcceAngle)
{
	long acceX = (long)rawAcceX;
    long acceZ = (long)rawAcceZ;

    long acceXZ  = sqrt((acceX * acceX) + (acceZ * acceZ));
    float currentAcceAngle = asin(((float)acceX) / ((float)acceXZ)) * RAD_TO_DEG;

    return currentAcceAngle - initialAcceAngle;
}

float complementary_filter(float angular_velocity, float acce_angle, float prev_angle, float gyro_wt)
{
	long current_time 		= millis();
	int time_diff			= current_time - prev_time;
	prev_time        		= current_time;
	return (gyro_wt * (prev_angle + angular_velocity * time_diff * 0.001) + (1 - gyro_wt) * acce_angle);
}

void calculatePitchAngle(float &angle, int16_t rawAcceX, int16_t rawAcceZ, int16_t rawGyroY, float initialAcceAngle, float initialGyroAngle)
{

	float angular_velocity	= calc_angular_velocity(rawGyroY, initialGyroAngle);
	float acce_angle		= calc_acce_angle(rawAcceX, rawAcceZ, initialAcceAngle);
	angle					= complementary_filter(angular_velocity, acce_angle, angle, 0.90);


	if(isnan(angle))
		angle	= int(eeprom_read_word(0));
	else eeprom_write_word(0, int(angle));

}
