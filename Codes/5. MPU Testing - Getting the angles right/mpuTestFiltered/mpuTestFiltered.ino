/*
  MPU Filtered Values
  We are using the complimentory filter from the library and calculating the final angle from raw values
  
  By Society of Robotics and Automation, VJTI
*/

#include <MPU.h>
#include <FILTER.h>

/*
 * Global variables for MPU values along X,Y,Z axes
 * Need to read them in this order only. Already done in library.
 */
int16_t rawAcceX, rawAcceY, rawAcceZ;
int16_t rawTemp;
int16_t rawGyroX, rawGyroY, rawGyroZ;

/*
 * They are Acce and Gyro set points respectively
 */
float initialAcceAngle = 0;
float initialGyroAngle = 0;

float pitchAngle = 0;

void printValues()
{
  Serial.print("Angle: "); Serial.print(pitchAngle);
  Serial.println();
}

void setup()
{
  startMPU();
  Serial.begin(9600);
}

void loop()
{
  readRawValuesOfMPU(rawAcceX, rawAcceY, rawAcceZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ);            //Read the values of the acce and gyro according to current bot position  
  calculatePitchAngle(pitchAngle, rawAcceX, rawAcceZ, rawGyroY, initialAcceAngle, initialGyroAngle);  //Calculating the final pitchAngle
  printValues();
}
