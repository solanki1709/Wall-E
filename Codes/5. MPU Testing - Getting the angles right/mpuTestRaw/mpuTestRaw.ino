/*
  MPU Testing
  Reading the raw values of the MPU(Acce and Gyro) using the MPU library and printing the values
 
  By Society of Robotics and Automation, VJTI
*/

#include <MPU.h>

/*
 * Global variables for MPU values along X,Y,Z axes
 * Need to read them in this order only. Already done in library.
 */
int16_t rawAcceX, rawAcceY, rawAcceZ;
int16_t rawTemp;
int16_t rawGyroX, rawGyroY, rawGyroZ;

void setup()
{
  startMPU();           //Initialization of MPU i.e. Acce and Gyro calibration   
  Serial.begin(9600);
}

void loop()
{
  readRawValuesOfMPU(rawAcceX, rawAcceY, rawAcceZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ);   //Read the values of the acce and gyro according to current bot position
  printData();
}

void printData()
{
  //Printing Acclerometer Values
  Serial.print("rawAcceX: "); Serial.print(rawAcceX); Serial.print("\t");
  Serial.print("rawAcceY: "); Serial.print(rawAcceY); Serial.print("\t");
  Serial.print("rawAcceZ: "); Serial.print(rawAcceZ); Serial.print("\t");

  //Printing Gyrometer Values
  Serial.print("rawGyroX: "); Serial.print(rawGyroX); Serial.print("\t");
  Serial.print("rawGyroY: "); Serial.print(rawGyroY); Serial.print("\t"); 
  Serial.print("rawGyroZ: "); Serial.print(rawGyroZ); Serial.println("\t");
}
