/*
  ADC Testing
  Reads Analog value of the Sensor connected on the PIN 0,of PORT A, of the SRA Development Board and displays the 
  value on the SERIAL MONITOR .PORT A is the Analog Port on Atmega 16 and has 8 ADC pins from 0 to 7.
 
  By Society of Robotics and Automation, VJTI
*/

#include <SRA16.h>      //The SRA16.h header file is included allowing us to use the library functions

int sensorData[4];      //An array of type int to store the data returned by the sensor

/** Put your setup code here, to run once
 */
void setup() {
  Serial.begin(9600);   //Begin Serial Communication at mentioned baud rate (bits per second)
  adc_init();           //Initialization of ADC registers
}

/** Put your main code here which you want to run repeatedly
 */
void loop() {
  readSensors();
  printData();
}

/** Reads the values of the sensors
 */
void readSensors()
{
  for (int i = 0; i < 4; i++)
  {
    sensorData[i] = adc_start(i);   //The function returns the value returned by the sensor connected to pin i of port A (i.e. A0 A1 A2 A3)
  }
}

/** Prints the sensorData on the serial monitor
 */
void printData()
{
  for(int i = 0; i < 4; i++)
  {
    Serial.print(sensorData[i]); Serial.print("\t");  // "\t" gives a tab space
  }
  Serial.println();   //Newline
}
