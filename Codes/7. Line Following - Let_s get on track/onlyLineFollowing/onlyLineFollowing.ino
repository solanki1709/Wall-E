
/*
  Line Following
  Reads values of the sensors and calculating the deviation from the line and moving the bot accordingly using PID
 
  By Society of Robotics and Automation, VJTI
*/

#include <SRA16.h>

/*
 * Line Following PID Constants
 */
#define YKp 5
#define YKi 0
#define YKd 0

/*
 * Motor value constraints
 */
float lowerPWMConstrain = 185;
float higherPWMConstrain = 399;
float leftPWM = 0, rightPWM = 0;

/*
 * Line Following PID Variables
 */
float yawAngle = 0, yawError, prevYawError, yawDifference, yawCumulativeError, yawCorrection; 
float yawKp = YKp;
float yawKi = YKi;
float yawKd = YKd;

int sensorRaw[4], sensorVal[4];

void setup()
{
  Serial.begin(9600);

  DDRC |= 0b11111100;
  
  bot_motion_init();
  adc_init();

  bot_forward();
  set_pwm1a(0);
  set_pwm1b(0);
  
  PORTC |= 0b11111100;
  delay(50);
  PORTC &= 0b00000011;
  delay(50);
  PORTC |= 0b11111100;
  delay(50);
  PORTC &= 0b00000011;
  delay(50);
  PORTC |= 0b11111100;
  delay(50);
  PORTC &= 0b00000011;
  delay(50);
  PORTC |= 0b11111100;
  delay(50);
  PORTC &= 0b00000011;
  delay(50);
}

void loop()
{
  readSensors();            //Read sensor values
  calculateSensorValues();  //Remapping sensor values to 0(Black)-1000(White)
  calculateYawAngle();      //Calculating weighted average
  calculateYawError();      //Calculating the correction required to follow the line
  bot_forward();
  adjustBot();              //Feed the corrected values to the motors
}

void calculateYawError()
{
  yawError = yawAngle*0.01;
  yawDifference = yawError - prevYawError;
  yawCumulativeError += yawError;
  if(yawCumulativeError>30)
    yawCumulativeError = 30;
  else if(yawCumulativeError<-30)
    yawCumulativeError = -30;
  yawCorrection = yawKp*yawError + yawKi*yawCumulativeError + yawKd*yawDifference;
  prevYawError = yawError;
}

void adjustBot()
{
  leftPWM = constrain(abs(250 - yawCorrection), lowerPWMConstrain, higherPWMConstrain);
  rightPWM = constrain(abs(250 + yawCorrection), lowerPWMConstrain, higherPWMConstrain);
  
  leftMotorSpeed(leftPWM);
  rightMotorSpeed(rightPWM); 
}

void leftMotorSpeed(int a)
{
  set_pwm1a(a);
}
void rightMotorSpeed(int b)
{
  set_pwm1b(b);
}

void readSensors()
{
  for (int j = 0; j < 4; j++)
    sensorRaw[j] = adc_start(j);
}

void calculateSensorValues()
{
  for (int j = 0; j < 4; j++)
  {
    sensorVal[j] = map(sensorRaw[j], 20, 150, 1000, 0);
    if (sensorVal[j] < 0)
      sensorVal[j] = 0;
    else if (sensorVal[j] > 1000)
      sensorVal[j] = 1000;
  }
}

void calculateYawAngle()
{
  int allBlackFlag = 1;
  unsigned long weightedSum = 0, sum = 0, pos = (4 - 1) * 1000.0;
  for (int j = 0; j < 4; j++)
  {
    if (sensorVal[j] > 400)
      allBlackFlag = 0;
    if (sensorVal[j] > 150)
    {
      weightedSum += (long)(sensorVal[j]) * ((j) * 1000);
      sum += sensorVal[j];
    }
  }
  if (sum != 0)
  {
    pos = weightedSum / sum;
  }

  if(allBlackFlag == 1)
  {
    if(yawAngle > 0)
      pos = (4 - 1) * 1000.0;
    else
      pos = 0;
  }
  yawAngle = pos - (4 - 1) * 500.0;
}

void printData()
{
  Serial.println();
}
