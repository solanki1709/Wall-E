/*
  Self Balancing with Line Following
  The Self Balancing and Line Following codes are combined and both the yaw and the pitch corrections are calculated
  and the motor values are set accordingly.
  
  By Society of Robotics and Automation, VJTI
*/

#include <MPU.h>
#include <FILTER.h>
#include <SRA16.h>

#define PKp 38
#define PKi 5
#define PKd 2

#define YKp 5
#define YKi 0
#define YKd 0

float setpoint = 4.5;
float forwardAngle = setpoint - 2.5;    //To make the bot go forward, the bot is tilted forward. -ve is forward and +ve is backward.

float initialAcceAngle = setpoint;
float initialGyroAngle = 0;

float lowerPWMConstrain = 145;
float higherPWMConstrain = 349;
float leftPWM = 0, rightPWM = 0;

int16_t rawAcceX, rawAcceY, rawAcceZ;
int16_t rawTemp;
int16_t rawGyroX, rawGyroY, rawGyroZ;

float pitchAngle = 0, pitchError, prevPitchError, pitchDifference, pitchCumulativeError, pitchCorrection; 
float pitchKp = PKp;
float pitchKi = PKi;
float pitchKd = PKd;

float yawAngle = 0, yawError, prevYawError, yawDifference, yawCumulativeError, yawCorrection; 
float yawKp = YKp;
float yawKi = YKi;
float yawKd = YKd;

int sensorRaw[4], sensorVal[4];

int pidFlag = 4;
int isBalancingFlag = 0;
int startLineFollowFlag = 0;

void setup()
{
  Serial.begin(9600);

  DDRC |= 0b11111100;
  PORTC = PORTC | 0b00001000 ;
 
  startMPU();

  PORTC = PORTC & 0b11110111 ;

  //calibrateMPU(initialAcceAngle, initialGyroAngle);
  initialAcceAngle = setpoint ;
  initialGyroAngle = 0 ;
  
  bot_motion_init();
  switch_init();
  adc_init();

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
  readRawValuesOfMPU(rawAcceX, rawAcceY, rawAcceZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ);
  calculatePitchAngle(pitchAngle, rawAcceX, rawAcceZ, rawGyroY, initialAcceAngle, initialGyroAngle);
  calculatePitchError();

  readSensors();
  calculateSensorValues();
  calculateYawAngle();
  calculateYawError();

 /** Tilt the bot forward only if the bot is somewhat balanced. Otherwise do not do the yawCorrection i.e. only do self balancing.
 */
  if(isBalancingFlag && startLineFollowFlag)
  {
    PORTC &= 0b11111011;
    PORTC |= 0b00001000; 
    initialAcceAngle = forwardAngle;
    if(-1.5 < pitchAngle && pitchAngle < 1.5)
      isBalancingFlag = 1;
    else 
      isBalancingFlag = 0;
  }
  else
  {
    PORTC &= 0b11110111;
    PORTC |= 0b00000100;
    initialAcceAngle = setpoint;
    yawCorrection = 0;
    if(-2.5 < pitchAngle && pitchAngle < 2.5)
      isBalancingFlag = 1;
    else 
      isBalancingFlag = 0;
  }

  balance();
  
  if (pressed_switch1())          //press D1 to start or stop line following
  {
    if(startLineFollowFlag)
      startLineFollowFlag = 0;
    else 
      startLineFollowFlag = 1;
    while (pressed_switch1());  
  }
  
  if (pressed_switch0()) {
    if (pidFlag == 1)
      pidFlag = 2;
    else if (pidFlag == 2)
      pidFlag = 3;
    else if (pidFlag == 3)
      pidFlag = 4;
    else if (pidFlag == 4)
      pidFlag = 1;
    while (pressed_switch0());
  }

  if (pressed_switch2()) {
    if (pidFlag == 1)
      pitchKp += 5;
    else if (pidFlag == 2)
      pitchKi += 5;
    else if (pidFlag == 3)
      pitchKd += 5;
    else if (pidFlag == 4)
    {
      setpoint += 0.5;
      initialAcceAngle = setpoint ;
    }
    while (pressed_switch2());
  }

  if (pressed_switch3()) {
    if (pidFlag == 1 && pitchKp >= 5)
      pitchKp -= 5;
    else if (pidFlag == 2 && pitchKi >= 5)
      pitchKi -= 5;
    else if (pidFlag == 3 && pitchKd >= 5)
      pitchKd -= 5;
    else if (pidFlag == 4)
    {
      setpoint -= 0.560;
      initialAcceAngle = setpoint ;
    }
    while (pressed_switch3());
  }
  // printData();
}

void calculatePitchError()
{
  pitchError = pitchAngle;                                                
  pitchDifference = pitchError - prevPitchError;
  pitchCumulativeError += pitchError;
  if(pitchCumulativeError>40)
    pitchCumulativeError = 40;
  else if(pitchCumulativeError<-40)
    pitchCumulativeError = -40;
  pitchCorrection = pitchKp * pitchError + pitchKi * pitchCumulativeError + pitchKd * pitchDifference;
  prevPitchError = pitchError;
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

void balance()
{
  long absolutePitchCorrection = abs(pitchCorrection);
  
  if (pitchAngle > 0)
    bot_backward();
  else if (pitchAngle < 0)
    bot_forward();
  else bot_brake();

  /** Doing both the corrections together
   */
  leftPWM = constrain(abs(absolutePitchCorrection - yawCorrection), lowerPWMConstrain, higherPWMConstrain);
  rightPWM = constrain(abs(absolutePitchCorrection + yawCorrection), lowerPWMConstrain, higherPWMConstrain);

  /* If the correction is extreme, take a sharper turn. This helps in following the line at the time of sharp turns.
   */
  if(yawAngle>1200 && startLineFollowFlag)
  {  
    leftMotorSpeed(150);
    rightMotorSpeed(rightPWM - 20);
  }
  else if(yawAngle<-1200 && startLineFollowFlag)
  {  
    leftMotorSpeed(leftPWM - 20);
    rightMotorSpeed(150);
  }
  else
  {
    leftMotorSpeed(leftPWM);
    rightMotorSpeed(rightPWM);
  }
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
    sensorVal[j] = map(sensorRaw[j], 20, 160, 1000, 0);
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
  Serial.println(pitchAngle);
}
