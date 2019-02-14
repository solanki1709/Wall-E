/*
  Self Balancing
  Taking the angle from the MPU and calculating the deviation from the set point, to move the bot accordingly.
 
  By Society of Robotics and Automation, VJTI
*/

#include <MPU.h>
#include "FILTER.h"
#include <SRA16.h>

/*
 * Self Balancing PID Constants
 */
#define PKp 35
#define PKi 5
#define PKd 2

float setpoint = 6;   //This is the MPU angle rest(Vertical) position of the bot

float initialAcceAngle = setpoint;
float initialGyroAngle = 0;

float lowerPWMConstrain = 145;
float higherPWMConstrain = 399;
float leftPWM = 0, rightPWM = 0;

int16_t rawAcceX, rawAcceY, rawAcceZ;
int16_t rawTemp;
int16_t rawGyroX, rawGyroY, rawGyroZ;

/*
 * Self Balancing PID Variable
 */
float pitchAngle = 0, pitchError, prevPitchError, pitchDifference, pitchCumulativeError, pitchCorrection; 
float pitchKp = PKp;
float pitchKi = PKi;
float pitchKd = PKd;


int pidFlag = 1;      //To adjust the values of P,I,D and setpoint on the go 

void setup()
{
  Serial.begin(9600);

  DDRC |= 0b11111100;
  set_pwm1a(0);
  set_pwm1b(0);
  
  PORTC = PORTC | 0b00001000 ;

  startMPU();

  PORTC = PORTC & 0b11110111 ;

  //calibrateMPU(initialAcceAngle, &initialGyroAngle);      //This function sets the setpoint as the MPU angle when you start the bot.
  initialAcceAngle = setpoint ;
  initialGyroAngle = 0 ;
  
  bot_motion_init();
  switch_init();
  
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
  balance();
    
  if (pressed_switch0()) {
    if (pidFlag == 1)
      pidFlag = 2;                  //pidFlag = 2 means now you can change I
    else if (pidFlag == 2)
      pidFlag = 3;                  //pidFlag = 3 means now you can change D
    else if (pidFlag == 3)
      pidFlag = 4;                  //pidFlag = 4 means now you can change setpoint
    else if (pidFlag == 4)
      pidFlag = 1;                  //pidFlag = 1 means now you can change P
    while (pressed_switch0());      //This while loop is for debouncing
  }

  if (pressed_switch2()) {          //By pressing switch D2, you can increase the value
    if (pidFlag == 1)
      pitchKp += 5;
    else if (pidFlag == 2)
      pitchKi += 5;
    else if (pidFlag == 3)
      pitchKd += 5;
    else if (pidFlag == 4)
    {
      setpoint += 1;
      initialAcceAngle = setpoint ;
    }
    while (pressed_switch2());
  }

  if (pressed_switch3()) {          //By pressing switch D3, you can decrease the value
    if (pidFlag == 1 && pitchKp >= 5)
      pitchKp -= 5;
    else if (pidFlag == 2 && pitchKi >= 5)
      pitchKi -= 5;
    else if (pidFlag == 3 && pitchKd >= 5)
      pitchKd -= 5;
    else if (pidFlag == 4)
    {
      setpoint -= 1;
      initialAcceAngle = setpoint ;
    }
    while (pressed_switch3());
  }
  //printData();
}

/** Using the PID algorithm to calculate the correction
 */
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

/** Setting the direction of the motor according to the pitchAngle with a little buffer in the middle. Then setting the PWM of both the motors.
 */
void balance()
{
  long absolutePitchCorrection = abs(pitchCorrection);
  
  if (pitchAngle > 0.2)
    bot_backward();
  else if (pitchAngle < -0.2)
    bot_forward();
  else bot_brake();

  leftPWM = constrain(absolutePitchCorrection, lowerPWMConstrain, higherPWMConstrain);
  rightPWM = constrain(absolutePitchCorrection, lowerPWMConstrain, higherPWMConstrain);
  
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

void printData()
{
  Serial.println();
}
