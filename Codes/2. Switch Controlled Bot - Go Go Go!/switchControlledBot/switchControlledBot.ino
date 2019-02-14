/*
  Switch Controlled Bot
  An Introductory code to test motors using switches connected on SRA board using
  a L298 motor driver with input terminals connected on C4,C5 and C6,C7 and PWM pins
  on D4,D5 pins of Atmega16.
  And the switches are connected on pin D0,D1,D2,D3 of the Atmega 16
  
  By Society of Robotics and Automation, VJTI
*/

#include <SRA16.h>      //The SRA16.h header file is included allowing us to use the library functions

/** Put your setup code here, to run once
 */
void setup() {
  pwm1_init();
  port_init();          //Initialization of all ports
  bot_motion_init();    //Initialization of the motor pins as output pins
  switch_init();        //Initialization of switch D3 D2 D1 D0 as input
  set_pwm1a(399);
  set_pwm1b(399);
}

/** Put your main code here which you want to run repeatedly
 */
void loop() {
  if (pressed_switch0())                //checks whether switch D0 is pressed and returns 1 if pressed else 0
  {
    bot_forward();                      //bot's direction is forward
  }
  else if (pressed_switch1())           //checks whether switch D1 is pressed and returns 1 if pressed else 0
  {
    bot_backward();                     //bot's direction is backward
  }
  else if (pressed_switch2())           //checks whether switch D2 is pressed and returns 1 if pressed else 0
  {
    bot_spot_left();                    //bot's direction is spot left
  }
  else if (pressed_switch3())           //checks whether switch D3 is pressed and returns 1 if pressed else 0
  {
    bot_spot_right();                   //bot's direction is spot right
  }
  else
  {
    bot_stop();
  }
}
