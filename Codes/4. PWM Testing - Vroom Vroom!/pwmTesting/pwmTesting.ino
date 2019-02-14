/*
  PWM Testing
  Varies the voltage sent to the motor, by changing the PWM value, 0-399 will be mapped to 0-5V on PWM pin
  
  By Society of Robotics and Automation, VJTI
*/

#include <SRA16.h>

/** Put your setup code here, to run once
 */
void setup() {
  pwm1_init();            //PWM pins of the atmega are initialized
  bot_motion_init();      //Bot motion is initialized
  switch_init();          //Switch can be accessed
  bot_forward();          //Bot's direction is set to forward
  Serial.begin(9600);
}

/** Put your main code here which you want to run repeatedly
 */
void loop() {
  if (pressed_switch1())  //checks whether the switch is pressed and returns 1 if pressed else 0
  {
    while (1)
    {
      for (int i = 399; i > 99; i = i - 20)
      {
        set_pwm1a(i);     //PWM is set to value i for motor 1a
        set_pwm1b(i);     //PWM is set to value i for motor 1b
        delay(500);
        Serial.println(i);
      }
    }
  }
}
