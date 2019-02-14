/*
  LED Flick
  Welcome to robotics
  
  By Society of Robotics and Automation, VJTI
*/

/** Put your setup code here, to run once 
 *  setup() is called automatically once every time bot is reset or started.
 *  It is usually used to do all initialization.
 */
void setup() {          
  //Port Initialization - DDRX controls the input/output status of port X. 
  DDRC  = 0b11111100;     //C7 C6 C5 C4 C3 C2 are declared as output and C1 C0 are declared as input.
  DDRB  = 0b00000011;     //B0 B1 are declared as output and B7 B6 B5 B4 B3 B2 are declared as input.
}

/** Put your main code here which you want to run repeatedly
 *  loop() is function which is called again and again infinitely.
 *  It is called automatically after the setup() function finishes execution.
 */
void loop() {  
  ledFlick();             // The function ledFlick() is called and since it is in loop() it repeats infinitely. 
}

/** This function turns all the LEDs on and then off only once. 
 */
void ledFlick()
{
  PORTC |= 0b11111100;     //C7 C6 C5 C4 C3 C2 are set high(on). Whereas C1 C0 are as it is.    
  PORTB |= 0b00000011;     //B0 B1 are set high(on). Whereas B7 B6 B5 B4 B3 B2 are as it is.
  delay(100);              //100ms delay so that the flick is visible.
  PORTC &= 0b00000011;     //C7 C6 C5 C4 C3 C2 are set low(off). Whereas C1 C0 are as it is.
  PORTB &= 0b11111100;     //B0 B1 are set low(off). Whereas B7 B6 B5 B4 B3 B2 are as it is.
  delay(100);  
}
