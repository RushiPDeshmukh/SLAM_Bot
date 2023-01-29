#include "driver.h"

/*
3,4 Digital 
5,6 PWM 
7,8 Digital
9,10 PWM

------- | DIR | EN
Motor 1 | 3   | 5
Motor 2 | 4   | 6
Motor 3 | 7   | 9
Motor 4 | 8   | 10

*/

Motor motor_1(3,5);
Motor motor_2(4,6);
Motor motor_3(7,9);
Motor motor_4(8,10);


void setup() {
}

void loop() {
  
  motor_1.setSpeed(1,200);
  motor_2.setSpeed(1,200);
  motor_3.setSpeed(1,200);
  motor_4.setSpeed(1,200);
  delay(5000);
  motor_1.setSpeed(0,200);
  motor_2.setSpeed(0,200);
  motor_3.setSpeed(0,200);
  motor_4.setSpeed(0,200);
  delay(5000);
  /*
  motor_2.setSpeed(1,200);
  delay(5000);
  motor_2.setSpeed(0,200);
  delay(5000);
  
  motor_3.setSpeed(1,200);
  delay(5000);
  motor_3.setSpeed(0,200);
  delay(5000);
  
  motor_4.setSpeed(1,200);
  delay(5000);
  motor_4.setSpeed(0,200);
  delay(5000);
  
  */
}
