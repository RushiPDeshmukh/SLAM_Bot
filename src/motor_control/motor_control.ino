#include "driver.h"

/*
3,4 Digital 
5,6 PWM 
7,8 Digital
9,10 PWM


------- | DIR | EN
Motor 1 | 3   | 5   Rear Left 
Motor 2 | 4   | 6   Rear Right
Motor 3 | 7   | 9   Front Left
Motor 4 | 8   | 10  Front Right

*/

Motor motor_rl(3,5);
Motor motor_rr(4,6);
Motor motor_fl(7,9);
Motor motor_fr(8,10);


void setup() {
}

void loop() {
  //Front
  motor_rl.setSpeed(1,100);
  motor_rr.setSpeed(1,100);
  motor_fl.setSpeed(1,100);
  motor_fr.setSpeed(1,100);
  delay(5000);
  //stop
  motor_rl.setSpeed(0,0);
  motor_rr.setSpeed(0,0);
  motor_fl.setSpeed(1,0);
  motor_fr.setSpeed(1,0);
  delay(5000);
  //Backward
  motor_rl.setSpeed(0,100);
  motor_rr.setSpeed(0,100);
  motor_fl.setSpeed(0,100);
  motor_fr.setSpeed(0,100);
  delay(5000);
  // 45 deg left
  motor_rl.setSpeed(1,100);
  motor_rr.setSpeed(1,0);
  motor_fl.setSpeed(1,0);
  motor_fr.setSpeed(1,100);
  delay(5000);
  // 45 deg right
  motor_rl.setSpeed(1,0);
  motor_rr.setSpeed(1,100);
  motor_fl.setSpeed(1,100);
  motor_fr.setSpeed(1,0);
  delay(5000);
  



  /*
  motor_rr.setSpeed(1,200);
  delay(5000);
  motor_rr.setSpeed(0,200);
  delay(5000);
  
  motor_fl.setSpeed(1,200);
  delay(5000);
  motor_fl.setSpeed(0,200);
  delay(5000);
  
  motor_fr.setSpeed(1,200);
  delay(5000);
  motor_fr.setSpeed(0,200);
  delay(5000);
  
  */
}
