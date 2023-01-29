#include "driver.h"

Motor::Motor(int DIR, int EN){
    this->DIR = DIR;
    this->EN = EN;
    pinMode(DIR, OUTPUT);
    pinMode(EN, OUTPUT);
    digitalWrite(DIR, LOW);
    digitalWrite(EN, LOW);
}

void Motor::setSpeed(int direction, int speed){
    speed = min(speed,255);
    speed = max(speed,0);
    digitalWrite(this->DIR, direction);
    analogWrite(this->EN,speed);

}


