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

Robot::Robot(int car_width,int car_height, Motor& motor_rl, Motor& motor_rr, Motor& motor_fl,Motor& motor_fr){
    this->car_width = car_width;
    this->car_height = car_height;
    this->motor_rl = &motor_rl;
    this->motor_rr = &motor_rr;
    this->motor_fl = &motor_fl;
    this->motor_fr = &motor_fr;
}

void Robot::runModel(float Vx,float Vy, float W){ // Vx - velocity in x direction; Vy - velocity in y direction; w - angular velocity of robot
    float state_vector[3] = {W,Vx,Vy};
    float result[4][1];
    float model_mat[4][3] = {{-1,1,(this->car_width+this->car_height)},{1,1,-1*(this->car_width+this->car_height)},{-1,1, -1*(this->car_width+this->car_height)},{1,1,(this->car_width+this->car_height)}};
    for(int i=0;i<4;i++){
        for(int j=0;j<1;j++){
            result[i][j] =0;
            for(int k=0;k<3;k++){
                result[i][j] += model_mat[i][k]*state_vector[k];
             }
        }
    }
    if(result[0][0]>0)
      this->motor_fr->setSpeed(1,this->multiplier*result[0][0]);
    else
      this->motor_fr->setSpeed(0,this->multiplier*result[0][0]*(-1));
    if(result[1][0]>0)
      this->motor_fl->setSpeed(1,this->multiplier*result[1][0]);
    else
      this->motor_fl->setSpeed(0,this->multiplier*result[1][0]*(-1));
    if(result[2][0]>0)
      this->motor_rl->setSpeed(1,this->multiplier*result[2][0]);
    else
      this->motor_rl->setSpeed(0,this->multiplier*result[2][0]*(-1));
    if(result[3][0]>0)
      this->motor_rr->setSpeed(1,this->multiplier*result[3][0]);
    else
      this->motor_rr->setSpeed(0,this->multiplier*result[3][0]*(-1));
    

    return;
}

