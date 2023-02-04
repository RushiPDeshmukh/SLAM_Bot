#include "driver.h"

Motor::Motor(int DIR, int EN, int car_widht, int car_height){
    this->DIR = DIR;
    this->EN = EN;
    this->car_height = car_height;
    this->car_width = car_width;
    this->model_matrix ={{-1,1,(this->car_width+this->car_height)},{1,1,-1*(this->car_width+this->car_height)},{-1,1, -1*(this->car_width+this->car_height)},{1,1,(this->car_width+this->car_height)}}
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

float[4][3] matMul(float state_vector[3]){
    float result[4][1];
    for(int i=0;i<4;i++){
        for(int j=0;j<1;j++){
            result[i][j] =0;
            for(int k=0;k<3;k++){
                result[i][j] += this->model_matrix[i][k]*state_vector[k];
             }
        }
    }
    return result;
}
void Motor::runModel(float Vx,float Vy, float W){
    float state_vector[3] = {Vx,Vy,W};
    float result[4][1] = matMul(state_vector);
    
}

