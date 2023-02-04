#include "driver.h"
#include <vector>
using namespace std;

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

Robot::Robot(int car_width,int car_height){
    this->car_width = car_width;
    this->car_height = car_height;
    this->model_matrix = {{-1,1,(this->car_width+this->car_height)},{1,1,-1*(this->car_width+this->car_height)},{-1,1, -1*(this->car_width+this->car_height)},{1,1,(this->car_width+this->car_height)}}
}

vector<vector<float> > Robot::runModel(float Vx,float Vy, float W){ // Vx - velocity in x direction; Vy - velocity in y direction; w - angular velocity of robot
    float state_vector[3] = {Vx,Vy,W};
    float result[4][1] = matMul(state_vector);
    return result;
}


vector<vector<float> > Robot::matMul(float state_vector[3]){
    vector<vector<float> > result[4][1];
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

