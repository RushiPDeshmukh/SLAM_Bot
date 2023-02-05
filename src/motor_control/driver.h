

#ifndef _DRIVER_H_
#include <Arduino.h>


class Motor{
    private:
      int DIR;
      int EN;
      
    public:
      Motor(int DIR,int EN);
      void setSpeed(int direction,int speed);
      
};
class Robot{
  private:
   int car_width;
   int car_height;
   Motor* motor_rl;
   Motor* motor_rr;
   Motor* motor_fl;
   Motor* motor_fr;

  public:
    Robot(int car_width,int car_height, Motor& motor_rl, Motor& motor_rr, Motor& motor_fl,Motor& motor_fr);
    void runModel(float Vx,float Vy,float W);
};


#endif