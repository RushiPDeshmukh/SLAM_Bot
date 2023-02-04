#ifndef _DRIVER_H
#include <Arduino.h>
class Motor{
    private:
      int DIR;
      int EN;
      const int car_width;
      const int car_height;
      int model_matrix[4][3];
      
    public:
      Motor(int DIR,int EN, int car_width,int car_height);
      void setSpeed(int direction,int speed);
      void runModel(int Vx,int Vy,int W)
};

#endif