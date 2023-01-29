#ifndef _DRIVER_H
#include <Arduino.h>
class Motor{
    private:
      int DIR;
      int EN;
      
    public:
      Motor(int DIR,int EN);
      void setSpeed(int direction,int speed);
};

#endif