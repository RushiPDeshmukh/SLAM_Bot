#ifndef _DRIVER_H_
#include <Arduino.h>
#include <vector>
using namespace std;
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
    const int car_width;
    const int car_height;
    vector<vector<float> > model_matrix[4][3];

  public:
    Robot(int car_width,int car_height);
    void runModel(int Vx,int Vy,int W);
    vector<vector<float> > matMul(float state_vector[3]);
};


#endif