#ifndef Control_h
#define Control_h
#include "Arduino.h"

class Control{

  public:
  
  float compensateError(float setPoint,float measured);
  void  setConstants(float kp,float ki,float bandIntegral,float eLimit); 
  void resetController(float sP);
  
  float valuePredicted;
  float error;  
  private:

  float kP;
  float kI;

  float integralStartBand;
  float discreteIntegral;
  float errorLimit;
  float selectedSetPoint;


};




#endif //Control_h
