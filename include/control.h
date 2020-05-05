#ifndef Control_h
#define Control_h
#include "Arduino.h"

class Control{

  public:
  
  float compensateVolumeError(float setPoint,float measured);
  void setConstants(float kp,float ki, float kd,double coeff[4],float deadBand);
  void resetController();
  
    
  public:

  float kP;
  float kI;
  float kD;

  float volDeadBand; //ml
  double volEqCoeff[4];
  float discreteIntegral;
  float oldError;
  float oldPredicatedSteps;
  float oldValuePredicted;
  float stableIntegral;
  float error;
};




#endif //Control_h
