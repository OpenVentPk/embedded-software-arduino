#include "control.h"

float Control::compensateVolumeError(float setPoint,float measured){

  error=setPoint-measured;
  float absError=fabs(error);
  float valuePredicted=0;


      if(absError>volDeadBand){
    discreteIntegral=0;
     // return oldValuePredicted;
      }
      else{
      discreteIntegral+=error;      
      }

      valuePredicted=kP*error+kI*discreteIntegral; //Weight of Error will reduce with Kp reaching its point + Weight of Error increase by time to reach goal with Ki.

      valuePredicted=valuePredicted+oldValuePredicted; //Total Steps added/subtracted From begining of cycle.

      oldValuePredicted=valuePredicted;

      return oldValuePredicted;
}

void Control::setConstants(float kp,float ki, float kd,double coeff[4],float deadBand){

     kP=kp;
     kI=ki;
     kD=kd;

     for(int i=0;i<4;i++)
     volEqCoeff[i]=coeff[i];
     volDeadBand=deadBand;

      oldValuePredicted=0;
      oldPredicatedSteps=0;
}

void Control::resetController(){
      discreteIntegral=0;

}
