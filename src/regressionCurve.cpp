#include "header.h"
#include "curveFitting.h"

//#define ALTERNATE_CALIBRATION_SCHEME

extern struct Slave slave;
extern struct TidalVolume TV;
extern struct P_Sensor p_sensor;
extern int Homing_Done_F;

byte calibStatus = ST_COMPLETE;//ST_NOT_INIT;//ST_COMPLETE;//
byte estimateVolume = false;

double VolCoeffs[ORDER+1];
double PressCoeffs[ORDER_PRESS_EQ+1];


void calibrate(int calibParam){

  unsigned int period = 1000; //us
 
  static double steps[(STEPPERRANGE/stepSize)+1]; //mm
  static double volume[(STEPPERRANGE/stepSize)+1]; 
  static double pressureArray[(STEPPERRANGE/stepSize)+1]; 

  static unsigned long reqMotorSteps = 0;
  static int i = stepSize;
  static int j = 1;
  static bool init = true;

  if (init)
  {
    slave.runAck = 0;
    slave.homeAck = 0;
    slave.lastCMD_ID = HOME;
    steps[0] = 0.0;
    volume[0] = 0.0;
    pressureArray[0] = p_sensor.pressure_gauge_CM;
    if (calibParam == VOL_CONT_MODE)
      TV.measured = 0.0;
    Serial.println("Inside Calibration Routine");
    delay(5000);//for testing only
    init = false;    
  }

  //Load Data
  if (calibStatus == ST_IN_PROG) {
    if (i<=STEPPERRANGE){
      if (slave.runAck == 0 && slave.lastCMD_ID != RUN)
      {
        reqMotorSteps = (long)((i / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV);
        txSlaveCMD(RUN, period, reqMotorSteps, "1"); //Move Forward
        slave.lastCMD_ID = RUN;
        #ifndef ALTERNATE_CALIBRATION_SCHEME
        if (calibParam == VOL_CONT_MODE)
          TV.measured = 0.0;
        #endif
      }
      else if (slave.runAck == CMD_RECEIVED)
      {
     //   Serial.println("CMD_RECEIVED");
        if (calibParam == VOL_CONT_MODE)
          estimateVolume = true;
      }
      else if (slave.runAck == CMD_COMPLETE && slave.lastCMD_ID == RUN)
      {
        estimateVolume = false;
        steps[j]= (double)(i);
        volume[j]=(double)(TV.measured);
        pressureArray[j] = p_sensor.pressure_gauge_CM;
      #ifndef ALTERNATE_CALIBRATION_SCHEME
        txSlaveCMD(HOME, period);
        slave.lastCMD_ID = HOME;
      #else
        Serial.print("Distance: ");Serial.print(steps[j]);Serial.println("mm");
        if (calibParam == VOL_CONT_MODE) {
          Serial.print("Volume: ");Serial.print(volume[j]);Serial.println("ml"); }
        else {
          Serial.print("Pressure: ");Serial.print(pressureArray[j]);Serial.println("cmH2O"); }
        slave.runAck = 0;
        slave.lastCMD_ID = NO_CMD;
        i +=stepSize;
        j++;
      #endif
      }    
      #ifndef ALTERNATE_CALIBRATION_SCHEME
      else if (slave.homeAck == 2)
      {
        slave.runAck = 0;
        slave.homeAck = 0;
        if (calibParam == VOL_CONT_MODE)
          TV.measured = 0.0;
        Serial.print("Distance: ");Serial.print(steps[j]);Serial.println("mm");
        if (calibParam == VOL_CONT_MODE) {
          Serial.print("Volume: ");Serial.print(volume[j]);Serial.println("ml"); }
        else {
          Serial.print("Pressure: ");Serial.print(pressureArray[j]);Serial.println("cmH2O"); }
        i +=stepSize;
        j++;
      }
      #endif
    }
    else
    {
      #ifndef ALTERNATE_CALIBRATION_SCHEME
      calibStatus = ST_COMPLETE;
      #else
      if (slave.lastCMD_ID != HOME)
      {
        txSlaveCMD(HOME, period);
        slave.lastCMD_ID = HOME;       
      }
      else if (slave.homeAck == 2)
      {
        calibStatus = ST_COMPLETE;
      }
      #endif
    }
  }
  
  if (calibStatus == ST_COMPLETE)
  {
    int ret;
    if (calibParam == VOL_CONT_MODE) {
      ret = fitCurve(ORDER, sizeof(volume)/sizeof(double), volume, steps, sizeof(VolCoeffs)/sizeof(double), VolCoeffs);

      //Highest to lowest for 3rd order y=ax^2+bx+c where x is volume and y is step in mm. for 3rd order equation. 
      if (ret == 0){ //Returned value is 0 if no error
        
        #ifdef E2PROM
        int eeAddress = ee_Vol_Coef_a;
        EEPROM.put(eeAddress, VolCoeffs);
  //      eeAddress += sizeof(VolCoeffs);
        #endif
      Serial.println("Highest to lowest for 3rd order y=ax^2+bx+c where x is volume and y is step in mm. for 3rd order equation");
        for (int k = 0; k < sizeof(VolCoeffs)/sizeof(double); k++){
          Serial.print(VolCoeffs[k], 8);
          Serial.print('\t');
        }
  //      delay(20000);//for testing only
      }
    }
    else {
      ret = fitCurve(ORDER_PRESS_EQ, sizeof(pressureArray)/sizeof(double), pressureArray, steps, sizeof(PressCoeffs)/sizeof(double), PressCoeffs);
      //Highest to lowest for 3rd order y=ax^2+bx+c where x is Pressure and y is step in mm. for 3rd order equation. 
      if (ret == 0){ //Returned value is 0 if no error
        
        #ifdef E2PROM
        int eeAddress = ee_Press_Coef_a;
        EEPROM.put(eeAddress, PressCoeffs);
  //      eeAddress += sizeof(PressCoeffs);
        #endif
      Serial.println("Highest to lowest for 3rd order y=ax^2+bx+c where x is Pressure and y is step in mm. for 3rd order equation");
        for (int k = 0; k < sizeof(PressCoeffs)/sizeof(double); k++){
          Serial.print(PressCoeffs[k], 8);
          Serial.print('\t');
        }
        delay(20000);//for testing only
      }
    }
  }
}