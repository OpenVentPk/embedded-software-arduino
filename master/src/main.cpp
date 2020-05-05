/*
 * All the #define are in Plateformio.ini file with -D name 
  OpenVentPK Prototype 1 OVPD-1 - Source Code
    //TODO:
        ADD Version History Here
        In Future;
        Implement Watchdog
        Address Wire.h long reported bug
        Implement Assist Control
        Implement Pressure Controlled Mode
        Interface O2 Sensor
        Interface Flow Rate Sensor

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Author: OpenVentPK.
    Created March 28, 2020
    EMAIL : <sohaib_ashraf@hotamail.com>
*/
#include "header.h"
#include "flowSensor.h"
#include "control.h"
#include "userInterface.h"

//Other Address could be 0x20, 0x27 , 0x3F
// Run I2C_Scanner Script to discover device

#ifdef TX_SERIAL_TELEMETRY
#include "telemetry.h"
extern struct TEL_TYPE TEL;
#endif
 
float plateauPressure, /* Plateau pressure is the pressure that is applied by the ventilator to the small airways and alveoli.
                           It is measured at end-inspiration with an inspiratory hold maneuver.*/
      PEEPressure,       // Positive end-expiratory pressure (PEEP)
      peakInspPressure;      // high pass filtered value of the pressure. Used to detect patient initiated breathing cycles
unsigned long tick1,            // counter used to trigger sensor measurements //loop 1
         tick2;             // counter used to trigger sensor measurements // loop 2
uint16_t breathLength;      // duration of the current breathing cycle in milliseconds. This is 60000/BPM.
         
boolean PeepValid = false;
boolean PltPrsValid = false;

//double CodeVer;
// Parameters saved to / recovered from EEPROM

uint8_t breathPhase = WAIT_PHASE;
uint8_t selfTestProg = ST_NOT_INIT; // Selft Test is Implemented
uint8_t selfTestStatus = ST_PASS;   // Selft Test is Implemented

byte calibrationParam;
extern byte calibStatus;
extern byte estimateVolume;

uint8_t Homing_Done_F = 0;

uint8_t ErrorNumber = 0;
uint8_t devMode = 0;
uint8_t activateVentilatorOperation = 0;

uint8_t WarmUpFlag = 1;
uint8_t DevModeDetectionInProg = 0;
uint8_t PEEPMesaureFlag = 0;

uint8_t CVmode = VOL_CONT_MODE;// CV or CP mode indicator;
uint8_t assistControl = 0;
boolean scanBreathingAttempt = false;
boolean patientTriggeredBreath = false;
boolean holdManeuver = true;
uint16_t holdDur_ms = 150;
//int triggerVariable = FLOW_VAR;
//int cyclingVariable = TIME_VAR;

uint8_t VentilatorOperationON = 0;

uint8_t OkButton = 0;
uint8_t SnoozeButton = 0;

uint8_t pressSnsrInUse = MS4525_IN_USE;

//#ifdef StepGen
uint8_t motorInUse = STEPPER_IN_USE;
//#endif

uint8_t patientWeight = 70; //kg

extern uint8_t IE_R_Value[3][2];

extern double VolCoeffs[ORDER+1];
extern double PressCoeffs[ORDER_PRESS_EQ+1];

extern struct Flow_Sensor FS;

struct setpointStatus setpoint;
struct P_Sensor p_sensor;
struct TidalVolume TV;
struct Slave slave;

Control *control;

#ifdef Beeper
struct Alarm alarm;
#endif

void (*resetFunction)(void) = 0; // Self reset (to be used with watchdog)

boolean checkValues()
{
  boolean isOk = (setpoint.reqBPM >= minBPM);                  // BPM in allowed range ?
  if (setpoint.reqBPM > maxBPM) isOk = false;
  if (setpoint.reqVolume < minVolume) isOk = false;            // Volume in allowed range ?
  if (setpoint.reqVolume > maxVolume) isOk = false;
  if (setpoint.reqPressure < minPressure) isOk = false;  // Compression in allowed range ?
  if (setpoint.reqPressure > maxPressure) isOk = false;
  if (isnan(setpoint.reqBPM)) isOk = false;                    // Check for malformed floating point values (NaN)
  if (isnan(setpoint.reqVolume)) isOk = false;
  if (isnan(setpoint.reqPressure)) isOk = false;

  return isOk;
}

#ifdef Beeper

void beep() // Launch a beep
{
  static unsigned int currentToneFreq = SNOOZE_ALARM;
  static unsigned long t_millis = 0;

#ifdef ActiveBeeper
  if (alarm.action == SNOOZE_ALARM)
  {
    noTone(pin_Beep);
    currentToneFreq = SNOOZE_ALARM;
    alarm.toneFreq = SNOOZE_ALARM;
    alarm.timePeriod = SNOOZE_ALARM;
  }
  else
  {
    if (alarm.toneFreq > currentToneFreq)
    {
      currentToneFreq = alarm.toneFreq; //High Severity Alarm Has priority
      t_millis = 0;
    }

    if ((millis() - t_millis) >= alarm.timePeriod)
    {
      t_millis = millis();
      tone(pin_Beep, currentToneFreq, (int)(alarm.timePeriod / 2)); //Duration in milliseconds
    }    
  }
#endif
}
#endif

void eeput(int n) // records to EEPROM (only if values are validated)
{
#ifdef E2PROM
  int eeAddress = eeStart;
  boolean isOk = checkValues();

  if (n == 1) isOk = true; // override (for debug testing)
  if (isOk)
  {
    EEPROM.put(eeAddress, setpoint.reqBPM);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, setpoint.reqVolume);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, setpoint.reqPressure);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, setpoint.reqI_E_Section);
    eeAddress += sizeof(int);
    EEPROM.put(eeAddress, setpoint.reqFiO2);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, setpoint.flowTriggerSenstivity);
  }
#endif
}

void eeget()
{
#ifdef E2PROM
  int eeAddress = eeStart;
  EEPROM.get(eeAddress, setpoint.reqBPM);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, setpoint.reqVolume);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, setpoint.reqPressure);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, setpoint.reqI_E_Section);
  eeAddress += sizeof(int);
  EEPROM.get(eeAddress, setpoint.reqFiO2);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, setpoint.flowTriggerSenstivity);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, VolCoeffs);
  eeAddress += sizeof(VolCoeffs);
  EEPROM.get(eeAddress, PressCoeffs);
  eeAddress += sizeof(PressCoeffs);
  
  Serial.println("Saved Coefficients:");
  Serial.println("Highest to lowest for 3rd order y=ax^3+bx^2+cx+d where x is volume and y is step in mm. for 3rd order equation");
  for (int i = 0; i <= ORDER; i++)
  {
        Serial.print(VolCoeffs[i], 5);
        Serial.print('\t');
  }
  Serial.println();

  delay(20000);//for testing only
#else
  setpoint.reqBPM = defaultBPM;
  setpoint.reqVolume = defaultVolume;
  setpoint.reqPressure = defaultPressure;
  setpoint.reqI_E_Section = defaultExpirationRatioIndex;
  setpoint.reqFiO2        = 60;
  setpoint.flowTriggerSenstivity = 0.5;
//  Serial.print("Read Default Settings\n");  //Arduino gets stuck if comment this line
#endif
}

void Timer1ISR()
{
}

//Self Test and Auto Calibrate Routines
void selfTest()
{
  static uint8_t ctr = 0;
  ErrorNumber = 0;
  selfTestStatus = ST_PASS;
  selfTestProg   = ST_IN_PROG;

  if (FS.connectionStatus != 0)
  {
    ErrorNumber = FLOW_SENSOR_DISCONNECTED;
    #ifndef TEL_AT_UART0
    Serial.print(F("Flow Sensor Error Code: ")); Serial.println(FS.connectionStatus);  
    #endif
    return;
  }

if (calibStatus != ST_IN_PROG)
{
  Homing_Done_F = slave.homeAck;

  if (Homing_Done_F == 0)
  {    
    if (ctr == 0) { 
      //Serial2.print("#HOME 2000"); //2000us
      txSlaveCMD(HOME, 2000);
      slave.lastCMD_ID = HOME; }
    else {ctr++; if (ctr == (1000/samplePeriod1)) ctr = 0;}        

    ErrorNumber     = HOMING_NOT_DONE_ERROR;
    selfTestStatus  = ST_FAIL;
    return;
  }
  else if (Homing_Done_F == 1)
  {
    ErrorNumber     = HOMING_NOT_DONE_ERROR;
    selfTestStatus  = ST_FAIL;
    return;
  }
  else if (Homing_Done_F == 2)
  {
    //HOMING COMPLETE and SUCCESSFUL
  }
  else if (Homing_Done_F == 3)
  {
    ErrorNumber     = MECH_INTEGRITY_FAILED;
    selfTestStatus  = ST_FAIL;
    return;
  }
}
  if (calibStatus != ST_COMPLETE)
  {
    calibStatus = ST_IN_PROG;
    calibrate(calibrationParam);
    return;
  }
  
/*  if (activateVentilatorOperation == 1)
  {
    //    Serial.println("Self Test FAIL");

    ErrorNumber     = START_SWT_ERROR;
    selfTestStatus  = ST_FAIL;
//    selfTestProg    = ST_COMPLETE;
    return;
  }
  else
  {
    //    Serial.println("Self Test PASS");
    selfTestProg    = ST_COMPLETE;
    return;
  }*/
selfTestProg    = ST_COMPLETE;
return;
}
#ifdef MS4525DO
/*
   used to get values from MS4525 sensor
   and should be called after 10ms
   due to sensor update or refresh rate
*/

inline static void get_sensor_data(uint16_t *raw) {
  Wire.beginTransmission(I2C_ADDRESS_MS4525DO);
  Wire.write(1);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDRESS_MS4525DO, 2); //request for two pressure bytes from sensor
  *raw = (Wire.read() & 0x3F) << 8; // read the msb from the I2C device
  *raw |= Wire.read();//read the lsb from the device
}
#endif

float readVcc() {
  long result; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert while (bit_is_set(ADCSRA,ADSC));
  result = ADCL; result |= ADCH << 8;
  result = 1126400L / result; // Ba,,ck-calculate AVcc in mV
  return result;
}


void voltage_correction(float &diff_press_pa)
{
  const float slope = 65.0f;
  /*
    apply a piecewise linear correction, flattening at 0.5V from 5V
  */
  float voltage_diff = readVcc() - 5.0f;
  if (voltage_diff > 0.5f) {
    voltage_diff = 0.5f;
  }

  if (voltage_diff < -0.5f) {
    voltage_diff = -0.5f;
  }
  diff_press_pa -= voltage_diff * slope;
}

void readSensors() // Read Values from Installed Sensor
{
#if defined (MS4525DO)
  // Calculate differential pressure. As its centered around 8000
  // and can go positive or negative
  uint16_t raw_pressure = 0;
  get_sensor_data(&raw_pressure);

  /*this equation is an inversion of the equation in the
    pressure transfer function figure on page 4 of the datasheet
    We negate the result so that positive differential pressures
    are generated when the bottom port is used as the static
    port on the pitot and top port is used as the dynamic port*/
  p_sensor.diff_press_PSI = -((raw_pressure - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
  p_sensor.diff_press_pa = p_sensor.diff_press_PSI * PSI_to_Pa;

  voltage_correction(p_sensor.diff_press_pa); //Recommended by Hamza
#ifdef MS4525_AS_Gauge_Pressure
  p_sensor.pressure_gauge = p_sensor.diff_press_pa;

  static float oldY = 0.0;
  static float avgP = 0.8;
  float y = p_sensor.pressure_gauge * Pa2cmH2O;
  y = (y * avgP) + ( oldY * (1.0 - avgP));
  oldY = y;
  p_sensor.pressure_gauge_CM = oldY;

  //  p_sensor.pressure_gauge_CM = p_sensor.pressure_gauge * Pa2cmH2O; //Unfiltered

#else
  float diff_press_bar = (p_sensor.diff_press_PSI * 0.0689476);
  //liter/min flow rate 60000 where is K is 520 constant
  p_sensor.q = ( K * diff_press_Bar ) * 60000;
#endif
#endif
 
  //FLOW SENSORS
  #if defined(FLOW_SENSOR_INSTALLED)
  FS.Q_SLM = getFlowValue();
  #endif


}

void Monitoring()
{
  static bool initInsp = true;
  static bool initHold = true;
  static bool initExp = true;
  static bool initMeasurePEEP = true;
  static float minuteVentilationSum = 0.0;

  static unsigned long T_old_us = millis();

  static unsigned long pre_millis_1min = 0;

  if ((millis() - pre_millis_1min) >= 60000)
  {
    pre_millis_1min = millis();
    TV.minuteVentilation = minuteVentilationSum;
    minuteVentilationSum = 0.0;
  }


  float delta_t = ((float)(millis() - T_old_us)); //ms

  if (calibStatus == ST_IN_PROG)
  {
    if (estimateVolume)
    {
      TV.measured += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
    }
  }
  else
  {
  // Plateau Pressure & PEEP and Set Breathing Flags
    switch (breathPhase)
    {
      case INSPIRATION_PHASE:
        if (initInsp) {
          TV.inspiration = 0.0; TV.measured = 0.0;
          peakInspPressure = p_sensor.pressure_gauge_CM;
          initInsp = false;
        }
        TV.inspiration += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
        TV.measured = TV.inspiration;   

        minuteVentilationSum += TV.inspiration;

        if (peakInspPressure < p_sensor.pressure_gauge_CM)
          peakInspPressure = p_sensor.pressure_gauge_CM;


        // reset init Flags
        initHold = true;
        initExp = true;
        initMeasurePEEP = true;
        scanBreathingAttempt = false;
        patientTriggeredBreath = false;
      break;
      case HOLD_PHASE:
        TV.measured += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);

        PltPrsValid = true;
        if (initHold)
        {
          plateauPressure = (p_sensor.pressure_gauge_CM);
          initHold = false;
        }
        else
        {
          plateauPressure = (plateauPressure + (p_sensor.pressure_gauge_CM)) * 0.5;
        }
      
        TV.staticCompliance = (TV.inspiration / (plateauPressure - PEEPressure));

        // reset init Flags
        initInsp = true;
        initExp = true;
        initMeasurePEEP = true;
        scanBreathingAttempt = false;
        patientTriggeredBreath = false;
        break;
      case EXPIRATION_PHASE:
        if (initExp) {TV.expiration = 0.0; initExp = false;}
        TV.expiration += (-1.0) * (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);      
        TV.measured += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);

        if (assistControl == 1) {
          if (scanBreathingAttempt)
          {
            if (FS.Q_SLM >= setpoint.flowTriggerSenstivity) {
            patientTriggeredBreath = true; }          
          }
        }
        if (PEEPMesaureFlag == 1)
        {
          PeepValid = true;
          if (initMeasurePEEP)
          {
            PEEPressure = (p_sensor.pressure_gauge_CM);
            initMeasurePEEP = false;
          }
          else
          {
            PEEPressure = (PEEPressure + (p_sensor.pressure_gauge_CM)) * 0.5;
          }
  //        TV.staticCompliance = (TV.inspiration / (plateauPressure - PEEPressure));
        }

        // reset init Flags
        initInsp = true;
        initHold = true;
        break;
      default: //WAIT PHASE
        // reset init Flags
        initInsp = true;
        initHold = true;
        initExp = true;
        initMeasurePEEP = true;
        scanBreathingAttempt = false;
        patientTriggeredBreath = false;
        break;
    }
  }

  T_old_us = millis();

#ifdef QT_PLOTTER
  Serial.print("$");
  Serial.print(FS.Q_SLM, 5);
  Serial.print(" ");
  Serial.print(TV.measured, 5);
  Serial.print(" ");
//  Serial.print(TV.inspiration, 5);
//  Serial.print(" ");
//  Serial.print(TV.expiration, 5);
//  Serial.print(" ");
  Serial.print(p_sensor.pressure_gauge_CM, 5);

  Serial.print(" ");
  Serial.print(control->valuePredicted,5);
  Serial.print(" ");
  Serial.print(control->error, 5);
  // Serial.print(" ");
  // Serial.print(breathPhase);
  // Serial.print(" ");
  // Serial.print(delta_t, 5);
  Serial.print(";\n");
#endif


}


/*
     This is the main Alarm Control.
        Sensor Monitoring.
        Alarm Triggering.
*/
void alarmControl() // Read Values from Installed Sensor
{

  #define COUNT_10ms  10/samplePeriod1
  #define COUNT_50ms  50/samplePeriod1
  #define COUNT_100ms 100/samplePeriod1

  //1 count = 10ms
  static uint8_t HighPeakPressAlarmCnt = 0;
  static uint8_t HighPltPressAlarmCnt = 0;
  static uint8_t LowPEEPAlarmCnt = 0;

  //ADD Power Related Alarms here
  // Low Battery
  // Battery Power In Use  

  if ((VentilatorOperationON == 1) && (breathPhase != WAIT_PHASE)) //Ventilation Related Alarms
  {
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////FAULT DETECTION////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////


    if (p_sensor.pressure_gauge_CM > (setpoint.curPressure)) //Pressure Setpoint in Control Loop (User Input Based)
    {
      HighPeakPressAlarmCnt++;
    } else HighPeakPressAlarmCnt--;

    if (PltPrsValid == true) //To Prevent False Alarms
    {
      if (plateauPressure > (35.0)) //FOR COVID-19 : NHS Requirement
      {
        HighPltPressAlarmCnt++;
      } else HighPltPressAlarmCnt--;
    } else HighPltPressAlarmCnt = 0;


    if (PeepValid == true) //To Prevent False Alarms
    {
      if (PEEPressure < (5.0)) //FOR COVID-19 : NHS Requirement
      {
        LowPEEPAlarmCnt++;
      } else LowPEEPAlarmCnt--;
    } else LowPEEPAlarmCnt = 0;

    
    ////////////////////////////////////////////////////////////////////////////////
    /////////////                   SET ALARMS                      ////////////////
    /////////////Order: LOW SEVERITY (TOP) -> HIGH SEVERITY (BOTTOM)///////////////
    ///////////////////////////////////////////////////////////////////////////////

  //HIGH SEVERITY ALARMS HAVE PREFEREENCE
    if (alarm.toneFreq <= SEVERITY_LOW_FREQ) //ONLY RING IF PREVIOUSLY MUTE
    {
      if (HighPltPressAlarmCnt >= ((int)(COUNT_100ms))) //10 loop counts = 100ms;
      {
        alarm.action = RING_ALARM;
    //    alarm.toneFreq = SEVERITY_HIGH_FREQ;
    //    alarm.timePeriod = SEVERITY_HIGH_TP;
        alarm.toneFreq = SEVERITY_LOW_FREQ; //TESTING ONLY
        alarm.timePeriod = SEVERITY_LOW_TP; //TESTING ONLY
        ErrorNumber = HIGH_PIP;
      }
    }

    if (alarm.toneFreq <= SEVERITY_MED_FREQ) //ONLY RING IF PREVIOUSLY MUTE OR PREVIOUS ALARM IS LOWER SEVERITY
    {
      if (HighPltPressAlarmCnt >= ((int)(COUNT_100ms))) //10 loop counts = 100ms;
      {
        alarm.action = RING_ALARM;
        alarm.toneFreq = SEVERITY_MED_FREQ;
        alarm.timePeriod = SEVERITY_MED_TP;
        ErrorNumber = HIGH_PLT;
      }
    }
    
    if (alarm.toneFreq <= SEVERITY_HIGH_FREQ)  //ONLY RING IF PREVIOUSLY MUTE OR PREVIOUS ALARM IS LOWER SEVERITY
    {
      if (LowPEEPAlarmCnt >= ((int)(COUNT_100ms))) //10 loop counts = 100ms;
      {
        alarm.action = RING_ALARM;
        alarm.toneFreq = SEVERITY_HIGH_FREQ;
        alarm.timePeriod = SEVERITY_HIGH_TP;
        ErrorNumber = LOW_PEEP;
      }
    }
  }
  else
  {
    HighPeakPressAlarmCnt = 0;
    HighPltPressAlarmCnt = 0;
    LowPEEPAlarmCnt = 0;

    PeepValid = false; //To Prevent False Alarms
    PltPrsValid = false; //To Prevent False Alarms
  }
}

bool timer3InterruptTriggered = false;

void timer3ISR()
{
  timer3InterruptTriggered = true;
}
void setup()
{
  // put your setup code here, to run once:

  unsigned long pre_millis = 0;
  unsigned long prePressedTimestamp = 0;
  unsigned int isPressedTime = 0;

  plateauPressure = 0.0;
  PEEPressure = 0.0;
  peakInspPressure = 0.0;

  CVmode = VOL_CONT_MODE;//PRESS_CONT_MODE; //Volume Controlled Mode
  calibStatus = ST_COMPLETE;
  calibrationParam = PRESS_CONT_MODE;

  Timer3.initialize(500000);   //microseconds //0.10sec
  Timer3.attachInterrupt(timer3ISR);

//  Timer1.initialize(200);
//  Timer1.attachInterrupt(Timer);

  pinMode(pin_Button_OK, INPUT);
  pinMode(pin_Button_SNZ, INPUT);
  pinMode(pin_Switch_START, INPUT);
  pinMode(pin_Switch_MODE, INPUT);

  //    pinMode(pin_LmtSWT_CL1, INPUT);
  //    pinMode(pin_LmtSWT_CL2, INPUT);

  pinMode(pin_Knob_1, INPUT);
  pinMode(pin_Knob_2, INPUT);
  pinMode(pin_Knob_3, INPUT);
  pinMode(pin_Knob_4, INPUT);
#ifdef Beeper
  pinMode(pin_Beep, OUTPUT);
#endif

  pinMode(3, OUTPUT);

  digitalWrite(3, LOW);
  delay(500);
  digitalWrite(3, HIGH);



  Wire.begin();

  LCD_setup();
  
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(SERIAL_BAUD);
  #ifndef TEL_AT_UART0
  #ifdef TX_SERIAL_TELEMETRY
     Serial1.begin(SERIAL_BAUD);
  #endif
  #endif



  initFlowSensor();

  // reserve 50 bytes for the inputString:
  slave.AckStr.reserve(50);

  //    noInterrupts();
  pre_millis = millis();

  while ((millis() - pre_millis) < WARM_UP_TIME)
  {
    prePressedTimestamp = millis();
    isPressedTime = millis() - prePressedTimestamp;

    while (OkButton == HIGH && SnoozeButton == HIGH && isPressedTime < 2000)
    {
      isPressedTime = millis() - prePressedTimestamp;
      if (timer3InterruptTriggered)
      {
        DevModeDetectionInProg = 1;
//        userInterface();
        timer3InterruptTriggered = false;
      }
    }
    DevModeDetectionInProg = 0;

    if (isPressedTime >= 2000)
    {
      devMode = 1;
      WarmUpFlag = 0;
      devModeFunc(); //MOVE LCD_DISPLAY to Userinterface
    }
    if (timer3InterruptTriggered)
    {
//      userInterface();
      timer3InterruptTriggered = false;
    }
  }
  //    interrupts();

  noInterrupts();
  eeget();    // read startup parameters (either from EEPROM or default value)
//eeput(0);

  InitializeParams();

#ifdef CLOSED_LOOP
  control =new Control();
  float deadBand = 20; //deadBand to stop adjustment.
// Without Lung (Zero Resistance)
  // VolCoeffs[0] = 0.00000003;
  // VolCoeffs[1] = -0.00005837;
  // VolCoeffs[2] = 0.07054022;
  // VolCoeffs[3] = 0.65576958;

//0.00000004      -0.00009575     0.11151478      -0.31410825  
//0.00000007      -0.00012079     0.10933197      1.79025626 //200430 -With test lung -Damn good
//0.00000003	-0.00005732	0.07018520	0.76970458	
// With Nominal Resistance Test Lung
  VolCoeffs[0] = 0.00000003;
  VolCoeffs[1] = -0.00005732;
  VolCoeffs[2] = 0.07018520;
  VolCoeffs[3] = 0.76970458;

  if (CVmode == VOL_CONT_MODE)
    control->setConstants(0.8,0.1,deadBand,1200); //values send in this function are needed to be tested.
  else //PRESS_CON_MODE
    control->setConstants(0.8,0.1,1,40);
    //Set_constant initialization for pressure control goes here
  // Distance: 40.00mm
  //Pressure: 23.85cmH2O
  //Pressure Equation 200504 //Nominal Resistance Test Lung
  // -0.00221921	0.10285785	0.33219816	3.53803133	////Nominal Resistance Test Lung //PEEP = 5cmH2O

 // -0.00258854	0.11181313	0.41835732	1.68697428    ///Nominal Resistance Test Lung //PEEP = 5cmH2O	
   PressCoeffs[0] = -0.00258854;
  PressCoeffs[1] = 0.11181313;
  PressCoeffs[2] = 0.41835732;
  PressCoeffs[3] = 1.68697428;
#endif
  interrupts();

  tick1 = millis();
  tick2 = millis();
}

void devModeFunc() //Developer Mode
{
  while (devMode == 1)
  {
//    userInterface();
    delay(1000);
  }
}

void Ventilator_Control()
{
  static boolean initIns = true;
  static boolean initHld = true;
  static boolean initExp = true;
  static boolean initWait = true;
  static boolean runMotor = true;
  static uint8_t selectedControlMode = VOL_CONT_MODE;
  static unsigned int Tin = 0;
  static unsigned int Tex = 0;
  static unsigned int Th = 0; //ms
  static unsigned int Ttrigger = 300; //ms
  static uint16_t Tcur = 0;
  static unsigned long BreathStartTimestamp = 0;

  static float reqMotorPos = 0.0; //mm
  /*static */float Vin = 0.0; //mm/s
  /*static */float Vex = 0.0;  //mm/s
  /*static */float RPMin   = 0.0;
  /*static */float RPMex   = 0.0;

  static uint16_t stepIn = 0;
  static uint16_t stepEx = 0;
  static uint16_t periodIn = 0; //us
  static uint16_t periodEx = 0; //us


  static float stepsPredicted = 0.0;

  static boolean init = true;

  //    noInterrupts();
//  CVmode = VOL_CONT_MODE; //Proto-1
//  assistControl = 0;

  if (init)
  {

    setpoint.curBPM = setpoint.reqBPM;                 // Start from these values without sweep
    setpoint.curVolume = setpoint.reqVolume;
    setpoint.curPressure = setpoint.reqPressure;
    setpoint.curI_E = IE_R_Value[setpoint.reqI_E_Section][1]; //Exhale Factor

    breathLength = (int)(60000 / setpoint.curBPM);
    // Take the hold time out of the exhale cycle. Do this to ensure respitory rate is correct.
//    Tex = (int)((breathLength - Th) / (1 + setpoint.curI_E)); // if I/E ratio = 0.5 ; it means expiration is twice as long as inspiration
//    Tin = (int)(Tex * setpoint.curI_E);
    if (holdManeuver) Th = holdDur_ms; else Th = 0;

    slave.runAck = 0;
    init = false;
    Tcur = breathLength;
  }

  if (activateVentilatorOperation == 1)
  {
    VentilatorOperationON = 1;
    initWait = true;

    if (assistControl == 1) {
      if (patientTriggeredBreath)
      {
        Tcur = breathLength; //Start a new inhale Cycle 
      }
    }
    

    if (Tcur >= breathLength)
    {
      Tcur = 0;
      if (CVmode != selectedControlMode)
      {
        selectedControlMode = CVmode;

        if (CVmode == VOL_CONT_MODE)
          control->resetController((float)setpoint.curVolume);
        else if(CVmode == PRESS_CONT_MODE)
          control->resetController((float)setpoint.curPressure);
      }
      if (abs(setpoint.curVolume - setpoint.reqVolume) >= 1) {
        setpoint.curVolume = setpoint.reqVolume;
        if (CVmode == VOL_CONT_MODE)
          control->resetController((float)setpoint.curVolume);
      }
      if (abs(setpoint.curPressure - setpoint.reqPressure) >= 1) {
        setpoint.curPressure = setpoint.reqPressure;
        if(CVmode == PRESS_CONT_MODE)
          control->resetController((float)setpoint.curPressure);
      }
      setpoint.curBPM = setpoint.reqBPM;                 // Load Fresh User Settings
      setpoint.curI_E = IE_R_Value[setpoint.reqI_E_Section][1]; //Exhale Factor

      breathLength = (int)(60000 / setpoint.curBPM);
      if (holdManeuver) Th = holdDur_ms; else Th = 0;
      // Take the hold time out of the exhale cycle. Do this to ensure respitory rate is correct.
      Tin = (int)((breathLength - Th) / (1 + setpoint.curI_E)); // if I/E ratio = 0.5 ; it means expiration is twice as long as inspiration
      Tex = (int)(breathLength - Th - Tin);
      if (CVmode == VOL_CONT_MODE) {

      #ifdef CLOSED_LOOP
          stepsPredicted = control->compensateError((float)setpoint.curVolume,TV.inspiration);
      #endif

//        reqMotorPos = setpoint.curVolume / LINEAR_FACTOR_VOLUME; //mm
        reqMotorPos = (VolCoeffs[0] * pow(stepsPredicted, 3)) + (VolCoeffs[1] * pow(stepsPredicted, 2)) + (VolCoeffs[2] * stepsPredicted) + VolCoeffs[3];
       // reqMotorPos = stepsPredicted;
        reqMotorPos = constrain(reqMotorPos, 0.0, 40.0);
      }
      else //PRESS_CONT_MODE 
      {
        stepsPredicted = control->compensateError((float)setpoint.curPressure, plateauPressure);
        reqMotorPos = (PressCoeffs[0] * pow(stepsPredicted, 3)) + (PressCoeffs[1] * pow(stepsPredicted, 2)) + (PressCoeffs[2] * stepsPredicted) + PressCoeffs[3];
       // reqMotorPos = stepsPredicted;
        reqMotorPos = constrain(reqMotorPos, 0.0, 40.0);
      }
        Vin = reqMotorPos / ((float)Tin / 1000.0f); // mm/s
        Vex = reqMotorPos / ((float)Tex / 1000.0f); // mm/s
        RPMin = (Vin / LIN_MECH_mm_per_rev) * 60.0;
        RPMex = (Vex / LIN_MECH_mm_per_rev) * 60.0;
        stepIn = (long)((reqMotorPos / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV);
        stepEx = (long)(stepIn + ((2.0 / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV));
        periodIn = (long)((((float)Tin / 1000.0) / stepIn) * 1000000); //us
        periodEx = (long)((((float)Tex / 1000.0) / stepIn) * 1000000); //us
        BreathStartTimestamp = millis();

//#ifdef __DEBUG
#ifndef TEL_AT_UART0
          static int i = 0;
           Serial.print(F("In Ventilator Control: ")); Serial.println(i++);
           Serial.print(F("Control Mode:      ")); Serial.println(CVmode); //0 = VOL; 1 = PRESS          
           Serial.print(F("Breathing Length:      ")); Serial.println(breathLength);
           Serial.print(F("Inspiration Time:      ")); Serial.print(Tin); Serial.println(F(" ms"));
           Serial.print(F("Expiration Time:       ")); Serial.print(Tex); Serial.println(F(" ms"));
           Serial.print(F("targetPosition:        ")); Serial.print(reqMotorPos); Serial.println(F(" mm"));
           Serial.print(F("Motor Speed Insp:      ")); Serial.print(Vin); Serial.println(F(" mm/s"));
           Serial.print(F("Motor Speed Exp:       ")); Serial.print(Vex); Serial.println(F(" mm/s"));
           Serial.print(F("RPM Insp:              ")); Serial.println(RPMin);
           Serial.print(F("RPM Exp:               ")); Serial.println(RPMex);
           Serial.print(F("Steps Insp:            ")); Serial.println(stepIn);
           Serial.print(F("Steps Exp:             ")); Serial.println(stepEx);
           Serial.print(F("Period Insp:           ")); Serial.print(periodIn); Serial.println(F(" us"));
           Serial.print(F("Period Exp:            ")); Serial.print(periodEx); Serial.println(F(" us"));
#endif           
           
//#endif
    }
    Tcur = millis() - BreathStartTimestamp;

      if (Tcur <= Tin)
      {
        if (initIns)
        {
//          slave.runAck = 0;
          runMotor = true;
          initHld = true;
          initIns = false;
          initExp = true;
        }        
//        breathPhase = INSPIRATION_PHASE;
        if (runMotor && (slave.runAck == 0 || slave.runAck == 2)) //!setpointAchieved && //CMD NOT RECEIVED
        {
          breathPhase = INSPIRATION_PHASE;
          txSlaveCMD(RUN, periodIn, stepIn, "1");
          runMotor = false;
          slave.lastCMD_ID = RUN;
        }
        PEEPMesaureFlag = 0;
      }
      else if ((Tcur > Tin) && (Tcur <= (Tin + Th)))
      {
        if (initHld)
        {
          slave.runAck = 2;
          txSlaveCMD(STOP);
          slave.lastCMD_ID = STOP;
//          slave.stopAck = 0;
          initHld = false;
          initIns = true;
          initExp = true;
        }        
        breathPhase = HOLD_PHASE;
      }
      else if ((Tcur > (Tin + Th)) && (Tcur < (Tin + Th + Tex)))
      {
        if (initExp)
        {
         // slave.runAck = 0;
          runMotor = true;
          initHld = true;
          initIns = true;
          initExp = false;
        }                
//        breathPhase = EXPIRATION_PHASE;

        if (runMotor && (slave.runAck == 0 || slave.runAck == 2)) //CMD NOT RECEIVED
        {
          breathPhase = EXPIRATION_PHASE;
          txSlaveCMD(RUN, periodEx, stepEx, "0");
          runMotor = false;
          slave.lastCMD_ID = RUN;
        }

        if ((Tcur >= (Tin + Tex)) && (Tcur < (Tin + Th + Tex)))
        {
          PEEPMesaureFlag = 1;
        }

        if (assistControl == 1) {
        if ((Tin + Th + Tex - Tcur) < Ttrigger) 
        {
          scanBreathingAttempt = true;
        }        }
      }
  }
  else
  {
    #ifndef TEL_AT_UART0
    //      Serial.println(F("Ventilator Operation Halt"));
    #endif
    if (initWait) {slave.homeAck = 0; initWait = false;}
    initHld = true;
    initIns = true;
    initExp = true;
    VentilatorOperationON = 0;
    breathPhase = WAIT_PHASE;
    Tcur = breathLength; // This will always start inspiration breath cycle on setting switch to start position
    PeepValid = false; //To Prevent False Alarms
    PltPrsValid = false; //To Prevent False Alarms


    if (slave.homeAck == 0)
    {
        txSlaveCMD(HOME, 2000);
        slave.lastCMD_ID = HOME;
	    slave.runAck = 0;
    }
    PEEPMesaureFlag = 0;
  }
  //    interrupts();
}

#ifdef TX_SERIAL_TELEMETRY
void GetTelData()
{

  static boolean init = true;
  byte TEL_BYTE = 0x00;
  if (init)
  {
    TEL.Time = 0;
    TEL.txUpdateRate = 0;
    TEL.FDCB = 0xFF;
    init = false;
  }

  TEL.Time += (samplePeriod1);

  if ((TEL.Time % 20) == 0)
  {
    TEL.mTV = TV.measured; //ml
    //TEL.mTV = constrain(TEL.mTV, 0.0, 1000.0);
    TEL.mTVinsp = TV.inspiration; //ml
    TEL.mTVexp = TV.expiration; //ml
    TEL.mPressure = p_sensor.pressure_gauge_CM; //cmH2O
    TEL.mFlowRate = FS.Q_SLM; //SLPM
    TEL.mPEEP = PEEPressure;
    TEL.mPltPress = plateauPressure;
    TEL.mFiO2 = 0.0;
    TEL.minuteVentilation = TV.minuteVentilation;
    TEL.mPeakPressure = peakInspPressure;
    TEL.mRR = 0.0;
    TEL.staticCompliance = TV.staticCompliance;
    TEL.spTrigger = setpoint.flowTriggerSenstivity;

    TEL.spTV = setpoint.reqVolume;
    TEL.spInsPressure = setpoint.reqPressure;
    TEL.spExpPressure = 5; //0;

    TEL.spFiO2 = setpoint.reqFiO2;
    TEL.spBPM = setpoint.reqBPM;
    TEL.spIE_Inhale = IE_R_Value[setpoint.reqI_E_Section][0];
    TEL.spIE_Exhale = IE_R_Value[setpoint.reqI_E_Section][1];
    TEL.patientWeight = patientWeight;

    TEL.statusByteError = ErrorNumber;
    TEL_BYTE = 0x00;
    TEL_BYTE |= breathPhase & 0x03;
    if (CVmode == PRESS_CONT_MODE)  TEL_BYTE |= 0x04;
    if (assistControl == 1)  TEL_BYTE |= 0x08;
    if (VentilatorOperationON == 1)  TEL_BYTE |= 0x10;
    if (selfTestProg == ST_IN_PROG)
      TEL_BYTE |= 0x20;
    else if ((selfTestProg == ST_COMPLETE) && (selfTestStatus == ST_FAIL))
      TEL_BYTE |= 0x40;
    else if ((selfTestProg == ST_COMPLETE) && (selfTestStatus == ST_PASS))
      TEL_BYTE |= 0x60;
    else //NOT INIT
      TEL_BYTE &= 0x9F; //Clear D5 and D6
//    if (spStatusAllowChange == 1) TEL_BYTE |= 0x80; //D7 High if Settings Unsaved

    TEL.statusByte1 = TEL_BYTE;
    TEL.FDCB = 0xCC;
  }
}
#endif



void loop()
{

  unsigned long start_Ts = 0;
  WarmUpFlag = 0;
#ifdef StepGen
//  digitalWrite(pin_Stepper_SLP, HIGH);
#endif
  if (millis() > (tick1 + samplePeriod1))
  {
    start_Ts = micros();
    tick1 = millis();
    readSensors();
    Monitoring();

//    if (slave.strComplete == true)
//    {
//      decodeSlaveTel();
//      slave.AckStr = "";
//      slave.strComplete = false;
//    }
    

    if (selfTestProg != ST_COMPLETE)
      selfTest();
    else
    {
//      alarmControl();
    }
#ifdef ActiveBeeper
    beep(); //alarmAction = RING_ALARM, SNOOZE_ALARM; alarmSeverity = SEVERITY_HIGH, SEVERITY_MED, SEVERITY_LOW, SEVERITY_MUTE
#endif

#ifdef TX_SERIAL_TELEMETRY
    GetTelData(); //Called at 100Hz
    Prepare_Tx_Telemetry(); //Called at 100Hz
#endif
//Serial.print(F("Busy Time 1: ")); Serial.println(micros()-start_Ts);
  }
  if ((selfTestProg == ST_COMPLETE) && (selfTestStatus == ST_PASS)) // I am not writing control loop inside 100Hz loop to keep both loop rates independant
  {
    if (millis() > (tick2 + samplePeriod2))
    {
      start_Ts = micros();
      tick2 = millis();
      Ventilator_Control(); //Mandatory Volume Controlled Mode Only
//Serial.print(F("Busy Time 2: ")); Serial.println(micros()-start_Ts);
    }
  }
  if (timer3InterruptTriggered)
  {
    readSwitches();
//     Serial.println("Entering Display Func");
//     start_Ts = micros();
     Display();
// Serial.print("Busy Time Display: "); Serial.println(micros()-start_Ts);
//     Serial.println("Exiting Display Func");
// #ifdef Beeper
//   if (SnoozeButton == 1)
//   {
//     alarm.action = SNOOZE_ALARM;
//     ErrorNumber = 0;
//   }
// #endif
    timer3InterruptTriggered = false;
  }
}


/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent2() {
    while (Serial2.available()) {
      // get the new byte:
      char inChar = (char)Serial2.read();
      // add it to the inputString:
      slave.AckStr += inChar;
      // if the incoming character is a newline, set a flag so the main loop can
      // do something about it:
      if (inChar == '\r') {
        #ifndef TEL_AT_UART0
      Serial.println(slave.AckStr);
      #endif
        slave.strComplete = true;
      }
    }
    if (slave.strComplete == true)
    {
      decodeSlaveTel();
      slave.AckStr = "";
      slave.strComplete = false;
    }
}

void decodeSlaveTel()
{
  String message = "";

    for (int i = 0; i < slave.AckStr.length(); i++)
    {
      if (slave.AckStr[i] == '#')
      {
        if ((slave.AckStr[i+1] == 'R') && (slave.AckStr[i+2] == 'U') && (slave.AckStr[i+3] == 'N'))
        {
          message = slave.AckStr[i+5];
          slave.runAck = message.toInt();
    //      Serial.print("runAck = ");Serial.println(slave.runAck);
          break;
        }
        else if ((slave.AckStr[i+1] == 'S') && (slave.AckStr[i+2] == 'T') && (slave.AckStr[i+3] == 'O') && (slave.AckStr[i+4] == 'P'))
        {
          message = slave.AckStr[i+6];
          slave.stopAck = message.toInt();
      //    Serial.print("stopAck = ");Serial.println(slave.stopAck);
          break;
        }
        else if ((slave.AckStr[i+1] == 'H') && (slave.AckStr[i+2] == 'O') && (slave.AckStr[i+3] == 'M') && (slave.AckStr[i+4] == 'E'))
        {
          message = slave.AckStr[i+6];
          slave.homeAck = message.toInt();  
//          Homing_Done_F = slave.homeAck;
     //     Serial.print("homeAck = ");Serial.println(slave.homeAck);
          break;        
        }
      }
    }
}


void txSlaveCMD(int CMD_ID, unsigned int period=0, unsigned int pulses=0, String dir="0")
{
  String cmdString = "";
  switch (CMD_ID)
  {
  case RUN:
    cmdString = "#RUN " + String(pulses) + " " + String(period) + " " + dir + "\r";
    break;
  case HOME:
    cmdString = "#HOME " + String(period) + "\r";
    break;
  case STOP:
    cmdString = "#STOP\r";
    break;
  default:
    break;
  }
  Serial2.print(cmdString);
#ifndef TEL_AT_UART0
  Serial.println(cmdString);
  #endif
}
