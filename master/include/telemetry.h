#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#define TEL_PACKET_LENGTH 48
struct TEL_TYPE {

byte FDCB;

uint8_t txUpdateRate;

unsigned long Time;
float mTV;
float mTVinsp;
float mTVexp;
float mPressure;
float mFlowRate;
float mPEEP;
float mPltPress;
float mFiO2;
float minuteVentilation;
float staticCompliance;
float mRR;
float mPeakPressure;

uint16_t spTV;
int16_t spInsPressure;
int16_t spExpPressure;
uint8_t spFiO2;
uint8_t spBPM;
uint8_t spIE_Inhale;
uint8_t spIE_Exhale;
int16_t spPEEP;
uint8_t patientWeight;
float spTrigger;

uint8_t statusByteError;
uint8_t statusByte1;
}; 

void Prepare_Tx_Telemetry();

#endif
