/*
*********************************************************
Sample sketch for control of a power supply with PWM 
use to simulate a PV panel in order to test MPPT modules

USE WITH ARDUINO LEONARDO AND MICRO ONLY!!!

****************************************************
*/

#include "PWM_32U4.h" // library for fast PWM
#include "PID_v1.h"

byte pwmValue = 125;
int val = 0;
float icc = 1.0; // courant de court-circuit (Ampères) du panneau
float tensvid = 10.0; // tension a vide du panneau
float coeff = tensvid*0.054;

//Connect setpoint to A0
//Connect PWM pin  to D6

#define PIN_INPUT A2 // mesure de la tension
#define PIN_SETPOINT A0 // consigne
#define courPin A1 // tension aux bornes de la resistance de mesure du courant

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=5, Ki=10, Kd=0.00;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
 
void setup() {
  Serial.begin(115200);
  //initialize all timers except for 0, to save time keeping functions

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);    

    pwm613configure(PWM23k);
    pwmSet6(0);
	Setpoint = 0;

  //turn the PID on and the time step for PID calculation

  myPID.SetSampleTime(1);
  myPID.SetMode(AUTOMATIC);
}
 


void loop() {

  float tenscons;
  double courrant = analogRead(courPin); // tension aux bornes de la resistance de mesure du courant debite dans la charge
  float current = 2.0*courrant*5.0/1023.0; // si R = 0.5 ohm
  Input = (2.0*analogRead(PIN_INPUT)-courrant)*10.0/1023.0; // tension du PV
  // calcul du point de consigne en tension
  tenscons = coeff*log(1+30000000*(icc-current));
  if (tenscons >0) Setpoint = tenscons*102.3;
  else Setpoint = 0;
//  Setpoint = analogRead(PIN_SETPOINT);
  myPID.Compute();

	pwmWrite6(Output);

 // Serial.println(String(Input) + ','+ String(Setpoint) + ',' + String(current)+',' + String(current*Input));
Serial.println( String(current*Input));
}
 
