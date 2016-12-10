//Written by Joseph St. Germain for RBE 2002 Lab 3 A16.
//Students are expected to fill in the PID::calc function and the calcCur function
// The PID calc function is inteded to run a PID velocity controller 
//and the calcCur function should calculate the current in ma from the current sensor.

#include "LookupTable.h"
#include "PID.h"
#include <Servo.h>
#include <Encoder.h>

//Setup lookup table class as lk
Lookup lk;
//setup PID class as pidCon
PID pidCon;
Encoder myEnc(2, 3);

//Variables to run the code
//setVel is the velocity used as an input to PID 
double setVel=90;
double curTime=0;
double prevTime=0;
double timeInterval=0;
double movement=0;
double ticksToDeg= 0.2206;
double prevPos;
long lastTime=0;
double Vel;
float velocity;
float RPM;
float current;

// Returns Vel in degress/second
double calcVel(){
  //current positions
  double curPos=myEnc.read();
  //current time
  curTime=millis();
  //time change in ms from last call
  timeInterval=curTime-prevTime;
  //encoder ticks since last call
  movement=curPos-prevPos;
  //encoder ticks to degrees
  movement= movement *ticksToDeg;
  //timeInterval in seconds
  timeInterval=timeInterval/1000;
  //Velocity in degrees per milliseconds
  Vel=movement/timeInterval;
  //sets curent vals to previous
  prevPos=curPos;
  prevTime=curTime;
  return Vel;
}


//This function should analogRead the current sense from the motor driver
//and convert the value to current in milliamps
double calcCur(void)
{
  //Read analog pin from current sensor
  int val = analogRead(A1);
  //convert to volts
  float voltage = val*(5/1023.0f);
  //converts to current in MilliAmps
  float milliamps = (voltage/0.525)*1000;
  return milliamps;
}

void setup() {
  Serial.begin(115200);
  pidCon.setpid(2,0,.01);
}


//Main loop
void loop() {

//Waits for the time constant
if(millis()-lastTime>10){
  
  //Calculate velocity from the encoder in degrees/second
  velocity=calcVel();
  //Serial.print("velocity=");
  Serial.print(velocity);
  //Serial.print(" , ");  
  Serial.print('\t');
  
  //Calculates RPM from velocity
  RPM =((velocity*60)/360);
  //Serial.print("RPM = ");
  Serial.print(RPM);
  //Serial.print(" , ");  
  Serial.print('\t');

  //calculates the current of the motor from the analogRead function
  //use the motor drivers data sheet to find the conversion from ADC counts
  //to milliamps 
  current = calcCur();
  //Serial.print("current = ");
  Serial.print(current);
  //Serial.print(" , "); 
  Serial.print('\t');
  
  //Calculate the PID value to be sent to the motor using the analogWrite function.
  //The calc function should take in a set velocity and the current velocity.
  float analogWriteVal = pidCon.calc(setVel,velocity);
   analogWrite(5,analogWriteVal);
  
  //Calculates the dutycycle of the PWM 0-1
  float PWM = analogWriteVal/255;
//  Serial.print("PWM = ");
  Serial.print(PWM);
 // Serial.print(" , "); 
 Serial.print('\t');

  //Use the lookup table class to find the torque given the current and RPM
  //Serial.print("torque = ");
  Serial.println(lk.torque(current,RPM));




  
 //Sets the lastTime variable 
  lastTime=millis();
}
}

