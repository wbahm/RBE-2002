#include "digitalIOPerformance.h"
#include "PreciseTime.h"
#include "PID.h"
volatile unsigned long count;
byte previousState;
unsigned long long lastTime;

const float setRPS = 0.5;
const float ticksPerSecond = 8245.81/setRPS;
const float usPerTick = 1000000/ticksPerSecond;
const unsigned long TEST_SETPOINT = (unsigned long)usPerTick*16;

#define PWM_FREQ 16000
#define CLOCK_FREQ 16000000
static const unsigned int PWM_COUNT = (((CLOCK_FREQ/PWM_FREQ))-1);// make constant to avoid recomputing each time

const unsigned long kp = 1;
const unsigned long ki = 0;
const unsigned long kd = 0;
unsigned long pidIn;
unsigned long pidOut;
unsigned long setPoint = 0;
PID myPid(kp,ki,kd,&pidIn,&pidOut,setPoint);
void setup() {
  // put your setup code here, to run once:
  pinModeFast(18,INPUT);
  pinModeFast(19,INPUT);
  count = 0;
  byte chanA = digitalReadFast(18);
  byte chanB = digitalReadFast(19);
  previousState = getState(chanA,chanB);
  initDrivePWM();
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
  Serial.begin(115200);
  interrupts();
  attachInterrupt(digitalPinToInterrupt(18), leftEncInterrupt, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(19), leftEncInterrupt, CHANGE); 
  initPreciseGlobal();
  lastTime = preciseTime();
  myPid.setSetpoint(TEST_SETPOINT);
}
unsigned long lastCount = 0;
unsigned long long lastPTime = 0;
void loop() {
  lastPTime = preciseTime();
  delay(1000);
  lastPTime = preciseTime()-lastPTime;
  unsigned long diffCount = count-lastCount;
  float RPS = (diffCount/8245.81)/(lastPTime/16000000.0);
  unsigned long tDiff = (unsigned long)lastPTime/16;
  noInterrupts();
  unsigned long pIn = (unsigned long)pidIn;
  unsigned long pOut = (unsigned long)pidOut;
  interrupts();
 /* Serial.print(tDiff);
  Serial.print('\t');
  Serial.print(diffCount);
  Serial.print('\t');
  Serial.print(pIn);
  Serial.print('\t');
  Serial.print(pOut);
  Serial.print('\t');
  Serial.println(RPS);*/
  unsigned long a = micros();
  myPid.compute();
  a = micros()-a;
  Serial.println(a);
//  Serial.println((unsigned long)TEST_SETPOINT);
  lastCount = count;


}
byte getState(byte chanA_val,byte chanB_val) {
  if(~chanA_val&chanB_val)
    return 0x00;
  else if(chanA_val&chanB_val)
    return 0x01;
  else if(chanA_val&(~chanB_val))
    return 0x02;
  else
    return 0x03;
}

static void initDrivePWM() {
    pinMode(2,OUTPUT);
    pinMode(5,OUTPUT);
    TCCR3B = _BV(WGM33)|_BV(WGM32)|_BV(CS30);
    TCCR3A = _BV(COM3A1)|_BV(COM3B1)|_BV(WGM31);//WGM3_14; //COM3 and WGM3 = 14 _BV(COM3A0)
    ICR3 = PWM_COUNT; // to count up to
    OCR3A = PWM_COUNT/2; //default to 50% duty cycle
    OCR3B = PWM_COUNT/2; //default to 50% duty cycle
}

static void setDrivePWM(unsigned int duty_cycle) { 
  //Sets the new PWM duty cycle; duty cycle scaled from 0 to 65535 for 0 - 100 percent
    constrain(duty_cycle,0,65535);
    OCR3B = ((duty_cycle*(unsigned long)PWM_COUNT)/65535);
}

void leftEncInterrupt() {
  //get diff time
  unsigned long long newTime = preciseTime();
  unsigned long long diffTime = newTime-lastTime;
  lastTime = preciseTime();
  pidIn = diffTime;
  //compute enc count
  byte chanA = digitalReadFast(18);
  byte chanB = digitalReadFast(19);
  byte state = getState(chanA,chanB);
  //byte state = ((~chanB)<<1)|((chanA&chanB)|(~(chanA|chanB))); //0-3
  if((previousState == 0x03) && (state==0x00))
  {
    //wrapped around
    count++;
  }
  else if((previousState == 0x00) && (state==0x03))
  {
    //wrapped around
    count--;
  }
  else if(state>previousState)
  {
    count++;
  }
  else if(state<previousState)
  {
    count--;
  }
  previousState = state;
  //compute pid
//  myPid.compute();
  //set motor based on PID output
  //long newVal = 32767+pidOut;
  /*if(newVal>65535)
    newVal = 65535;
  else if(newVal<0)
    newVal = 0;*/
  setDrivePWM(65535);//(newVal);
}
