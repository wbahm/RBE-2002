#include "digitalIOPerformance.h"
byte previousState = 0x00;
void setup() {
  pinModeFast(18,INPUT);
  pinModeFast(19,INPUT);
  byte chanA = digitalReadFast(18);
  byte chanB = digitalReadFast(19);
  //previousState = ((~chanB)<<1)|((chanA&chanB)|(~(chanA|chanB))); //0-3
   if(~chanA&chanB)
    previousState = 0x00;
  else if(chanA&chanB)
    previousState = 0x01;
  else if(chanA&(~chanB))
    previousState = 0x02;
  else
    previousState = 0x03;
  Serial.begin(115200);
  interrupts();
  attachInterrupt(digitalPinToInterrupt(18), leftEncInterrupt, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(19), leftEncInterrupt, CHANGE); 
}

static long count = 0;
static long lastCount = 0;
void loop() {
  lastCount = count;
  delay(1000);
  Serial.println(count-lastCount);
}
void leftEncInterrupt() {
  byte chanA = digitalReadFast(18);
  byte chanB = digitalReadFast(19);
  byte state = 0x00;
  //byte state = 0x00;
  if(~chanA&chanB)
    state = 0x00;
  else if(chanA&chanB)
    state = 0x01;
  else if(chanA&(~chanB))
    state = 0x02;
  else
    state = 0x03;
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
}
