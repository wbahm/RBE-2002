#include "PreciseTime.h"
#include "Arduino.h"

unsigned int precise_overflow = 0;

void initPreciseGlobal() { //sets up timer 5 for counting
  TCCR5A = 0x00; // Normal mode; No output comparitors used
  TCCR5B = 0x01; //No clk prescaler
  TIMSK5 = 0x01;
  interrupts();
}
unsigned long long preciseTime() { //time in 1/16 us ticks
  //reads timer and overflows, updates precise
  return (((precise_overflow*(unsigned long long)65536)+(unsigned long long)TCNT5));
}

ISR(TIMER5_OVF_vect)
{
  precise_overflow++;
}

