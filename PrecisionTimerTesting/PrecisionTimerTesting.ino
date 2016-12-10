void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initPreciseGlobal();
}
unsigned int precise_overflow = 0;
void loop() {
  unsigned long long orig = preciseTime();
  unsigned long m_orig = millis();
  while((millis()-m_orig) < 10){}
  orig = preciseTime() - orig;
  //unsigned long diff = orig;
//  noInterrupts();
  uint64_t pipe = preciseTime();//lets assume  the data is 12345ABCD9 hexadecimal
  char buf[50];
  if(pipe > 0xFFFFFFFFLL) {
    sprintf(buf, "%lu%08lu", (unsigned long)(pipe>>32), (unsigned long)(pipe&0xFFFFFFFFULL));
  } else {
    sprintf(buf, "%lu", (unsigned long)pipe);
  }
  //Serial.println( buf );
  Serial.println(buf);
}



static void initPreciseGlobal() { //sets up timer 5 for counting
  TCCR5A = 0x00; // Normal mode; No output comparitors used
  TCCR5B = 0x01; //No clk prescaler
  TIMSK5 = 0x01;
  interrupts();
}
static unsigned long long preciseTime() {
  //reads timer and overflows, updates precise
  return ((precise_overflow*(unsigned long long)65536)+(unsigned long long)TCNT5);
}

ISR(TIMER5_OVF_vect)
{
  precise_overflow++;
//  TIFR5 = 0x00;//clear flag
}
