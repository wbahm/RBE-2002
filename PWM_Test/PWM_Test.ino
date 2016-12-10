void setup() {
  // put your setup code here, to run once:
  initDrivePWM();
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
    pinMode(6,OUTPUT);
  digitalWrite(6,HIGH);
  Serial.begin(115200);
    //setDrivePWM(65000);
}

#define PWM_FREQ 16000
#define CLOCK_FREQ 16000000
static const unsigned int PWM_COUNT = (((CLOCK_FREQ/PWM_FREQ))-1);// make constant to avoid recomputing each time

void loop() {
  // put your main code here, to run repeatedly:

  for(unsigned int i = 20000;i<65535;i+=5000)
  {
    setDrivePWM(i);
    delay(1000);
    //delay(1000);
  }

}

static void initDrivePWM() {
    pinMode(2,OUTPUT);
    pinMode(5,OUTPUT);
    TCCR3B = _BV(WGM33)|_BV(WGM32)|_BV(CS30);
    TCCR3A = _BV(COM3A1)|_BV(COM3B1)|_BV(WGM31);//WGM3_14; //COM3 and WGM3 = 14 _BV(COM3A0)
    ICR3 = PWM_COUNT; // to count up to
    OCR3A = PWM_COUNT/2;///2; //default to 50% duty cycle
    OCR3B = PWM_COUNT/2;///2; //default to 50% duty cycle
}

static void setDrivePWM(unsigned long duty_cycle) { 
  //Sets the new PWM duty cycle; duty cycle scaled from 0 to 65535 for 0 - 100 percent

    OCR3B = ((duty_cycle*(unsigned long)PWM_COUNT)/65535);
    OCR3A = ((duty_cycle*(unsigned long)PWM_COUNT)/65535);
    Serial.println(((duty_cycle*(unsigned long)PWM_COUNT)/65535));
}

