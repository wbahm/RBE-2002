#include <Encoder.h>
Encoder myEnc(18, 19);
#include <PID_v1.h>;
double v_target = 8; // in/sec PID setpoint
double motorVelocity = 0; //pid input
double motorPwr = 0; //pid output
double kp = 2000;
double ki = 6000;
double kd = 30;
PID myPID(&motorVelocity, &motorPwr, &v_target, kp, ki, kd, DIRECT);
void setup() {
  Serial.begin(115200);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(25);
  myPID.SetOutputLimits(10000,65535);
  // put your setup code here, to run once:
  initDrivePWM();
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
}

void loop() {
  myPID.Compute();
  setDrivePWM(motorPwr);
  updateVelocity();
  Serial.print(millis());
  Serial.print('\t');
  Serial.print(motorPwr);
  Serial.print('\t');
  Serial.println(motorVelocity);
}
unsigned long lastCount = 0;
unsigned long long lastTime = 0;
static const float DIST_PER_REV = 3.14159*2.75;
static const float TICKS_PER_REV = 8245.81;
void updateVelocity() {
  unsigned long diffCnt = myEnc.read()-lastCount;
  unsigned long long diffTime = micros()-lastTime;
  motorVelocity = (diffCnt*DIST_PER_REV*1000000.0)/(diffTime*TICKS_PER_REV);
  lastTime = micros();
  lastCount = myEnc.read();
}
#define PWM_FREQ 16000
#define CLOCK_FREQ 16000000
static const unsigned int PWM_COUNT = (((CLOCK_FREQ/PWM_FREQ))-1);// make constant to avoid recomputing each time


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

    OCR3B = ((duty_cycle*(unsigned long)PWM_COUNT)/65535);
}

