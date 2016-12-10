void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Sys start");
}

void loop() {
  float y = 0;
  long long t = micros();
  for(float i = 0; i < 3000;i++)
  {
    y = sin(i);
  }
  Serial.print("sin ");
  Serial.print(y);
  Serial.print('\t');
  Serial.println((micros()-t)/3000.0);
  y = 0;
  t = micros();
  for(float i = 0; i < 3000;i++)
  {
    y = cos(i);
  }
  Serial.print("cos ");
  Serial.print(y);
  Serial.print('\t');
  Serial.println((micros()-t)/3000.0);
}
