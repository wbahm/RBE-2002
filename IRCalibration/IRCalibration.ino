
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReference(INTERNAL2V56);
}

const int PINS[5] = {A0,A1,A2,A3,A4};

const int NUM_SAMPLES = 2500;

void loop() {
  
  for(int i = 0; i<5;i++)
  {
    unsigned long total = 0;
    unsigned int min_val = 1023;
    unsigned int max_val = 0;
    unsigned int readings[NUM_SAMPLES];
    for(int k = 0; k<NUM_SAMPLES;k++)
    {
      readings[k] = analogRead(PINS[i]);
      if(readings[k]>max_val)
      {
        max_val = readings[k];
      }
      if(readings[k]<min_val)
      {
        min_val = readings[k];
      }
      total += readings[k];
    }
    double mean = total/(double)NUM_SAMPLES;
    double std_sum = 0;
    for(int j = 0;j<NUM_SAMPLES;j++)
    {
      std_sum += (readings[j]-mean)*(readings[j]-mean);
    }
    double variance = std_sum/(double)NUM_SAMPLES;
    double std = sqrt(variance);
    Serial.print(min_val);
    Serial.print('\t');
    Serial.print(max_val);
    Serial.print('\t');
    Serial.print(mean);
    Serial.print('\t');
    Serial.print(std);
    Serial.print('\t');
  }
  Serial.println(' ');
}
