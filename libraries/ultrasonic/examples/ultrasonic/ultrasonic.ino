//library for the VEX ultrasonic sensor
#include <ultrasonic.h>

//call the ultrasonic constructor
//ultrasonic(INPUTPIN, OUTPUTPIN)
ultrasonic ultrasonic(31, 27);


void setup() {
  //setup serial communication for print statements
  Serial.begin(115200);
}

void loop() {
  //variable to store the distance
  float dist = 0;

  //desired distance
  float desiredDist = 6.5;

  //find the distance to the object
  //time for the function to return is approximately (ms)
  //~285 max range
  //~100 min range
  dist = ultrasonic.distance();

  //if the distance measured is past 108in (max range) set it to 108
  //likely want to resample in this case
  if (dist > 108.0) dist = 108;

  //check to see if the the distance reached within 10% of the desired distance
  if (dist >= desiredDist * 0.9 && dist <= desiredDist * 1.1) {
    Serial.println("Target Reached!");
    Serial.print("Dist: ");
    Serial.println(dist, DEC);

    //do something

  }
  //if it's too close
  else if (dist < desiredDist) {
    Serial.println("Too Close!");
    Serial.print("Dist: ");
    Serial.println(dist, DEC);
    
    //do something

  }
  //otherwise it's too far
  else{
    Serial.println("Too far!");
    Serial.print("Dist: ");
    Serial.println(dist, DEC);
    
    //do something

  }
}
