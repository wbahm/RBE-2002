#include <Wire.h>

int IRsensorAddress = 0xB0;
int slaveAddress;
byte data_buf[42];
int i;

int Ix[4];
int Iy[4];
int s;

void Write_2bytes(byte d1, byte d2)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1);
    Wire.write(d2);
    Wire.endTransmission();
}

void setup()
{
    slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
    Serial.begin(115200);
     Wire.begin();
             //reg,data
  Write_2bytes(0x1A,0x04); delay(10);
  Write_2bytes(0x30,0x08); delay(10);
  Wire.beginTransmission(slaveAddress);
  Wire.write(0x00); //block 1 reg start

  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0xFF);
  Wire.write(0x00);
  Wire.write(0xC0);
  Wire.endTransmission();
  Wire.beginTransmission(slaveAddress);
  Wire.write(0x1A); //block 1 reg start

  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
  Write_2bytes(0x33,0x03); delay(10);
  Write_2bytes(0x30,0x08); delay(10);
  delay(100);
    
}
void loop()
{
    //IR sensor read
    for (i=0;i<42;i++) { data_buf[i]=0; } //clear data buffer
    Wire.beginTransmission(slaveAddress);
     //        Wire.write(0x1A);
    Wire.write(0x3E);
    Wire.endTransmission();
    Wire.requestFrom(slaveAddress, 21);        // Request the 2 byte heading (MSB comes first)
    i=0;
    while(Wire.available() && i < 21) {
        data_buf[i] = Wire.read();
        char str[2];
        sprintf(str,"%02x", data_buf[i]);
        Serial.print(str);
        i++;
    }
    Serial.print(' ');
        Wire.beginTransmission(slaveAddress);
              Wire.write(0x1A);
    Wire.write(0x3E);
    Wire.endTransmission();
    Wire.requestFrom(slaveAddress, 21);        // Request the 2 byte heading (MSB comes first)
    i=0;
    while(Wire.available() && i < 21) {
        data_buf[i] = Wire.read();
        char str[2];
        sprintf(str,"%02x", data_buf[i+21]);
        Serial.print(str);
        i++;
    }
    
/*
    while(Wire.available() && i < 21) {
        data_buf[18+i-3] = Wire.read();
      }
        i++;
    }
    while(Wire.available()) Wire.read();
    for(int k =0;k<4;k++)
    {
      byte* target = &data_buf[(k*9)+1];
      int targetSize = target[2] & 0xF;
      int xPos = (((target[2]>>4)&0x3)<<8)|(target[0]);
      int yPos = (((target[2]>>6)&0x3)<<8)|(target[1]);
      int xMin = target[3];
      int yMin = target[4];
      int xMax = target[5];
      int yMax = target[6];
      int intensity = target[8];
      Serial.print(xPos);
      Serial.print(',');
      Serial.print(yPos);
      Serial.print(',');
      Serial.print(target[3],HEX);
      Serial.print('\t');
    }*/
    Serial.println(' ');
    
/*
    Ix[0] = data_buf[1];
    Iy[0] = data_buf[2];
    s   = data_buf[3];
    Ix[0] += (s & 0x30) <<4;
    Iy[0] += (s & 0xC0) <<2;

    Ix[1] = data_buf[4];
    Iy[1] = data_buf[5];
    s   = data_buf[6];
    Ix[1] += (s & 0x30) <<4;
    Iy[1] += (s & 0xC0) <<2;

    Ix[2] = data_buf[7];
    Iy[2] = data_buf[8];
    s   = data_buf[9];
    Ix[2] += (s & 0x30) <<4;
    Iy[2] += (s & 0xC0) <<2;

    Ix[3] = data_buf[10];
    Iy[3] = data_buf[11];
    s   = data_buf[12];
    Ix[3] += (s & 0x30) <<4;
    Iy[3] += (s & 0xC0) <<2;

    for(i=0; i<4; i++)
    {
      if (Ix[i] < 1000)
        Serial.print("");
      if (Ix[i] < 100)  
        Serial.print("");
      if (Ix[i] < 10)  
        Serial.print("");
      Serial.print( int(Ix[i]) );
      Serial.print(",");
      if (Iy[i] < 1000)
        Serial.print("");
      if (Iy[i] < 100)  
        Serial.print("");
      if (Iy[i] < 10)  
        Serial.print("");
      Serial.print( int(Iy[i]) );
      if (i<3)
        Serial.print(",");
    }
    Serial.println("");*/
    
    delay(100);
}

