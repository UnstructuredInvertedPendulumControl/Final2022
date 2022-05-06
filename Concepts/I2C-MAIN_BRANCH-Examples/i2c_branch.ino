#include <Wire.h>



//--------------------------------------------------------------------------------------------------
/* 
Example I2C Packet
  - definition must be on both Main and Branch device code
  - type can be used to cast byte arrays of the same size to this data struct
  
  double - 8 bytes

  total size of this array - 16 bytes
      - use sizeof('typename') to remove hardcoded size numbers

*/
typedef struct {
    double value;
    double value2;
} i2c_packet;

i2c_packet pkt;/* Global Structure variable defintion */
double reading = 0;
//--------------------------------------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Wire.begin(0x04);
  Wire.onRequest(i2c_handler);
  Serial.begin(9600);
  Serial.println("Branch Device Setup");
  Serial.println(0x04);
  Serial.print("Size of Data Struct <Bytes>:\t");
  Serial.println(sizeof(i2c_packet)); 
}

void loop() {
  // put your main code here, to run repeatedly:
  reading = (double)random();
  pkt.value= reading;
  Serial.println(pkt.value);
}

//--------------------------------------------------------------------------------------------------
/*
I2C interrupt function defintion
  Every time the device is requested by the main, this function interrupts the infinite loop
  to reply to the I2C request

  - this writes the entire i2c_packet to the line and ends the transmission
*/
void i2c_handler()
{
  Wire.write((uint8_t*)&pkt, sizeof(i2c_packet));
}
