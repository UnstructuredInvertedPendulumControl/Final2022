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

i2c_packet pkt; /* Global Structure variable defintion */

//--------------------------------------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Wire.begin();  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  grab_data();
  
  Serial.print(pkt.value);
  Serial.print("\t");
  Serial.print(pkt.value2);
  Serial.println();
}
//-------------------------------------------------------------------------------------------------
/* 
grab_data Function
- requests I2C device on the Wire.h I2C interface
- reads incoming bytes into a byte array buffer or global structure variable
- ends the I2C transmission on the Wire.h I2C interface
*/
void grab_data()
{
  byte buffer[sizeof(i2c_packet)];          /*Either write to a buffer and type cast or global variable and cast it to byte and back to struct */
  memset(&buffer, 0, sizeof(i2c_packet));
  Wire.requestFrom(0x04, sizeof(i2c_packet));
  Wire.readBytes((byte*)&pkt, sizeof(i2c_packet));
  Wire.endTransmission();
};
