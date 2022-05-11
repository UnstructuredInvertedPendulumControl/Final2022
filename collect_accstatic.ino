//data collection for static calibration of body acceleromter
//Read sensor data and print to serial monitor
//Values recorded: raw acceleromter readings for body accelerometer
//Use with collect_accstatic_to_csv.py

//Author: Alex Boehm
//created Spring 2022 for WIP senior design


#include <Wire.h>
#include <ADXL345.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

/////////////////////////////////////////////////////////////////////////////////////////
//define data structures
struct Acc{
    uint8_t channel;
    ADXL345* dev_ref;
    BLA::Matrix<3,1> reading; //raw reading
    Acc
    (
      uint8_t ch, 
      ADXL345* dev_ref 
    ) : 
      dev_ref(dev_ref)
      {
      this->channel = ch;
      };
};

/////////////////////////////////////////////////////////////////////////////////////////
//setup I2C communication

//connect to I2C
void ch(uint8_t i){
    if (i > 7) return;    
    Wire.beginTransmission(0x70);
    Wire.write(1 << i);
    Wire.endTransmission();  
}

///////////////////////////////////////////////////////////////////////////////////////////
//ADXL setup and functions

//turn on ADXL device
void adxl_setup(Acc* dev_ref) {  
    ch(dev_ref->channel);    
    ADXL345 adxl = *(dev_ref->dev_ref);
    adxl.powerOn();
    adxl.setRangeSetting(2);
    adxl.setMode(ADXL345_MODE_BYPASS);
};

//read and calibrate ADXL data
void read_adxl(Acc* dev_ref)  {
  int x, y, z;
  ADXL345 adxl = *(dev_ref->dev_ref);
  ch(dev_ref->channel);
  adxl.readXYZ(&x, &y, &z);   //read raw output
  dev_ref->reading(0) = x;
  dev_ref->reading(1) = y;
  dev_ref->reading(2) = z;
 };


//adxl print function
void print_adxl(Acc* dev_ref)  {
  for (int i = 0; i < 2; i++) {
      Serial.print(dev_ref->reading(i)); Serial.print(",");
  }
}


//reserve memory for ADXL class
ADXL345 accelerometer_ctl;

//initialize accelerometer structures
Acc device_lower(3,&accelerometer_ctl);
Acc device_upper(2,&accelerometer_ctl); 
Acc device_body(1,&accelerometer_ctl);


///////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Wire.begin();
  adxl_setup(&device_body);
}

void loop() {
  //read ADXLs
  read_adxl(&device_body);

  //print adxls
  print_adxl(&device_body);
}
