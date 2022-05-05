//WIP data collection
//Read sensor data and print to serial monitor
//Values recorded: calibrated accelerometer values, motor feedback (accelerometer and velocity)
//  potentiometer angle reading, calculated incline (beta) angle, and calculated pendulum (theta) angle
//Use with collect_data_to_csv.py

//Author: Alex Boehm
//created Spring 2022 for WIP senior design

#include <Wire.h>
#include <ADXL345.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

/////////////////////////////////////////////////////////////////////////////////////////
//define data structures
struct Acc{
    uint8_t channel;
    ADXL345* dev_ref;
    BLA::Matrix<3,1> reading; //raw reading
    BLA::Matrix<2,1> calib;   //calibrated value
    BLA::Matrix<2,3> sens;    //sensitivty matrix
    BLA::Matrix<2,1> offset;  //offset vector
    Acc
    (
      uint8_t ch, 
      BLA::Matrix<2,3> sens,
      BLA::Matrix<2,1> offset,
      ADXL345* dev_ref 
    ) : 
      dev_ref(dev_ref),
      sens(sens),
      offset(offset) 
      {
      this->channel = ch;
      };
};

typedef struct{
    double velocity;
    double acceleration;
 } RotData_t ;

/////////////////////////////////////////////////////////////////////////////////////////
//setup I2C communication for motor feedback (from the Nano)
//Nano should have [filename] uploaded

//connect to I2C
void ch(uint8_t i){
    if (i > 7) return;    
    Wire.beginTransmission(0x70);
    Wire.write(1 << i);
    Wire.endTransmission();  
}

//read from I2C
void read_rdata(RotData_t* target)
{
  ch(4);
  Wire.requestFrom(0x04, sizeof(RotData_t));
  Wire.readBytes((byte*)target, sizeof(RotData_t));
  Wire.endTransmission();
}

//initialize structure
RotData_t rdata;

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
  dev_ref->calib = dev_ref->sens * dev_ref->reading + dev_ref->offset;  //calibrates reading
};

//reserve memory for ADXL class
ADXL345 accelerometer_ctl;

//calibration sensitivity and offsets
//rows set so that forward is +x and up is +y (only planar)
Matrix<2,3> Body_sens = {0.038501,0.002046,-0.000339,0.000232,0.000092,0.040023};
Matrix<2,1> Body_offset = {-0.114553,0.065328};
Matrix<2,3> Lower_sens = {0.000861,0.038687,0.005476,-0.037089,0.001532,-0.000387};
Matrix<2,1> Lower_offset = {-0.249873,0.605172};
Matrix<2,3> Upper_sens = {0.000306,0.038034,-0.000722,-0.037326,0.000868,0.000102};
Matrix<2,1> Upper_offset = {-0.447562,0.543993};

//initialize accelerometer structures
Acc device_lower(
  3, 
  Lower_sens, 
  Lower_offset, 
  &accelerometer_ctl);
Acc device_upper(
  2,
  Upper_sens,
  Upper_offset,
  &accelerometer_ctl); 
Acc device_body(
  1,
  Body_sens,
  Body_offset,
  &accelerometer_ctl);


/////////////////////////////////////////////////////////////////////////////////////////
//potentiometer read function

//intialize angle (deg)
double pot_ang = 0;

//connection pin
int pot_pin = A0;

//get angle in degrees from potentiometer reading
double angle()  {
  int reading = analogRead(pot_pin);
  double ang = -26 + (reading - 377) * 0.26;
  return ang;
}

/////////////////////////////////////////////////////////////////////////////////////////
//error processing and angle calcs

//initialize angles
double theta = 0;
double beta = 0;

//distance matrices for theta calculation (according to Cole Wood's paper)
Matrix <2,2> Drm = {0,-0.305,-0.305,0};        //mean distance
Matrix <2,2> inv_Drd = {0,-2.8571,-2.8571,0};  //distance difference (inverse)

//calculate pendulum angle (rad)
double theta_calc(Acc* dev_ref_upper, Acc* dev_ref_lower, Acc* dev_ref_body){  
  //calc mean and difference in acc readings on pendulum
  Matrix<2,1> am, ad;
  am = dev_ref_upper->calib + dev_ref_lower->calib;
  am *= 0.5;
  ad = dev_ref_upper->calib - dev_ref_lower->calib;
  
  //calculate acc of cart in pendulum coord sys
  Matrix<2,1> aO = am - Drm*inv_Drd*ad;

  //acc of cart in cart coord sys
  Matrix<2,1> a3 = dev_ref_body->calib; 

  //calc angle between cart and pend coord systems
  double cross = a3(0)*aO(1) - a3(1)*aO(0);
  double dot = a3(0)*aO(0) + a3(1)*aO(1);
  double theta = atan2(cross,dot);
  return theta;
}

//calculate incline angle (rad
double beta_calc(Acc* dev_ref_body,RotData_t* target){  
  //subtract dynamic acc (motor feedback) from acc measured by ADXL on cart. Acc left is gravitational
  //find angle of gravitaitonal acc
  double beta = atan2(dev_ref_body->calib(0) - target->acceleration,dev_ref_body->calib(1));
  return beta;
}

//////////////////////////////////////////////////////////////////////////////////////////
//printing funcitons

//adxl print function
void print_adxl(Acc* dev_ref)  {
  for (int i = 0; i < 2; i++) {
      Serial.print(dev_ref->calib(i)); Serial.print(",");
  }
}

//print all output to serial monitor
void print_all(Acc* dev_ref_upper, Acc* dev_ref_lower, Acc* dev_ref_body,RotData_t* target,double pot,double theta,double beta){
  print_adxl(dev_ref_upper);
  print_adxl(dev_ref_lower);
  print_adxl(dev_ref_body);
  Serial.print(target->velocity);
  Serial.print(",");
  Serial.print(target->acceleration);
  Serial.print(",");
  Serial.print(pot);
  Serial.print(",");
  Serial.print(theta*180/M_PI);
  Serial.print(",");
  Serial.print(beta*180/M_PI);
  Serial.println(",");
}

///////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Wire.begin();
  memset(&rdata, 0, sizeof(RotData_t));
  adxl_setup(&device_lower);
  adxl_setup(&device_upper);
  adxl_setup(&device_body);
}

void loop() {
  //read ADXLs
  read_adxl(&device_upper);
  read_adxl(&device_lower);
  read_adxl(&device_body);

  //read motor feedback (from Nano)
  read_rdata(&rdata);

  //read potentiometer angle (deg)
  pot_ang = angle();

  //calculate angles (rad)
  theta = theta_calc(&device_upper,&device_lower,&device_body);
  beta = beta_calc(&device_body,&rdata);

  //print readings to serial monitor
  print_all(&device_upper,&device_lower,&device_body,&rdata,pot_ang,theta,beta);
}
