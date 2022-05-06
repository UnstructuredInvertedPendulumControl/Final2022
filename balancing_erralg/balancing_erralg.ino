//balancing on flat ground w/o vel control
//original error alg (no filter)
//created 4/13/22



#include <Wire.h>
#include <BasicLinearAlgebra.h>
// https://www.arduino.cc/reference/en/libraries/basiclinearalgebra/
#include <math.h>

// https://github.com/Seeed-Studio/Accelerometer_ADXL345/blob/master/examples/ADXL345_FIFO_demo_code/ADXL345_FIFO_demo_code.ino
#include <ADXL345.h>

using namespace BLA;
//-------------------------------------------------------------------------------------------------------------------------------
/*
  Acc class holds the data for each adxl sensor
    - takes in an Adxl345 object (just interfaces with the hardware)
    - stores result and post processing in this class for control usage
    - struct in C++ is a public class, so C struct initialization is iffy - view this as a class with all public data and members
        - not getters/setters
 */

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

//-------------------------------------------------------------------------------------------------------------------------------
/*
    Controller Class
      PID Control
      Vectorized Error for multiple things
      Same form factor and use case as Acc's structure 
 */
 struct Controller{
    BLA::Matrix<2,1> error = {0,0};
    BLA::Matrix<2,1> error_last = {0,0};    //last error calc
    BLA::Matrix<2,1> error_int = {0,0};     //integral of error
    BLA::Matrix<2,1> error_deriv = {0,0};   //derivative of error
    BLA::Matrix<1,2> kp;
    BLA::Matrix<1,2> ki;
    BLA::Matrix<1,2> kd;
    double command = 0;
    double ts;
    Controller (
      BLA::Matrix<1,2> kp,
      BLA::Matrix<1,2> ki,
      BLA::Matrix<1,2> kd,
      double ts
    ):
    kp(kp),
    ki(ki),
    kd(kd),
    ts(ts){};
};

//-------------------------------------------------------------------------------------------------------------------------------
/*
  I2C data packet from the Arduino Nano Hall Effect Post Processing Sensor
 */
 typedef struct{
    double velocity;
    double acceleration;
 } RotData_t ;

//-------------------------------------------------------------------------------------------------------------------------------
/*
 Muliplexer code
  @ 0x70
  bit shift one by i up to 7 times for 8 channels possible 0 inclusive
 */
void ch(uint8_t i){
    if (i > 7) return;    
    Wire.beginTransmission(0x70);
    Wire.write(1 << i);
    Wire.endTransmission();  
}
//-------------------------------------------------------------------------------------------------------------------------------
/*
  turn on the adxl sensor and configure its use case i.e register bitfields

 */
void adxl_setup(Acc* dev_ref)
{
    // ripped from seed studio examples
    // turn on ADXL device    
    ch(dev_ref->channel);    
    ADXL345 adxl = *(dev_ref->dev_ref);
    adxl.powerOn();
    adxl.setRangeSetting(2);
    adxl.setMode(ADXL345_MODE_BYPASS);
    //Serial.println("Finished Setting up ADXL345 Device");
};

//-------------------------------------------------------------------------------------------------------------------------------
/*
  Read the ADXL
 */
void read_adxl(Acc* dev_ref)
{
  int x, y, z;
  ADXL345 adxl = *(dev_ref->dev_ref);
  ch(dev_ref->channel);
  adxl.readXYZ(&x, &y, &z);   //read raw output
  dev_ref->reading(0) = x;
  dev_ref->reading(1) = y;
  dev_ref->reading(2) = z;
  dev_ref->calib = dev_ref->sens * dev_ref->reading + dev_ref->offset;  //calibrates reading
};

void printadxl(Acc* dev_ref)  {
  for (int i = 0; i < 2; i++) {
//    Serial.print(dev_ref->reading(i)); Serial.print(",");
      Serial.print(dev_ref->calib(i)); Serial.print(",");
  }
}

//-------------------------------------------------------------------------------------------------------------------------------
/*
  Read from Arduino Nano - see I2C Main/Branch examples for more info
 */
void read_rdata(RotData_t* target)
{
  ch(4);
  Wire.requestFrom(0x04, sizeof(RotData_t));
  Wire.readBytes((byte*)target, sizeof(RotData_t));
  Wire.endTransmission();
}

ADXL345 accelerometer_ctl;
RotData_t rdata;



//-------------------------------------------------------------------------------------------------------------------------------
/*
  Control Program initialization
 */

//calibration sensitivity and offsets
//rows set so that forward is +x and up is +y (only planar)
Matrix<2,3> Body_sens = {0.038501,0.002046,-0.000339,0.000232,0.000092,0.040023};
Matrix<2,1> Body_offset = {-0.114553,0.065328};
Matrix<2,3> Lower_sens = {0.000861,0.038687,0.005476,-0.037089,0.001532,-0.000387};
Matrix<2,1> Lower_offset = {-0.249873,0.605172};
Matrix<2,3> Upper_sens = {0.000306,0.038034,-0.000722,-0.037326,0.000868,0.000102};
Matrix<2,1> Upper_offset = {-0.447562,0.543993};

//initialize accelerometer objects
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


//initialize the controller
Matrix <1,2> kp = {25,0};  //gains for controller, first: x, second: y
Matrix <1,2> ki = {0.05,0};
Matrix <1,2> kd = {0.1,0};
double ts = 0.01;        //timestep for controller, s (for deriv)

Controller flat_c (kp, ki, kd, ts);


//pendulum angle calc
Matrix <2,2> Drm = {0,-0.305,-0.305,0};          //rm=0.5*(r1+r2)
Matrix <2,2> inv_Drd = {0,-2.8571,-2.8571,0};  //rd=(r1-r2)





//-------------------------------------------------------------------------------------------------------------------------------
/*
  Angle Estimation and Controller Implementation
 */

double theta_calc(Matrix <2,1> am, Matrix <2,1> ad, Matrix <2,1> a3){
  //calculate acc of cart in pendulum coord sys
  Matrix<2,1> aO = am - Drm*inv_Drd*ad;

  //calc angle between cart and pend coord systems
  double cross = a3(0)*aO(1) - a3(1)*aO(0);
  double dot = a3(0)*aO(0) + a3(1)*aO(1);
  double theta = atan2(cross,dot);
  return theta;
}



//potentiometer stuff
int pot_pin = A0;

//get angle in degrees from potentiometer reading
double angle()  {
  int reading = analogRead(pot_pin);
  double ang = -26 + (reading - 377) * 0.26;
  return ang;
}



//error alg
void error_alg(Acc* dev_ref_upper, Acc* dev_ref_lower, Acc* dev_ref_body,Controller* cont){
  
 //calc mean and difference in acc readings on pendulum
  Matrix<2,1> am, ad;
  am = dev_ref_upper->calib + dev_ref_lower->calib;
  am *= 0.5;
  ad = dev_ref_upper->calib - dev_ref_lower->calib;

  //calculate angle of pendulum and make rotation matrix
  double theta = theta_calc(am,ad,dev_ref_body->calib);
  Matrix <2,2> R = {cos(theta),-sin(theta),sin(theta),cos(theta)};
  Matrix <2,1> g = {0,9.81};    //acc due to grav

  //define errors  
  cont->error_last = cont->error;
  cont->error = R*am - g;
  cont->error_deriv = (cont->error - cont->error_last) / cont->ts;
  cont->error_int = cont->error_int + (cont->error * cont->ts);
}


//PID
void eval_pid(Controller* cont){
  double px = cont->kp(0) * cont->error(0);
  double py = cont->kp(0,1) * cont->error(1);
  double ix = cont->ki(0) * cont->error_int(0);
  double iy = cont->ki(0,1) * cont->error_int(1);
  double dx = cont->kd(0) * cont->error_deriv(0);
  double dy = cont->kd(0,1) * cont->error_deriv(1);
  cont->command = px + py + ix + iy + dx + dy;
}

//send command to motor
int dir_pin = 13;
int pwm_pin = 11;
int motor_lim = 90;

// writes command to the motor (PWM in set of [0,255], limits to motor_lim variable ( usually like 130))
void motor_controller(Controller* cont) {
   digitalWrite(dir_pin, cont->command > 0 ? 1 : 0);
   analogWrite(pwm_pin, abs(cont->command) < motor_lim ? abs(cont->command) : motor_lim);
}


//-------------------------------------------------------------------------------------------------------------------------------
/*
 run  the program
 */
void setup() {
  Serial.begin(9600);
  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  Wire.begin();
  memset(&rdata, 0, sizeof(RotData_t));

  Wire.setClock(100000);
//  Serial.println("Setting up Devices");
//  Serial.println("Setting up 1");
  adxl_setup(&device_lower);
//  Serial.println("Setting up 2");
  adxl_setup(&device_upper);
//  Serial.println("Setting up 3");
  adxl_setup(&device_body);
//  Serial.println("Finished Setup");
  delay(6000);

}


void loop()
{

  //flat balancing
  read_adxl(&device_upper);
  read_adxl(&device_lower);
  read_adxl(&device_body);
  error_alg(&device_upper,&device_lower,&device_body,&flat_c);
  eval_pid(&flat_c);
  motor_controller(&flat_c);
    
  delay(ts*1000);


}
