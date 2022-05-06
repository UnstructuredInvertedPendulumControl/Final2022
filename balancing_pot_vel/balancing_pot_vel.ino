//balancing on flat ground with potentiometer and velocity feedback
//vectorized error
//made 4/25/22


#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;


//-------------------------------------------------------------------------------------------------------------------------------
/*
    Controller Class
      PID Control
      Vectorized Error for multiple things
      Same form factor and use case as Acc's structure 
 */
struct Controller{
    BLA::Matrix<2,1> error = {0,0};         //top: velcoity
    BLA::Matrix<2,1> error_last = {0,0};    //bottom: angle
    BLA::Matrix<2,1> error_int = {0,0};
    BLA::Matrix<2,1> error_deriv = {0,0};
    BLA::Matrix<1,2> kp;
    BLA::Matrix<1,2> ki;
    BLA::Matrix<1,2> kd;
    BLA::Matrix<2,1> setpoint;
    double command = 0;
    double ts;
    Controller (

      BLA::Matrix<1,2> kp,
      BLA::Matrix<1,2> ki,
      BLA::Matrix<1,2> kd,
      BLA::Matrix<2,1> setpoint,
      double ts
    ):
    kp(kp),
    ki(ki),
    kd(kd),
    ts(ts){};
};

//initialize the controller
BLA::Matrix<1,2> kp = {1,25};
BLA::Matrix<1,2> ki = {0,0};
BLA::Matrix<1,2> kd = {0,0};
BLA::Matrix<2,1> setpoint = {0.2,0};
double ts = 0.01;        //timestep for controller, s (for deriv)

Controller flat_c (kp, ki, kd, setpoint, ts);

//motor feedback 
//-------------------------------------------------------------------------------------------------------------------------------
/*
  Read from Arduino Nano - see I2C Main/Branch examples for more info
 */
typedef struct{
    double velocity;
    double acceleration;  //not used
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
  Read from Arduino Nano - see I2C Main/Branch examples for more info
 */
void read_rdata(RotData_t* target)
{
  ch(4);
//  Serial.print(43);
//  Serial.print("\t");
  Wire.requestFrom(0x04, sizeof(RotData_t));
  Wire.readBytes((byte*)target, sizeof(RotData_t));
  Wire.endTransmission();
}

RotData_t rdata;

//potentiometer stuff
int pot_pin = A0;

//get angle in degrees from potentiometer reading
double angle()  {
  int reading = analogRead(pot_pin);
  double ang = -26 + (reading - 377) * 0.26;
  return ang;
}


void error_calc(double theta,RotData_t* nano_ref,Controller* cont){
  BLA::Matrix<2,1> reading = {-nano_ref->velocity,-theta};    //velocity direciotn is wrong from nano
  
  cont->error_last = cont->error;
  cont->error = cont->setpoint - reading;

  cont->error_deriv = (cont->error - cont->error_last) / cont->ts;
  cont->error_int = cont->error_int + (cont->error * cont->ts);
}

void eval_pid(Controller* cont){
  double pv = cont->kp(0) * cont->error(0);
  double pt = cont->kp(0,1) * cont->error(1);
  double iv = cont->ki(0) * cont->error_int(0);
  double it = cont->ki(0,1) * cont->error_int(1);
  double dv = cont->kd(0) * cont->error_deriv(0);
  double dt = cont->kd(0,1) * cont->error_deriv(1);
  cont->command = pv + pt + iv + it + dv + dt;
}

int dir_pin = 13;
int pwm_pin = 11;
int motor_lim = 150;

//send command to motor
void motor_controller(Controller* cont ) {
  int comm = cont->command;
   digitalWrite(dir_pin, comm > 0 ? 1 : 0);
   analogWrite(pwm_pin, abs(comm) <= motor_lim ? abs(comm): motor_lim);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  Wire.begin();
  memset(&rdata, 0, sizeof(RotData_t));
  Wire.setClock(100000);
}

void loop() {
  // put your main code here, to run repeatedly:
  read_rdata(&rdata);
  error_calc(angle(),&rdata,&flat_c);
  eval_pid(&flat_c);
  motor_controller(&flat_c);
  delay(ts*1000);
}
