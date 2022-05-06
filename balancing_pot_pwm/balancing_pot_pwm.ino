//balancing on flat ground with potentiometer and pwm feedback
//vectorized error
//made 4/26/22

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
    double motor_comm_mag = 0;
    int motor_comm_dir = 0;
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


//-------------------------------------------------------------------------------------------------------------------------------
/*
  Control Program initialization
 */
//initialize the controller
BLA::Matrix<1,2> kp = {0.1,20};         //first: pwm
BLA::Matrix<1,2> ki = {0,0};          //second: angle
BLA::Matrix<1,2> kd = {0,0};
BLA::Matrix<2,1> setpoint = {10,0};  //pwm setpoint, angle setpoint (deg)
double ts = 0.01;                     //timestep for controller, s (for deriv)

Controller flat_c (kp, ki, kd, setpoint, ts);


//potentiometer stuff
int pot_pin = A0;

//get angle in degrees from potentiometer reading
double angle()  {
  int reading = analogRead(pot_pin);
  double ang = -26 + (reading - 377) * 0.26;
  return ang;
}

// error calculation
void error_calc(double theta,Controller* cont){
  BLA::Matrix<2,1> reading = {cont->motor_comm_mag*cont->motor_comm_dir,-theta};    //check theta calc
  
  cont->error_last = cont->error;
  cont->error = cont->setpoint - reading;

  cont->error_deriv = (cont->error - cont->error_last) / cont->ts;
  cont->error_int = cont->error_int + (cont->error * cont->ts);
}

void eval_pid(Controller* cont){
  double pv = cont->kp(0,0) * cont->error(0);
  double pt = cont->kp(0,1) * cont->error(1);
  double iv = cont->ki(0,0) * cont->error_int(0);
  double it = cont->ki(0,1) * cont->error_int(1);
  double dv = cont->kd(0,0) * cont->error_deriv(0);
  double dt = cont->kd(0,1) * cont->error_deriv(1);
  cont->command = pv + pt + iv + it + dv + dt;
}

int dir_pin = 13;
int pwm_pin = 11;
int motor_lim = 150;

//send command to motor
// writes command to the motor (PWM in set of [0,255], limits to motor_lim variable ( usually like 130))
void motor_controller(Controller* cont ) {
   int comm = cont->command;
   cont->motor_comm_mag = abs(comm) <= motor_lim ? abs(comm): motor_lim;
   cont->motor_comm_dir = comm > 0 ? 1 : -1;
   digitalWrite(dir_pin, comm > 0 ? 1 : 0);
   analogWrite(pwm_pin, abs(comm) <= motor_lim ? abs(comm): motor_lim);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  error_calc(angle(),&flat_c);
  eval_pid(&flat_c);
  motor_controller(&flat_c);
  delay(ts*1000);
}
