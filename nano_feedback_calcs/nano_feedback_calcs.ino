#include <CircularBuffer.h>
//addition to dylan_hall_effect_speed_read.ino script
 // adding capability to filter the readings and calc the accelerations
 
#include <Wire.h>
 
 int analog_pin = A1;

 volatile double timestep = 0.01; //s   slower will yeild us greater accuarcy due to rounding errors, also the hall sensor doesnt run any better ive found
 volatile double rot_v = 0;
 volatile double linear_v = 0;
 volatile double smooth_v = 0;

#define MA_LENG 5
volatile CircularBuffer <double, MA_LENG> ma;    // circular buffer (LIFO queue)
 //set up for the acc calc
 volatile double last_vel = 0;
 volatile double new_vel = 0;
 volatile double acc = 0;

 typedef struct {
   int last;
   int now;
 } Rot_t;   // rotational data structure

 typedef struct{
    double velocity;
    double acceleration;
 } Data_t ; // data structure that is broadcasted to the I2C line on request from the main

 volatile Rot_t cart = {
   .last = 0,
   .now = 0,
 };

volatile Data_t data = {.velocity = 0.0, .acceleration = 0.0};

 double eval_velocity(Rot_t* state)  {
   state->last = state->now;
   state->now = analogRead(analog_pin);
   double diff = 0;
   diff = state->last - state->now;
   double HallChange = diff ;    // I deleted the timestep correction part from here and moved it to the bottom
    // completely analog quadrant change fix
   if(HallChange >= 357)      // 357 is close to 715/2, yeiled best results forwards and backwards 
   {
     HallChange = HallChange - 715;
   }
   else if(HallChange <= -357)
   {
     HallChange=HallChange + 715;
   }

   return HallChange;    //this is giving us the chnage in the hall effect sensor readings 
 }


double ma_update(double val, CircularBuffer<double, MA_LENG> *buff)
{
  buff->push(val);
  double sum = 0;
  for (int i =0; i< MA_LENG; i++) sum += (*buff)[i];
  return sum / MA_LENG; 
}

 double acc_calc(double smooth_v){
   last_vel = new_vel;
   new_vel = smooth_v;
   acc = (new_vel - last_vel);
   return acc;
   
 }

int interruptPin = 2; // interrupt pin (RX0)

 void setup() {
   // put your setup code here, to run once:
   Serial.begin(9600);
   Serial.println("started");
   pinMode(interruptPin, INPUT);    // attempt to setup interrupt on pin 2 (Not Nano Every in datasheet)
   delayMicroseconds(50);
   attachInterrupt(digitalPinToInterrupt(interruptPin), calc_ISR, FALLING);  //Attempt to attach interrupt to pin
   Wire.begin(0x04);                  // I2C Address 
   Wire.onRequest(i2c_onReq);         // set Wire I2C onRequest handler function 
 }

 void loop() {
  calc_ISR(); // run calculations
              // if interrupt starts working, comment all of this loop out
  Serial.print(data.velocity);
  delay(1000*timestep);
 }

 void i2c_onReq()
 {
     // type casting magic
  // struct -> byte[]
  // struct -> *
  // (byte*)(struct *) -> chops structure pointer into byte size array
  // &data -> give pointer reference to type case
  // byte* array of byte length sizeof(Data_t) -> gives length in # of bytes to write to the line
    // cast : (byte*)(structure*) -> (byte*) of size sizeof(structure_type); write each byte in that byte array to the Wire line
    Wire.write((byte*)&data, sizeof(Data_t));
   // Serial.println("I2C Data Served");
 };

/*
Calculation in Interrupt form
  - phantom 5V on interrupt pin causing issues (get a new nano)
*/
 void calc_ISR()
 {
  
  rot_v = (eval_velocity(&cart)*(1/timestep)*.1678322);  // .1678322 = ((715/360)*.33333)  Wheel velocity in terms of degees per second 
   linear_v = ((rot_v/360)*.771*.745476186*1.1697); // v with correction factors
   smooth_v = ma_update(linear_v, &ma);   //circular buffer stuff
   //acc = acc_calc(smooth_v);            // acceleraton calculation
   data.velocity = smooth_v;
   data.acceleration = 0;//acc/timestep;
   
 };