// CHECK UNITS OF ALL SENSORS TO ENSURE WE HAVE THE RIGHT UNITS!
#define airbrakeMotor 8 //pin to control motor
#include<stdio.h>
#include <math.h> //for trig functions
#include <PID_v1.h> //PID library
#include <Servo.h>
// ADDED IN LIBRARIES FOR SENSORS
#include <Wire.h>
#include <Adafruit_MPL3115A2.h> //Altimeter library
#include <Adafruit_Sensor.h> //General sensor library for Adafruit
#include <Adafruit_BNO055.h> //Gyro
#include <utility/imumaths.h>
//Declare bno as class Adafruit_BNO055
//Assign bno to result of function Adafruit_BNO055(int32_t sensorID,...)
//The function instantiates a new Adafruit_BNO055 class
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//instantiates new class of barometer with name baro
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
// END OF ADDED IN LIBRARIES FOR SENSORS
//Define Variables we'll be connecting to
double Setpoint, Input, Output, alt, v, a, m, rawv,rawa, previous,Cd,theta;
int previousOutput = 0;
double g=9.80665; // (m/s^2)
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, REVERSE);
/* Arduino automatically does function prototypes
double Drag(a,v); //Prototype
double ProjectedAltitude(Cd, v, alt); //Prototype
double VerticalAcceleration(); //Prototype
double VerticalVelocity(); //Prototype
double sphericalThetaRead(); //Prototype
*/
void setup() {
  //LSM9DS1 etup
 //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G); //Acceleromter Range
 //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS); //Magnetometer Sensitivity
 // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);  //Gyroscope
  Servo esc; //declare electronic speed controller
  pinMode(7,INPUT);
  pinMode(8,INPUT);
  esc.attach(8); //connects signal wire to pin 8 to control the motor
  bno.setExtCrystalUse(true); //use external crystal boolean (keeps track of time?)
  //m = MASS OF THE ROCKET AFTER BURNOUT;
  double apogee=3048+baro.getAltitude(); //3048meters=10,000 ft.
  // PID Set-Up - Default max output is 255, Default were specified above @ 2,5,1
  // We should compare these to see how they fare, collecting data of projected altitude vs. setpoint over time
  // Default sample time is 200ms, which should be sufficient, tuning parameters matter more
  Setpoint=apogee; // This should be correct, this way the rocket is always shooting for exactly 10k above ground
  // We could probably do this with while(alt<5000) {alt=baro.getAltitude}
  /*
  while(motor == burning) {
    v=VerticalVelocity();
    a=VerticalAcceleration();
    //wait
  }
*/
  Cd = Drag(a,v);
  double alt = baro.getAltitude();
  Input = ProjectedAltitude(Cd, v, alt);
  // turn the PID on
  myPID.SetMode(AUTOMATIC);
}
void loop() {
  // Setpoint is permanently set, so all we need is input
  /* Get a new sensor event */
  //garbage code below
  //sensors_event_t a, m, g, temp;
  // Sensors
  sensors_event_t event; //initializing class event of type sensors_event_t
  bno.getEvent(&event); //storing the sensor event in class event (e.g get data)
  rawa = (event.acceleration.z, 4); //get z acceleration *Z IS WITH RESPECT TO GYRO NOT ROCKET
  //*TODO* get raw velocity and store in rawv
  v=VerticalVelocity(rawv,event);//computes velocity with respect to true vertical
  rawa = event.acceleration.z;
  a=VerticalAcceleration(rawa);
  alt = baro.getAltitude(); // Check units on this
  Cd=Drag(a,v);
  double Input = ProjectedAltitude(Cd, v, alt);
  // our error is: error = Setpoint - Input;
  // this computes the PID and stores the needed output as global variable <Output>
  myPID.Compute();
  // I need to ensure that the analogWrite adjusts properly
  // It seems the myPID.Compute() takes care of the output
  // This method of using this library should be tested with a similiar, non-rocket situation
  // to determine if it works
  // Probably should set an analog out limit in the PID software
  analogWrite(airbrakeMotor,Output); //airbrakeMotor needs to be defined THIS WILL ERROR UNTIL THAT IS ADDED
  // This may actually need to be:
  // analogWrite(airbrakeMotor,previous + Output);
  // previous = Output;
}
double Drag(double a, double v) {
  double c = (a-m*g)/(v*v); // from mg+cv^2=a, we rearrange to solve for c
  return c;
}
double ProjectedAltitude(double Cd,double v,double alt) {
  double y = (m/Cd)*log(cos(atan(v/sqrt(m*g/Cd)))); // Calculate additional altitude projected
  double ypro = y + alt;  // Projected altitude is the additional + current alt
  return ypro;
}
double VerticalAcceleration(double rawacc) {
  //double rawa = event.acceleration.z; //get from hardware
  double theta = sphericalThetaRead(event.orientation.x, event.orientation.y, event.orientation.z); //get from hardware
  //using spherical coordinates
  double a = cos(theta)*rawacc;
  return a;
}
double VerticalVelocity(double rawvelocity, sensors_event_t event) {
  //double rawv = event.velocity.z //get from hardware
  double theta = sphericalThetaRead(event.orientation.x, event.orientation.y, event.orientation.z); //get from hardware
  //using spherical coordinates
  double v = cos(theta)*rawvelocity;
  return v;
}
double sphericalThetaRead(double x,double y,double z) {
  // hardware read and calculations
  /*
  double x=event.orientation.x;
  double y=event.orientation.y;
  double z=event.orientation.z;
  */
  double r=sqrt(x*x+y*y+z*z);
  double theta=atan(y/x);
  //phi=atan(sqrt(x^2+y^2)/z);
  return theta;
}
