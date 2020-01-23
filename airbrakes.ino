//*TODO*
/*	1. Ensure vertical velocity function is working properly
		2. Check units of all sensors
		2. Modularize functions
*/


// CHECK UNITS OF ALL SENSORS TO ENSURE WE HAVE THE RIGHT UNITS!

#define g 9.80665 //acceleration of gravity (m/s^2)
#define airbrakeMotor 8 //pin to control motor
#define halfwayToApogee 1524 //1524m = 5000ft
#define apogeeHeight 3048 //3048m = 10,000ft
#include<stdio.h>
#include <math.h> //for trig functions
#include <PID_v1.h> //PID library
#include <Servo.h> //library to control airbrake servo motor
#include <Wire.h>
#include <Adafruit_MPL3115A2.h> //Altimeter library
#include <Adafruit_Sensor.h> //General sensor library for Adafruit
#include <Adafruit_BNO055.h> //Gyro
#include <utility/imumaths.h>//for trig

//The function instantiates a new Gyroscope class with name bno
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//instantiates new barometer class with name baro
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

//Define variables as global - could use pointers for memory reduction
double Setpoint, Input, Output, alt, v, a, m, rawv,rawa, previous,Cd,theta;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, REVERSE);

/* Arduino automatically does function prototypes
double getDrag(a,v); //Prototype
double getProjectedAltitude(Cd, v, alt); //Prototype
double VerticalAcceleration(); //Prototype
double VerticalVelocity(); //Prototype
double getAngleToVertical(); //Prototype
*/
void setup() {
  Servo esc; //declare electronic speed controller
  
	pinMode(7,INPUT);//For sensors?
	pinMode(8,INPUT);//For senors?

  esc.attach(airbrakMotor); //connects signal wire to pin 8 to control the motor
  bno.setExtCrystalUse(true); //use external crystal boolean (keeps track of time
 
	double apogee=apogeeHeight+baro.getAltitude(); //3048 meters=10,000 ft. this variable will never change, it is our setpoint (where we want the rocket to nose over at)
  Setpoint=apogee; // This should be correct, this way the rocket is always shooting for exactly 10k above ground

  // PID Set-Up - Default max output is 255, Default were specified above @ 2,5,1
  // We should compare these to see how they fare, collecting data of projected altitude vs. setpoint over time
  // Default sample time is 200ms, which should be sufficient, tuning parameters matter more

//This next chunk of code waits until after motor-burnout to start the airbrakes algorithm
// We could probably do this with while(alt-Setpoint<5000) {alt=baro.getAltitude}

  while(alt-Setpoint < halfwayToApogee) {//1524meters = 5,000ft (half-way to apogee)
    v=VerticalVelocity();
    a=VerticalAcceleration();
    alt=baro.getAltitude;//keep measuring the altitude till motor burnout 
  }

  Cd = getDrag(a,v);
  double alt = baro.getAltitude();
  Input = getProjectedAltitude(Cd, v, alt);
  // turn the PID on
  myPID.SetMode(AUTOMATIC);
}
void loop() {
  // Setpoint is permanently set, so all we need is input

  /* Get a new sensor event */
  // Sensors
  sensors_event_t event; //initializing struct of type sensors_event_t
  bno.getEvent(&event); //storing the sensor event in class event (e.g get data)
  rawa = event.acceleration.z; //get z acceleration *Z IS WITH RESPECT TO GYRO NOT ROCKET

  //*TODO* get raw velocity and store in rawv
  v=VerticalVelocity(rawv,event);//computes velocity with respect to true vertical
  
	//throw these next three lines in a UDF entitled getRead();
	rawa = event.acceleration.z;
  a=VerticalAcceleration(rawa, event);
  alt = baro.getAltitude();
  Cd=getDrag(a,v);
  double Input = getProjectedAltitude(Cd, v, alt);//Input is projected alt
  
	// our error is: error = Setpoint - Input;
  // this computes the PID and stores the needed output as global variable <Output>
  myPID.Compute();//have the PID compute the proper output

  // Probably should set an analog out limit in the PID software
	//write the computed output to the airbrakeMotor pin
	analogWrite(airbrakeMotor,Output); }

/*
This function takes two inputs, the vertical acceleration (a), and the 
velocity (v).

Using these two variables, it will compute the approximate drag coefficient 
of the rocket in real time.  

Because drag depends on how fully extended the airbrake flaps are, we must
continuously calculate drag as we make corrections based off of our 
projected altitude.
*/
double getDrag(double a, double v) {
  double c = (a-m*g)/(v*v); // from mg+cv^2=a, we rearrange to solve for c
  return c;
}

/*
This function takes three inputs:
Cd  = the drag coefficient (computed by getDrag)
v   = the true vertical velocity of the rocket (relative to earth)
alt = the current altitude of the rocket

Using the kinematic equations and integration tables, a formula calculates what 
the projected altitude of the rocket is, if the airbrake deployment level were to
remain the same.

This function returns the projected altitude, which is used by the PID to compare to our goal apogee, allowing the airbrake motor to make the corrections.
*/
double getProjectedAltitude(double Cd,double v,double alt) {
  double y = (m/Cd)*log(cos(atan(v/sqrt(m*g/Cd)))); // Calculate additional altitude projected
  double ypro = y + alt;  // Projected altitude is the additional + current alt
  return ypro;
}

/*
Computes vertical acceleration given rawacc (raw acceleration) which is
the acceleration relative to the z-axis
OF THE ROCKET (we are concerned with the z-axis that is relative to earth)
event is a structure of type sensors_event_t that is passed to getAngletoVertical
*/
double VerticalAcceleration(double rawacc,sensors_event_t event) {//event is a struct of type sensors_event_t
  double theta = getAngleToVertical(event); //get from hardware
  //using spherical coordinates
  double a = cos(theta)*rawacc;
  return a;
}

/*
rawvelocity is the velocity in the z direction OF THE GYROSCOPE NOT THE ROCKET
event is a structure of type sensors_event_t that is passed to getAngletoVertical
*/
double VerticalVelocity(double rawvelocity, sensors_event_t event) {
  double theta = getAngleToVertical(event); //get from hardware
  //using spherical coordinates
  double v = cos(theta)*rawvelocity;
  return v;
}

/*
Using spherical coordinates, get angle of rocket to vertical
event is a structure of type sensors_event_t which holds information from the
gyro sensor used to derive the angles
*/
double getAngleToVertical(sensors_event_t event) {
  // hardware read
  double x=event.orientation.x;//get angle to x
  double y=event.orientation.y;//get angle to y
  double z=event.orientation.z;//get angle to z
	
	// conversion to spherical
  double theta=atan(y/x);//rectangular to spherical conversion
  return theta;
}
