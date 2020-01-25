// Joshua Köhler - 1/24/20 - Clark Aerospace - ESRA 2020 - Airbrakes Software

//*TODO*
/*	1. Add real mass of rocket in the define statement
		2. Check units of all sensors
		3. Modularize functions

Also a few optimization decisions to make:
1.	Should each get____ function get a new event, or should they all be passed
		the same event for consistency?

2.	Is using the system time accurate enought to compute velocity with the
		altimeter?

3.	If #2 is good, should we use the same method to compute accleration instead
		of the accelerometer?

4.	Consider the system timing of the PID, and how using our own timing in the 
		velocity function could affect this.

5.  Add code to turn off the airbrakes after a successful flight

6. Make sure the max for the analogWrite, hardware limit switches, and PID limit		are all in sync.  Default PID limit should be 255 (2^8 -1)

*/

// CHECK UNITS OF ALL SENSORS TO ENSURE WE HAVE THE RIGHT UNITS!
#define gravity 9.80665 //acceleration of gravity (m/s^2)
#define mass 60 //mass of the rocket *TODO* THIS MUST BE CHANGED TO BE ACCURATE!
#define airbrakeMotor 8 //pin to control motor
#define halfwayToApogee 1524 //1524m = 5000ft
#define apogeeHeight 3048 //3048m = 10,000ft
#include <stdio.h>
#include <unistd.h> //sleep function
#include <math.h> //for trig functions
#include <PID_v1.h> //PID library
#include <time.h> //used to get system time for calculations
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

//Define variables as  - could use pointers for memory reduction
double Setpoint, Input, Output; //global variables 
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, REVERSE);

// Arduino automatically does function prototypes

void setup() {//all code in the setup function is run only once
  Servo esc; //declare electronic speed controller
  
	pinMode(7,INPUT);//For sensors?
	pinMode(8,INPUT);//For senors?

  esc.attach(airbrakeMotor); //connects signal wire to pin 8 to control the motor
  bno.setExtCrystalUse(true); //use external crystal boolean (keeps track of time
 
	Setpoint = apogeeHeight+baro.getAltitude(); //3048 meters=10,000 ft. this variable will never change, it is our setpoint (where we want the rocket to nose over at)
// This should be correct, this way the rocket is always shooting for exactly 10k above ground

  // PID Set-Up - Default max output is 255, Default were specified above @ 2,5,1
  // We should compare these to see how they fare, collecting data of projected altitude vs. setpoint over time
  // Default sample time is 200ms, which should be sufficient, tuning parameters matter more

//This next chunk of code waits until after motor-burnout to start the airbrakes algorithm
// We could probably do this with while(alt-Setpoint<5000) {alt=baro.getAltitude}
	double alt=0;//initialize alt to force while loop
  while(alt-Setpoint < halfwayToApogee) {//1524meters = 5,000ft (half-way to apogee)
    alt=baro.getAltitude();//keep measuring the altitude till motor burnout 
  }
	myPID.SetMode(AUTOMATIC);//turn the PID on
}

/*
This main function loop has three steps
1. Read sensors, and calculate the Input variable for the PID compute.
2. Compute Output with the PID
3. Write this output to the airbrakes motor
*/
void loop() {
  setInput();//set Input for PID compute

  myPID.Compute();//have the PID compute the proper output

  // Probably should set an analog out limit in the PID software
	//write the computed output to the airbrakeMotor pin
	analogWrite(airbrakeMotor,Output);
}


/*
Gets all readings to calculate the Input for the PID.
Note that Input is a global variable, so the PID function can access it at 
all times.
*/
void setInput(void){
	sensors_event_t event; //initialize for gyro read
	double alt,v,Cd;
	bno.getEvent(&event);//get data from the gyro
  alt = baro.getAltitude();//get data from altimeter
	v = getVelocity();//get velocity from altimeter THIS PAUSES THE ARDUINO
  Cd = getDrag(event,v);
  Input = getProjectedAltitude(Cd, v, alt);//calculate Input to PID
}
/*
This function takes two inputs, an event of type sensors_event_t
for the gyro sensor, and the velocity (v).

Using these two variables, it will compute the approximate drag coefficient 
of the rocket in real time.  

Note that getAcceleration(event) is called inside of get drag in order to
compute the vertical acceleration.

Because drag depends on how fully extended the airbrake flaps are, we must
continuously calculate drag as we make corrections based off of our 
projected altitude.
*/
double getDrag(sensors_event_t event,double v) {
	double a = getAcceleration(event);
  double c = (a-mass*gravity)/(v*v); // from mg+cv^2=a, we rearrange to solve for c
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
  double y = (mass/Cd)*log(cos(atan(v/sqrt(mass*gravity/Cd)))); // Calculate additional altitude projected
  double ypro = y + alt;  // Projected altitude is the additional + current alt
  return ypro;
}

/*
Computes vertical acceleration relative to earth.
event.acceleration.z gives the vertical acceleration relative to the 
NOSE OF THE ROCKET (we are concerned with the z-axis that is relative to earth)
The angle theta is used to compute the vertical acceleration relative to earth.
event is a structure of type sensors_event_t that is passed to getAngletoVertical
*/
double getAcceleration(sensors_event_t event) {//event is a struct of type sensors_event_t
  double theta = getAngleToVertical(event); //get from hardware
  //using spherical coordinates
  double a = cos(theta)*event.acceleration.z;
  return a;
}

/*
This function uses the difference between two altimeter readings, with a timed
pause in-between, to calculate velocity.  We are not sure how accurate this will
be.
*/
double getVelocity(void) {
	time_t start,end;
	double alt,theta,velocity;
	start = time(NULL); //get begin time
	alt = baro.getAltitude();//get first altitude reading 
	delay(100);//delay for 100ms
	alt = baro.getAltitude()-alt;//2nd reading - 1st reading is the difference in altitude over the time period
	end = time(NULL);//get finish time
	velocity=alt/(end-start);//velocity is distance/time
  return velocity;
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
	
	// conversion to spherical
  double theta=atan(y/x);//rectangular to spherical conversion
  return theta;
}
