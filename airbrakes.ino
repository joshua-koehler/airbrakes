// Joshua Köhler - 1/24/20 - Clark Aerospace - ESRA 2020 - Airbrakes Software

/*
Note on units:
All units are in SI.
Acceleration: m/s^2
Altitude: m
Orientation: radians (converted to degrees in OUR code, but sensors gives in rad
*/

//*TODO*
/*	1. Add real mass of rocket in the define statement

Also a few optimization decisions to make:
1.	Get one event, and extrapolate all function data from this.

2.	Consider the system timing of the PID, and how using our own timing in the 
    velocity function could affect this.

3.	Add code to turn off the airbrakes after nose over (not mission-critical)

4.	Make sure the max for the analogWrite, hardware limit switches, and PID limit
    are all in sync.  Default PID limit should be 255 (2^8 -1)

5. Ensure PID will alternate between FWD (forward) and BWD (backward) by
   modifying library source code
*/

/*
if compiling C++ do the following:
#include "Arduino.h"
add function prototypes
*/

#define gravity 9.80665 //acceleration of gravity (m/s^2)
#define ACC_ID 55 //arbitrary id number of accelerometer sensor
#define mass 60 //mass of the rocket *TODO* CHANGE TO REAL MASS ONCE KNOWN
#define halfwayToApogee 1524 //1524m = 5000ft
#define apogeeHeight 3048 //3048m = 10,000ft
#include <unistd.h> //sleep function
#include <math.h> //for trig functions
#include <PID_v1.h> //PID library
#include <time.h> //used to get system time for calculations
#include <Servo.h> //library to control airbrake servo motor
#include <Wire.h>//I2C for both altimeter and accelerometer
#include <Adafruit_MPL3115A2.h> //Altimeter library
#include <Adafruit_Sensor.h> //General sensor library for Adafruit
#include <Adafruit_BNO055.h> //Accelerometer
#include <utility/imumaths.h>//for trig

/*
Pin connections map:
Note: both sensors can be connected to the same pins, I2C can support up to 121
devices simultaneously.

Accelerometer:
SCL - analog 5
SDA - analog 4
VDD - 3.3-5V DC
GROUND - GND

Barometric Altimeter:
SCL - Analog 5
SDA - Analog 4
Vin - 3.3-5V DC
GND - GND

Stepper motor:
FWD - analog 10
BWD - analog 11
GND - GND
GND - GND
*/

// Set the delay between fresh samples
uint16_t BO055_SAMPLERATE_DELAY_MS = 100;

//The function instantiates a new Gyroscope class with name bno
Adafruit_BNO055 bno = Adafruit_BNO055(ACC_ID);//ACC_ID is arbitary id

//instantiates new barometer class with name baro
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

double Setpoint, Input, Output; //global variables for PID

/*
Specify the links and initial tuning parameters
PID Set-Up - Default max output is 255, Default were specified above @ 2,5,1
We should compare these to see how they fare, collecting data of 
projected altitude vs. setpoint over time
Default sample time is 200ms, which should be sufficient, tuning parameters 
matter more.
*/
PID myPID(&Input, &Output, &Setpoint,2,5,1, REVERSE);

double velocity = 0;  //Initialize global velocity variable to 0

// Arduino automatically does function prototypes

void setup() {//all code in the setup function is run only once
	Serial.begin(115200);

	//Initialize and test accelerometer
	Serial.println("Orientation Sensor Test"); Serial.println("");
	if(!bno.begin()){
		//There was a problem detecting the BNO055 ... check your connections
		Serial.print("BNO055 not detected, check wiring or I2C address");
		while(1);//display error message forever
	}
	
	//Initialize and test altimeter
	Serial.println("Altimeter Test"); Serial.println("");
	if(!baro.begin()){
		//There was a problem detecting the alt ... check your connections
		Serial.print("Altimeter not detected, check wiring or I2C address");
		while(1);//display error message forever
	}


	delay(1000);//delay to display messages

	Servo esc; //declare electronic speed controller

	pinMode(7,INPUT);//altimeter 
	pinMode(airbrakeMotor,OUTPUT);//airbrakeMotor is pin 8

	esc.attach(airbrakeMotor); //connects signal wire to pin 8 to control the motor
	bno.setExtCrystalUse(true); //use external crystal boolean (keeps track of time
 
	/*
	3048 meters=10,000 ft. this 
	variable will never change, it is our setpoint (where we want the rocket to 
	nose over at)
	The PID is always aiming for 3048m
	*/
	Setpoint = apogeeHeight+baro.getAltitude(); 

	/*
	This next chunk of code waits until an arbitary altitude to start the airbrakes
	algorithm. More simulations should be done to optimize this with motor burn-out
	and effectiveness of airbrakes at that time. A preset pre-burnout braking
	algorithm depending on boost could be run before this PID code kicks in to
	improve performance.
	*/

	double alt=0;//initialize alt to force while loop
	//1524meters = 5,000ft (half-way to apogee)
	while(alt-Setpoint < halfwayToApogee) {
		alt=baro.getAltitude();//keep measuring the altitude 
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

	/* 
	Probably should set an analog out limit in the PID software
	write the computed output to the airbrakeMotor pin
	*/
	analogWrite(airbrakeMotor,Output);
}


/*
Gets all readings to calculate the Input for the PID.
Note that Input is a global variable, so the PID function can access it at 
all times.
*/
void setInput(void){
	double alt,Cd;
	
	/* Note:
	Consider getting one event of accelerometer, and one of gyro 
	and passing them to all functions.  This should improve performance.
	sensors_event_t accEvent, orientEvent;
	bno.getEvent(&accEvent, Adaruit_BNO055::VECTOR_LINEARACCEL);//read acceleromete
	bno.getEvent(&orientEvent, Adaruit_BNO055::VECTOR_GYROSCOPE);//read gyro
	*/
	alt = baro.getAltitude();//get data from altimeter
	v = getVelocity();//get velocity from altimeter THIS PAUSES THE ARDUINO
	Cd = getDrag();
	Input = getProjectedAltitude(Cd, v, alt);//calculate Input to PID
}
/*
This function computes acceleration, storing it in a.
It then calculates the drag coefficient, using the global velocity variable.

Using these two variables, it will compute the approximate drag coefficient 
of the rocket in real time.  

Note that getAcceleration(event) is called inside of get drag in order to
compute the vertical acceleration.

Because drag depends on how fully extended the airbrake flaps are, we must
continuously calculate drag as we make corrections based off of our 
projected altitude.
*/
double getDrag(void) {
	double a = getAcceleration();
	// from mg+cv^2=a, we rearrange to solve for c
	double c = (a-mass*gravity)/(velocity*velocity); 
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
double getAcceleration(void) {//event is a struct of type sensors_event_t
	sensors_event_t event;//event variable
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);//read sensor
	double theta = getAngleToVertical(); //get from gyro
	//using spherical coordinates
	double a = cos(theta)*event.acceleration.z;
	return a;
}

/*
This function uses the average of two accelerometer readings multiplied by
the elapsed time, summed with the previous velocity reading.
Thus it integrates over the acceleration to find velocity.

velocity is a global variable that is initialized to 0 on the pad
*/
double getVelocity(void) {
	time_t start,end;
	double acc_first,acc_second;
	start = time(NULL); //get begin time
	acc_first = getAcceleration();//get first acceleration
	delay(100);//delay for 100ms
	acc_second = getAcceleration();//get acceleration again
	end = time(NULL);//get finish time
	velocity=velocity + (end-start)*(acc_first+acc_second)/(2);//Vo + at
	return velocity;
}

/*
Using spherical coordinates, get angle of rocket to vertical
event is a structure of type sensors_event_t which holds information from the
gyro sensor used to derive the angles
*/
double getAngleToVertical(void){
	// hardware read
	sensors_event_t event;//orient event
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);//get orientation
	double x=event.orientation.x*180/pi;//get angle to x in radians, convert to deg
	double y=event.orientation.y*180/pi;//get angle to y in radians, convert to deg
	
	// conversion to spherical
	double theta=atan(y/x);//rectangular to spherical conversion
	return theta;
}
