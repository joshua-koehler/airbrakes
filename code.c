#include<stdio.h>

PID Controller (somewhat still Pseudo) Code
/* Initial --- 11.5.2019
Notes: The primary (and perhaps only) part of this software that needs to be worked out is which variable to use for the error calculation, and how to do this.  If acceleration and velocity can be
measured accurately, I suggest using acceleration as the control variable with the following derived
from the kinematic equations:
Ideal Acceleration = -(current_velocity^2)/(2*(10,000ft - current_alt)
The advantage is that Drag is automatically taken into account, eliminating the need of a Drag equation.
We should discuss this further, but this seems promising.
After an airbrake team discussion, I plan on doing a code review with Malcolm and a couple other software engineers.
*/

/* Update --- 11.22.2019
Solved the algorithm.
Drag is actually the control variable.
Using calculus, the full kinematic equations were solved with quadratic Drag, solving for peak altitude.
This peak altitude equation has only one input variable: the Drag coefficient C.

/* Update --- 11.30.2019
Did systems integration.  
To do:
1. Ensure hardware reads are correct (especially the gyro)
2. Ensure units are matching
3. Proofread code
4. Code review
5. Input correct mass m
6. Data collection
7. Data display
For the PID (inverse logic), the basic process is now.
Setpoint = 10k - current_altitude
Calculate current Cd by extrapolating from the acceleration.
Plug this Cd into the y_peak equation.
Input = ypeak(Cd)

And that's it!

*/


// CHECK UNITS OF ALL SENSORS TO ENSURE WE HAVE THE RIGHT UNITS!
#include <math.h>
#include <PID_v1.h>

double Drag(a,v); //Prototype
double ProjectedAltitude(Cd, v, alt); //Prototype
double VerticalAcceleration(); //Prototype
double VerticalVelocity(); //Prototype
double sphericalThetaRead(); //Prototype

// ADDED IN LIBRARIES FOR SENSORS
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
// END OF ADDED IN LIBRARIES FOR SENSORS

//Define Variables we'll be connecting to
double Setpoint, Input, Output, alt, v, a, apogee, m, previous;
int previousOutput = 0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, REVERSE);

void setup() {
	pinMode(7,INPUT);
	pinMode(8,INPUT);
	esc.attach(8);
	bno.setExtCrystalUse(true);
	m = MASS OF THE ROCKET AFTER BURNOUT
	apogee=10000+baro.getAltitude;
	
	// PID Set-Up - Default max output is 255, Default were specified above @ 2,5,1
	// We should compare these to see how they fare, collecting data of projected altitude vs. setpoint over time
	// Default sample time is 200ms, which should be sufficient, tuning parameters matter more
	Setpoint=apogee // This should be correct, this way the rocket is always shooting for exactly 10k above ground
	
	// We could probably do this with while(alt<5000) {alt=baro.getAltitude}
	while(motor == burning) {
		v=VerticalVelocity();
		a=VerticalAcceleration();
		//wait
	}

	Cd = Drag(a,v);
	Input = ProjectedAltitude(Cd, v);

	// turn the PID on
	myPID.SetMode(AUTOMATIC);
	
}

void loop() {
 	// Setpoint is permanently set, so all we need is input
	
	// Sensors
	sensors_event_t_event;
	bno.getEvent(&event);	

	v=VerticalVelocity();
	a=VerticalAcceleration();
	float alt = baro.getbaro.getAltitude; // Check units on this
	Cd=Drag(a,v); 
	
	Input = ProjectedAltitude(Cd, v, alt);
	// our error is: error = Setpoint - Input;
	
	// this computes the PID and stores the needed output as global variable <Output>
	myPID.Compute();
	
	// I need to ensure that the analogWrite adjusts properly
	// It seems the myPID.Compute() takes care of the output
	// This method of using this library should be tested with a similiar, non-rocket situation
	// to determine if it works
	
	// Probably should set an analog out limit in the PID software
	analogWrite(airbrakeMotor,Output);
	// This may actually need to be: 
	// analogWrite(airbrakeMotor,previous + Output);
	// previous = Output;

}
	
	
double Drag(a,v) {
	double c = (a-mg)/(v^2); // from mg+cv^2=a, we rearrange to solve for c
	return c;
}
	
double ProjectedAltitude(Cd, v, alt) {
	double y = (m/Cd)*ln(cos(atan(v/sqrt(m*g/Cd)))) // Calculate additional altitude projected
	double ypro = y + alt;	// Projected altitude is the additional + current alt
	return ypro;
}

double VerticalAcceleration() {
	double rawa = event.acceleration.z; //get from hardware
	double theta = sphericalThetaRead(); //get from hardware
	//using spherical coordinates
	double a = cos(theta)*rawa;
	return a;
	
}

double VerticalVelocity() {
	double rawv = event.velocity.z //get from hardware
	double theta = sphericalThetaRead(); //get from hardware
	//using spherical coordinates
	double v = cos(theta)*rawv;
	return v;
}

double sphericalThetaRead() {
	// hardware read and calculations
	double x=event.orientation.x;
	double y=event.orientation.y;
	double z=event.orientation.z;
	double r=sqrt(x^2+y^2+z^2);
	double theta=atan(y/x);
	//phi=atan(sqrt(x^2+y^2)/z);
	return theta;
}
