//PID Controller (somewhat still Pseudo) Code
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

/* Update --- 12.13.2019
 *  Debugged and integrated libraries.
 *  event. is not working, some libary issue I thinkg
 */


// CHECK UNITS OF ALL SENSORS TO ENSURE WE HAVE THE RIGHT UNITS!
#include<stdio.h>
#include <math.h>
#include <PID_v1.h>

// ADDED IN LIBRARIES FOR SENSORS
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//i2c
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); //Accelerometer

#include <Adafruit_LSM9DS1.h>
#include <SPI.h>


#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
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
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G); //Acceleromter Range
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS); //Magnetometer Sensitivity
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);  //Gyroscope
  
  pinMode(7,INPUT);
  pinMode(8,INPUT);
  esc.attach(8);
  bno.setExtCrystalUse(true);
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

  lsm.read();  /* ask it to read in the data */ 
  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  
  // Sensors
  sensors_event_t_event;
  bno.getEvent(&event); 
  
  rawv = event.velocity.z() //get from hardware
  v=VerticalVelocity(rawv);
  
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

double VerticalVelocity(double rawvelocity) {
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
