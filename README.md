This repository contains all the software used for the Airbrakes team of Clark Aerospace, 2019/2020 ESRA season 
(Spaceport America Cup). 

For the competition, the team designed and built a roughly 12' long, 70 pound (fuel included) rocket.
Competing against some 120 teams from around the world, Clark entered the 10k COTS (Commerical Off The Shelf motor) category.
As such, the goal is to launch the rocket, and apogee (hit the peak of the flight), as close to 10,000 feet above ground level
as possible, recovering the rocket safely. 

This is where the airbrakes come in.  Utilizing airbrake flaps controlled by a servo motor, an open source PID library, and 
on-board avionics, the rocket continuously calculates its projected altitude, and makes small adjustments to the airbrakes 
(extending or retracting) during the flight, to get as close as possible to 10,000 feet. 

In order to do this, the kinematic equations were calculated with drag as coaxial to gravity, enabling an analytical solution.
Using integration tables to get the final equation, final height was isolated. As the rocket ascends, the drag coefficient is 
calculated in real time, and this coefficient is fed into the projected altitude equation.  This projected altitude is an input
variable to the PID, being compared with the setpoint 10k AGL (10,000 feet above ground level).

More testing will be done to collect data, and further tune the parameters to ensure the most precise system possible.

If you have any questions about this project, feel free to send me a message on linkedin at 
https://www.linkedin.com/in/josh-koehler-b37841175/
 

