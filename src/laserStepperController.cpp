#include <ros/ros.h>

#include <wiringPi.h>

#define laserStepperDirectionPin 19
#define laserStepperStepPin 21

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserStepperController");

	ros::NodeHandle nodeHandle;


	wiringPiSetupSys();

	system("gpio export 19 output");
	//pinMode(laserStepperDirectionPin, OUTPUT);

	system("gpio export 21 output");
	//pinMode(laserStepperStepPin, OUTPUT);
	
	// Set the laser stepper direction
	digitalWrite(laserStepperStepPin, HIGH);

	ros::Rate(1.0);
	while(nodeHandle.ok())
	{
		// keep moving the laser stepper
		digitalWrite(laserStepperStepPin, HIGH);
		usleep(1);
		digitalWrite(laserStepperStepPin, LOW);
		usleep(1);

		ros::spinOnce();

	} // end while

} // end main
