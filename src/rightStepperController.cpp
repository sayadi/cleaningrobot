#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <wiringPi.h>

#define rightStepperDirectionPin 13
#define rightStepperStepPin 15

float rightStepperSpeed; // in m/s
float rightStepperRPM;
double rightStepperDelay;

const float distanceBetweenTwoWheels = 0.313; // in m
const float wheelsRaduis = 0.04325; // in m

void cmd_velCallback (const geometry_msgs::Twist twist)
{

    rightStepperSpeed = (twist.angular.z * M_PI * distanceBetweenTwoWheels / 2) + twist.linear.x; // this is in m/s

    rightStepperRPM = rightStepperSpeed / (M_PI * wheelsRaduis);

    // calculate the delay for the rpm in msec
    rightStepperDelay = abs( 600 / (4 * rightStepperRPM) );

    // Set the directions
    if(rightStepperSpeed < 0)
    {
      digitalWrite(rightStepperDirectionPin, LOW);

    } // end if

    else // rightStepperSpeed >= 0
    {
      digitalWrite(rightStepperDirectionPin, HIGH);

    } // end else


   ROS_INFO("Right stepper delay: %f\n", rightStepperDelay);


    	// Move the motor
	digitalWrite(rightStepperStepPin, LOW);
	usleep(rightStepperDelay);
	digitalWrite(rightStepperStepPin, HIGH);
	usleep(rightStepperDelay);
	digitalWrite(rightStepperStepPin, LOW);
	usleep(rightStepperDelay);
	digitalWrite(rightStepperStepPin, HIGH);

} // end cmd_velCallback

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rightStepperController");

	ros::NodeHandle nodeHandle;

	ros::Subscriber cmd_velSubscriber = nodeHandle.subscribe("cmd_vel", 10, cmd_velCallback);


	wiringPiSetupSys();

	system("gpio export 13 output");
	//pinMode(rightStepperDirectionPin, OUTPUT);

	system("gpio export 15 output");
	//pinMode(rightStepperStepPin, OUTPUT);

	ros::Rate r(1.0);
	while(nodeHandle.ok())
		ros::spinOnce();
	r.sleep();

} // end main
