#include <ros/ros.h>
#include <stdio.h>

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "services");
	ros::NodeHandle nodeHandle;

	ros::Rate r(1.0);


	system("cd && ./ros_services.sh");
	ROS_INFO("Started mapping and exploratoin services");
	
	ros::spinOnce();
	r.sleep();

} // end main
