#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>

float laserRange;
unsigned int laserIntensity;

void laserRangeCallback (const std_msgs::Int16 &laserRangeSTD)
{
   
     laserRange = (float) laserRangeSTD.data;


} // end laserRangeCallback

void laserIntensityCallback (const std_msgs::Int16 &laserIntensitySTD)
{
     laserIntensity = (unsigned int) laserIntensitySTD.data;


} // end laserIntensityCallback

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserScanNode");
    ros::NodeHandle nodeHandle;

    ros::Subscriber laserRangeSubscriber = nodeHandle.subscribe("laserRangeTopic", 10, laserRangeCallback);
    ros::Subscriber laserIntensitySubscriber = nodeHandle.subscribe("laserIntensityTopic", 10, laserIntensityCallback);

    ros::Publisher laserScanPublisher = nodeHandle.advertise <sensor_msgs::LaserScan> ("base_scan", 10);

    ros::Rate r(1.0);

    while(nodeHandle.ok())
    {
	ros::spinOnce();
        double laserFrequency = 100 * 1000;

        int readingsNo = 200;
        ros::Time scanTime = ros::Time::now();
        double minAngle = -1.57;
        double maxAngle = 1.57;
        double angleIncrement = 3.14 / readingsNo;
        double timeIncrement = readingsNo / laserFrequency;
        double minRange = 0.0;
        double maxRange = 100.0;

	ros::spinOnce();
        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scanTime;
        scan.header.frame_id = "base_laser";
        scan.angle_min = minAngle;
        scan.angle_max = maxAngle;
        scan.angle_increment = angleIncrement;
        scan.time_increment = timeIncrement;
        scan.range_min = minRange;
        scan.range_max = maxRange;

        scan.ranges.resize(readingsNo+1);
        scan.intensities.resize(readingsNo+1);

        for(unsigned int i = 0; i <= readingsNo; i++)
        {
            scan.ranges[i] = laserRange;
            scan.intensities[i] = laserIntensity;
	    ros::spinOnce();

        } // end for

        laserScanPublisher.publish(scan);
	ros::spinOnce();
        r.sleep();

    } // end while

} // end main
