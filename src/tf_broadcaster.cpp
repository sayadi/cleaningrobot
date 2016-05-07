#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(1.0);

  tf::TransformBroadcaster broadcaster;

  while(n.ok())
{
    ros::spinOnce();
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_laser"));

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
