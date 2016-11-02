#include "ros/ros.h"
#include "sensor_msgs/Image.h"

void imgReceived(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("Received image\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_sub");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("images", 1000, imgReceived);

  ros::spin();

  return 0;
}
