#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

void imgReceived(const sensor_msgs::ImageConstPtr& msg)
{
	Mat img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC3)->image;
	double r = 0, g =0, b =0;
	for(int j = 0; j<img.rows; j++) {
		for(int i =0;i<img.cols;i++) {
			Vec3b &cur = img.at<Vec3b>(j,i);
			r+= cur[0];
			g+= cur[1];
			b+= cur[2];
		}
	}

	r /= (img.rows*img.cols);
	g /= (img.rows*img.cols);
	b /= (img.rows*img.cols);

	ROS_INFO("Received image %lf %lf %lf\n", r, g, b);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_sub");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("images", 1000, imgReceived);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub = it.subscribe("raw_images", 1, imgReceived);
  ros::spin();

  return 0;
}
