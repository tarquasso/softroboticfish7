#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <raspicam/raspicam_cv.h>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_pub");
	ros::NodeHandle n;

	raspicam::RaspiCam_Cv Camera;
	int nCount=100;
	Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
	cv::Mat image;
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img_msg;
	std_msgs::Header header;

    	cout<<"Opening Camera..."<<endl;
	if(!Camera.open() ) {
		cerr<<"couldn't open camera";
		return -1;
	}

	sleep(3);

	//create pub topic
	ros::Publisher chatter_pub = n.advertise<sensor_msgs::Image>("images", 1000);
	ros::Rate loop_rate(1); //Image at 1 Hz

	int count = 0;
	while (ros::ok())
	{
		Camera.grab();
		Camera.retrieve(image);

		header.seq = count;
		header.stamp = ros::Time::now();

		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
		img_bridge.toImageMsg(img_msg);

		ROS_INFO("Sending image\n");
		chatter_pub.publish(img_msg);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

    	cout<<"Stopping Camera..."<<endl;
	Camera.release();

	return 0;
}
