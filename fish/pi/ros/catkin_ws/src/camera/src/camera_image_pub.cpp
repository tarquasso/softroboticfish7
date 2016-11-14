#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <raspicam/raspicam_cv.h>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_pub");
	ros::NodeHandle n;

	raspicam::RaspiCam_Cv Camera;
	Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );

	cv::Mat image;
	cv_bridge::CvImage img_bridge;
	std_msgs::Header header;

    	cout<<"Opening Camera..."<<endl;
	if(!Camera.open() ) {
		cerr<<"couldn't open camera";
		return -1;
	}

	sleep(3);

	//create pub topic
	image_transport::ImageTransport it(n);
	image_transport::Publisher image_pub = it.advertise("raw_images", 1);
	ros::Rate loop_rate(1); //Image at 1 Hz

	int count = 0;
	while (ros::ok())
	{
		Camera.grab();
		Camera.retrieve(image);

		header.seq = count;
		header.stamp = ros::Time::now();

		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);

		ROS_INFO("Sending image \n");
		image_pub.publish(img_bridge.toImageMsg());

		++count;
		ros::spinOnce();
		loop_rate.sleep();
	}

    	cout<<"Stopping Camera..."<<endl;
	Camera.release();

	return 0;
}
