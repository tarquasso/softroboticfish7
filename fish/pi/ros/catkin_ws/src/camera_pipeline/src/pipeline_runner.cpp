#include "camera_pipeline/pipeline_runner.h"
#include <AprilTags/Tag36h11.h>
#include <boost/foreach.hpp>
#include <thread>
#include <chrono>
#include <ros/ros.h>

namespace camera_pipeline {

PipelineRunner::PipelineRunner(ros::NodeHandle& nh): it_(nh){

  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(AprilTags::tagCodes36h11));

  // detections_pub_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  // pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("tag_detections_pose", 1);
  // image_pub_ = it_.advertise("tag_detections_image", 1);
}

PipelineRunner::~PipelineRunner() {
  // nothing
}

void PipelineRunner::start_capture(void) {
  camera_.set(CV_CAP_PROP_FORMAT, CV_8UC1);
  camera_.open();
  grabber_thread_ = std::thread(&PipelineRunner::grab_routine, this);
  publisher_thread_ = std::thread(&PipelineRunner::publish_routine, this);
}

void PipelineRunner::grab_helper(const void *args) {
  ROS_INFO("starting grabber routine\n");
  ((PipelineRunner*)args)->grab_routine();
}

void PipelineRunner::publish_helper(const void *args) {
  ROS_INFO("starting main routine\n");
  ((PipelineRunner*)args)->publish_routine();
}

void PipelineRunner::grab_routine() {
	while (true) {
    cam_mtx_.lock();
    camera_.grab();
    cam_mtx_.unlock();
  }
}

void PipelineRunner::publish_routine() {
  cv::Mat image;
  int i = 0;
  while (true) {
    ROS_INFO("processing image");
    cam_mtx_.lock();
    camera_.retrieve(image);
    cam_mtx_.unlock();
    std::vector<AprilTags::TagDetection>  detections = tag_detector_->extractTags(image);
    BOOST_FOREACH(AprilTags::TagDetection detection, detections){
      ROS_INFO("detected tag id %d", detection.id);
      detection.draw(image);
    }
    cv::imwrite("/home/fish/pics/cam_detections_" + std::to_string(i++) + ".jpg", image);
    // Do some processing
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

}