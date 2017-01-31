#ifndef PIPELINE_RUNNER_H
#define PIPELINE_RUNNER_H

#include <ros/ros.h>
#include <AprilTags/TagDetector.h>
#include <raspicam/raspicam_cv.h>
#include <image_transport/image_transport.h>
#include <thread>
#include <mutex>

namespace camera_pipeline {

class PipelineRunner{
  public:
    PipelineRunner(ros::NodeHandle& nh);
    ~PipelineRunner();
    void start_capture(void);
  private:
    static void grab_helper(const void *args);
    static void publish_helper(const void *args);
    void grab_routine(void);
    void publish_routine(void);

  private:
    std::thread publisher_thread_;
    std::thread grabber_thread_;
    image_transport::ImageTransport it_;
    raspicam::RaspiCam_Cv camera_;
    std::mutex cam_mtx_;
    // ros::Publisher detections_pub_;
    // ros::Publisher pose_pub_;
    // tf::TransformBroadcaster tf_pub_;
    boost::shared_ptr<AprilTags::TagDetector> tag_detector_;
};

}

#endif