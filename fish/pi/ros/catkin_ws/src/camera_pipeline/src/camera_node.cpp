#include <camera_pipeline/pipeline_runner.h>
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "camera_pipeline");
  ros::NodeHandle nh;
  camera_pipeline::PipelineRunner runner(nh);
  ROS_INFO("constructed runner\n");
  runner.start_capture();
  ROS_INFO("started runner\n");
  ros::spin();
}
