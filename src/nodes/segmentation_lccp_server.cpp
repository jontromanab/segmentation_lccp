#include <ros/ros.h>
#include <segmentation_lccp/segmentation_lccp_ros.h>

std::string segmentation_service = "segmentation_service";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation_lccp_server");
  ros::NodeHandle nh;
  LccpSegmentationAlgorithm seg(&nh, segmentation_service);
  ros::spin();
  return 0;
}
