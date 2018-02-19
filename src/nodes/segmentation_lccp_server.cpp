#include <ros/ros.h>
#include <segmentation_lccp/segmentation_lccp_ros.h>

std::string segmentation_service = "segmentation_service";
static const double VOXEL_RESOLUTION = 0.0015f;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation_lccp_server");
  ros::NodeHandle nh;
  LccpSegmentationAlgorithm::Parameters params;
  nh.getParam("segmentation_server/zmax", params.zmax);
  nh.getParam("segmentation_server/zmin", params.zmin);
  nh.getParam("segmentation_server/th_points", params.th_points);
  nh.getParam("segmentation_server/disable_transform", params.disable_transform);
  nh.getParam("segmentation_server/voxel_resolution", params.voxel_resolution);
  nh.getParam("segmentation_server/seed_resolution", params.seed_resolution);
  nh.getParam("segmentation_server/color_importance", params.color_importance);
  nh.getParam("segmentation_server/spatial_importance", params.spatial_importance);
  nh.getParam("segmentation_server/normal_importance", params.normal_importance);
  nh.getParam("segmentation_server/use_extended_convexity", params.use_extended_convexity);
  nh.getParam("segmentation_server/use_sanity_criterion", params.use_sanity_criterion);
  nh.getParam("segmentation_server/concavity_tolerance_threshold", params.concavity_tolerance_threshold);
  nh.getParam("segmentation_server/smoothness_threshold", params.smoothness_threshold);
  nh.getParam("segmentation_server/min_segment_size", params.min_segment_size);
  LccpSegmentationAlgorithm seg(&nh, params, segmentation_service);
  ros::spin();
  return 0;
}
