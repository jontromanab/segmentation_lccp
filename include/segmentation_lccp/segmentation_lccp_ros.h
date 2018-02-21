#ifndef SEGMENTATION_LCCP_ROS_H
#define SEGMENTATION_LCCP_ROS_H

#include<segmentation_lccp/segmentation_lccp.h>
#include<sensor_msgs/PointCloud2.h>
#include<segmentation_lccp/segmentation.h>
#include<ros/ros.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

/**
 * \brief Wrapper ROS class to obtain segmented objects by a service request
*/

class LccpSegmentationAlgorithm
{
public:
  /**
   * @brief The Parameters struct contains parameters of the algorithm
   */
  struct Parameters{
    double zmin;
    double zmax;
    int th_points;
    //superVoxel parameters
    bool disable_transform;
    double voxel_resolution;
    double seed_resolution;
    double color_importance;
    double spatial_importance;
    double normal_importance;
    //LCCP segmentation parameters
    double concavity_tolerance_threshold;
    double smoothness_threshold;
    int min_segment_size;
    bool use_extended_convexity;
    bool use_sanity_criterion;
  };
  /**
   * @brief Constructor
   */
  LccpSegmentationAlgorithm(ros::NodeHandle* handle, const Parameters& param, std::string name);
  ~LccpSegmentationAlgorithm(void);
private:
  ///server
  ros::ServiceServer segmentation_server_;

  /**
   * @brief segmentationCallback Callback function to server
   * @param req input cloud
   * @param res table cloud and vector of objects clouds
   * @return if service successful
   */
  bool segmentationCallback(segmentation_lccp::segmentation::Request& req,
                            segmentation_lccp::segmentation::Response& res);

  ///NodeHandle
  ros::NodeHandle nh_;

  ///Name of service
  std::string service_name_;

  ///Segmentation parameters
  supervoxel_parameters param_;

  pthread_mutex_t obj_seg_mutex_;
  void obj_seg_mutex_enter_(void);
  void obj_seg_mutex_exit_(void);

};

#endif // SEGMENTATION_LCCP_ROS_H

