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
   * @brief Constructor
   */
  LccpSegmentationAlgorithm(ros::NodeHandle* handle, std::string name);
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

};

#endif // SEGMENTATION_LCCP_ROS_H

