#ifndef SEGMENTATION_LCCP_ROS_H
#define SEGMENTATION_LCCP_ROS_H

#include<segmentation_lccp/segmentation_lccp.h>
#include<sensor_msgs/PointCloud2.h>
#include<segmentation_lccp/segmentation.h>
#include<ros/ros.h>

/**
 * \brief Wrapper ROS class to obtain segmented objects by a service request
*/

class LccpSegmentationAlgorithm
{
public:
  LccpSegmentationAlgorithm(ros::NodeHandle* handle, std::string name);
private:
  ros::ServiceServer segmentation_server_;
  bool segmentationCallback(segmentation_lccp::segmentation::Request& req,
                            segmentation_lccp::segmentation::Response& res);

  ros::NodeHandle nh_;
  std::string service_name_;


};

#endif // SEGMENTATION_LCCP_ROS_H

