#ifndef SEGMENTATION_LCCP_NODE_H
#define SEGMENTATION_LCCP_NODE_H
#include<segmentation_lccp.h>
#include<sensor_msgs/PointCloud2.h>
#include<segmentation_lccp/segmentation.h>

/**
 * \brief Wrapper ROS class to obtain segmented objects by a service request
*/

class LccpSegmentationAlgorithm
{
public:
  LccpSegmentationAlgorithm();
private:
  ros::ServiceServer segmentation_server_;
  bool segmentationCallback(segmentation_lccp::segmentation::Request& req,
                            segmentation_lccp::segmentation::Response& res);


}

#endif // SEGMENTATION_LCCP_NODE_H
