#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <segmentation_lccp/segmentation.h>



class SegmentationClient
{
public:
  SegmentationClient(ros::NodeHandle* nodeHandle);
  void cloudCallback(const sensor_msgs::PointCloud2& msg);
private:
  ros::NodeHandle nh_;
  std::string cloud_topic_ = "/camera/depth_registered/points";
  std::string segmentation_service_ = "segmentation_service";
  ros::Subscriber sub_;
  ros::Publisher table_cloud_pub_;
};

SegmentationClient::SegmentationClient(ros::NodeHandle *nodeHandle):nh_(*nodeHandle){
  ROS_INFO("Constructor client");
  sub_ = nh_.subscribe(cloud_topic_, 1, &SegmentationClient::cloudCallback, this);
  table_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("table",1000);
}

void SegmentationClient::cloudCallback(const sensor_msgs::PointCloud2& msg){
  ROS_INFO("I am receiving cloud");
  ros::ServiceClient client = nh_.serviceClient<segmentation_lccp::segmentation>(segmentation_service_);
  segmentation_lccp::segmentation srv;
  srv.request.input_cloud = msg;
  client.call(srv);
  std::cout<<"Table has: "<<srv.response.plane_cloud.data.size()<<std::endl;
  table_cloud_pub_.publish(srv.response.plane_cloud);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation_lccp_client");
  ros::NodeHandle nh;
  SegmentationClient client(&nh);
  ros::spin();
  return 0;
}
