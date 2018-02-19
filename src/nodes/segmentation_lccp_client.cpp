#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <segmentation_lccp/segmentation.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>



class SegmentationClient
{
public:
  SegmentationClient(ros::NodeHandle* nodeHandle);
  void cloudCallback(const sensor_msgs::PointCloud2& msg);
  void publishClouds();
private:
  ros::NodeHandle nh_;
  std::string cloud_topic_ = "/camera/depth_registered/points";
  std::string segmentation_service_ = "segmentation_service";
  ros::Subscriber sub_;
  ros::Publisher table_cloud_pub_;
  sensor_msgs::PointCloud2 table_cloud_ros_;
  sensor_msgs::PointCloud2 object_cloud_ros_;
};

SegmentationClient::SegmentationClient(ros::NodeHandle *nodeHandle):nh_(*nodeHandle){
  ROS_INFO("Constructor client");
  sub_ = nh_.subscribe(cloud_topic_, 1, &SegmentationClient::cloudCallback, this);
  table_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("table",10);
}

void SegmentationClient::cloudCallback(const sensor_msgs::PointCloud2& msg){
  ROS_INFO("I am receiving cloud");
  ros::ServiceClient client = nh_.serviceClient<segmentation_lccp::segmentation>(segmentation_service_);
  segmentation_lccp::segmentation srv;
  srv.request.input_cloud = msg;
  if(client.call(srv)){
    table_cloud_ros_ = srv.response.plane_cloud;
    std::cout<<"I am calling service"<<std::endl;
    std::cout<<"Table cloud has: "<<table_cloud_ros_.data.size()<<std::endl;
    publishClouds();
  }
}

void SegmentationClient::publishClouds(){
  table_cloud_pub_.publish(table_cloud_ros_);
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation_lccp_client");
  ros::NodeHandle nh;
  SegmentationClient client(&nh);
  ros::spin();
  return 0;
}
