#include<segmentation_lccp/segmentation_lccp_node.h>

LccpSegmentationAlgorithm::LccpSegmentationAlgorithm(std::string name):
  service_name_(name){
  std::cout<<"I am strating service"<<std::endl;
  segmentation_server_ = nh_.advertiseService(service_name_, &LccpSegmentationAlgorithm::segmentationCallback,
                                              this);
}

bool LccpSegmentationAlgorithm::segmentationCallback(segmentation_lccp::segmentation::Request &req, segmentation_lccp::segmentation::Response &res){
  std::cout<<"Hello I am service"<<std::endl;
}


int main(int argc, char *argv[]){
  ros::init(argc, argv, "segmentation_service");
  LccpSegmentationAlgorithm seg("segmentation_service");
  ros::spin();
  return 0;
}
