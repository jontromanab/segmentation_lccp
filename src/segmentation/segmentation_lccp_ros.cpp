#include<segmentation_lccp/segmentation_lccp_ros.h>

LccpSegmentationAlgorithm::LccpSegmentationAlgorithm(std::string name):
  service_name_(name){
  std::cout<<"I am strating service"<<std::endl;
  segmentation_server_ = nh_.advertiseService(service_name_, &LccpSegmentationAlgorithm::segmentationCallback,
                                              this);
}

bool LccpSegmentationAlgorithm::segmentationCallback(segmentation_lccp::segmentation::Request &req,
                                                     segmentation_lccp::segmentation::Response &res){
    std::cout<<"I am a server"<<std::endl;

}

