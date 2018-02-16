#include<segmentation_lccp/segmentation_lccp_ros.h>

LccpSegmentationAlgorithm::LccpSegmentationAlgorithm(ros::NodeHandle* handle, std::string name):
  nh_(*handle), service_name_(name){
  std::cout<<"I am starting service"<<std::endl;
  segmentation_server_ = nh_.advertiseService(service_name_, &LccpSegmentationAlgorithm::segmentationCallback,
                                              this);
}

bool LccpSegmentationAlgorithm::segmentationCallback(segmentation_lccp::segmentation::Request &req,
                                                     segmentation_lccp::segmentation::Response &res){
    std::cout<<"I am a server"<<std::endl;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(req.input_cloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    lccp_segmentation* seg = new lccp_segmentation();
    seg->init(*cloud);
    //seg->print_parameters();
    std::vector<Object> seg_objs;
    seg->segment();
    seg_objs = seg->get_segmented_objects();
    std::cout<<"\n Detected "<<seg_objs.size()<<" Objects.\n\n";

}

