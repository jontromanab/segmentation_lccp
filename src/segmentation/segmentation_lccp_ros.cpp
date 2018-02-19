#include<segmentation_lccp/segmentation_lccp_ros.h>

LccpSegmentationAlgorithm::LccpSegmentationAlgorithm(ros::NodeHandle* handle, std::string name):
  nh_(*handle), service_name_(name){
  std::cout<<"I am starting service"<<std::endl;
  segmentation_server_ = nh_.advertiseService(service_name_, &LccpSegmentationAlgorithm::segmentationCallback,
                                              this);
}

bool LccpSegmentationAlgorithm::segmentationCallback(segmentation_lccp::segmentation::Request &req,
                                                     segmentation_lccp::segmentation::Response &res){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::cout<<"Input cloud ros has: "<<req.input_cloud.data.size()<<std::endl;
    pcl::fromROSMsg(req.input_cloud, *cloud);
    std::unique_ptr<lccp_segmentation> seg(new lccp_segmentation);
    std::cout<<"Input cloud has: "<<cloud->points.size()<<std::endl;
    seg->init(*cloud);
    std::vector<Object> seg_objs;
    seg->segment();
    seg_objs = seg->get_segmented_objects();
    std::cout<<"\n Detected "<<seg_objs.size()<<" Objects.\n\n";
    /*
    std::vector<PointCloud> clouds = seg->get_segmented_objects_simple();
    std::cout<<"We have "<<clouds.size()<<" clouds. \n";

    std::vector<Object> objects = seg->get_segmented_objects();
    std::cout<<"We have "<<objects.size()<<" objects. \n";

    }*/

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //CloudPtr input_cloud = seg->get_input_cloud();
    CloudPtr table_cloud = seg->get_plane_cloud();
    for(int i = 0; i < table_cloud->points.size(); ++i){
      table_cloud->at(i).r = 0;
      table_cloud->at(i).g = 255;
      table_cloud->at(i).b = 0;
      table_cloud->at(i).a = 255;
    }
    pcl::toROSMsg(*table_cloud, res.plane_cloud);
    return true;
}

