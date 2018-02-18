#include<segmentation_lccp/segmentation_lccp_ros.h>

LccpSegmentationAlgorithm::LccpSegmentationAlgorithm(ros::NodeHandle* handle, std::string name):
  nh_(*handle), service_name_(name){
  std::cout<<"I am starting service"<<std::endl;
  segmentation_server_ = nh_.advertiseService(service_name_, &LccpSegmentationAlgorithm::segmentationCallback,
                                              this);
}

bool LccpSegmentationAlgorithm::segmentationCallback(segmentation_lccp::segmentation::Request &req,
                                                     segmentation_lccp::segmentation::Response &res){
    std::cout<<"Input ros msg has: "<<req.input_cloud.data.size()<<std::endl;

    //pcl::PCLPointCloud2 pcl_pc2;
    //pcl_conversions::toPCL(req.input_cloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    pcl::fromROSMsg(req.input_cloud, *cloud);
    lccp_segmentation* seg = new lccp_segmentation();
    std::cout<<"Input cloud has: "<<cloud->points.size()<<std::endl;
    seg->init(*cloud);
    //seg->print_parameters();
    std::vector<Object> seg_objs;
    seg->segment();
    seg_objs = seg->get_segmented_objects();
    std::cout<<"\n Detected "<<seg_objs.size()<<" Objects.\n\n";
    /*
    std::vector<PointCloud> clouds = seg->get_segmented_objects_simple();
    std::cout<<"We have "<<clouds.size()<<" clouds. \n";

    std::vector<Object> objects = seg->get_segmented_objects();
    std::cout<<"We have "<<objects.size()<<" objects. \n";

    for(int i=0;i<objects.size();++i){
      float r, g, b;
      r = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
      g = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
      b = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
      for(int j=0;j<objects[i].obj_cloud.points.size();++j){
        PointT temp_point;
        temp_point.x = objects[i].obj_cloud.points.at(j).x;
        temp_point.y = objects[i].obj_cloud.points.at(j).y;
        temp_point.z = objects[i].obj_cloud.points.at(j).z;
        temp_point.r = r*255;
        temp_point.g = g*255;
        temp_point.b = b*255;
      }
    }*/

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    CloudPtr input_cloud = seg->get_input_cloud();
    CloudPtr table_cloud = seg->get_plane_cloud();
    table_cloud->width = table_cloud->points.size();
    std::cout<<"table cloud size:  "<<table_cloud->points.size()<<std::endl;
    std::cout<<"input cloud size:  "<<input_cloud->points.size()<<std::endl;
    //table_cloud->height = 1;
    //table_cloud->is_dense = true;
    sensor_msgs::PointCloud2 table_cloud_msg;
    pcl::toROSMsg(*table_cloud, table_cloud_msg);
    //table_cloud_msg.header.seq = 1;
    //table_cloud_msg.header.frame_id = req.input_cloud.header.frame_id;
    //table_cloud_msg.header.stamp = ros::Time::now();
    std::cout<<"Table has: "<<table_cloud_msg.data.size()<<std::endl;
    res.plane_cloud = table_cloud_msg;
}

