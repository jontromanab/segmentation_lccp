#include<segmentation_lccp/segmentation_lccp_ros.h>

LccpSegmentationAlgorithm::LccpSegmentationAlgorithm(ros::NodeHandle* handle, const Parameters& param, std::string name):
  nh_(*handle), service_name_(name){
  ROS_INFO("Segmentation service started");
  this->param_.zmin = param.zmin;
  this->param_.zmax = param.zmax;
  this->param_.th_points = param.th_points;
  this->param_.disable_transform = param.disable_transform;
  this->param_.voxel_resolution = param.voxel_resolution;
  this->param_.seed_resolution = param.seed_resolution;
  this->param_.color_importance = param.color_importance;
  this->param_.spatial_importance = param.spatial_importance;
  this->param_.normal_importance = param.normal_importance;
  this->param_.concavity_tolerance_threshold = param.concavity_tolerance_threshold;
  this->param_.smoothness_threshold = param.smoothness_threshold;
  this->param_.min_segment_size = param.min_segment_size;
  this->param_.use_extended_convexity = param.use_extended_convexity;
  this->param_.use_sanity_criterion = param.use_sanity_criterion;
  segmentation_server_ = nh_.advertiseService(service_name_, &LccpSegmentationAlgorithm::segmentationCallback,
                                              this);
}

bool LccpSegmentationAlgorithm::segmentationCallback(segmentation_lccp::segmentation::Request &req,
                                                     segmentation_lccp::segmentation::Response &res){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(req.input_cloud, *cloud);
    std::unique_ptr<lccp_segmentation> seg(new lccp_segmentation);
    seg->init(*cloud, this->param_);
    std::vector<Object> seg_objs;
    seg->segment();
    seg_objs = seg->get_segmented_objects();

    //Table cloud
    CloudPtr table_cloud = seg->get_plane_cloud();
    for(uint i = 0; i < table_cloud->points.size(); ++i){
      table_cloud->at(i).r = 0;
      table_cloud->at(i).g = 255;
      table_cloud->at(i).b = 0;
      table_cloud->at(i).a = 255;
    }
    pcl::toROSMsg(*table_cloud, res.plane_cloud);

    //Object clouds
    for(uint i=0;i<seg_objs.size();++i){
      sensor_msgs::PointCloud2 obj_msg;
      pcl::toROSMsg(seg_objs[i].obj_cloud, obj_msg);
      obj_msg.header.seq = i;
      obj_msg.header.frame_id = req.input_cloud.header.frame_id;
      obj_msg.header.stamp = ros::Time::now();
      res.object_cloud.push_back(obj_msg);
    }
    return true;
}

