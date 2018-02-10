#include<segmentation_lccp/segmentation_lccp.h>

supervoxel_parameters::supervoxel_parameters(){
  this->disable_transform = this->DISABLE_TRANSFORM;
  this->voxel_resolution = this->VOXEL_RESOLUTION;
  this->seed_resolution = this->SEED_RESOLUTION;
  this->color_importance = this->COLOR_IMPORTANCE;
  this->spatial_importance = this->SPATIAL_IMPORTANCE;
  this->normal_importance = this->NORMAL_IMPORTANCE;

  this->concavity_tolerance_threshold = this->CONCAVITY_TOLERANCE_THRESHOLD;
  this->smoothness_threshold = this->SMOOTHNESS_THRESHOLD;
  this->min_segment_size = this->MIN_SEGMENT_SIZE;
  this->use_extended_convexity = this->USE_EXTENDED_CONVEXITY;
  this->use_sanity_criterion = this->USE_SANITY_CRITERION;

  this->zmin = this->ZMIN;
  this->zmax = this->ZMAX;
  this->th_points = this->TH_POINTS;
}

supervoxel_parameters::~supervoxel_parameters(){

}

lccp_segmentation::lccp_segmentation(){
  this->initialized_ = true;
}

lccp_segmentation::~lccp_segmentation(){

}

void lccp_segmentation::init(PointCloud input_cloud, supervoxel_parameters &opt){
  this->cloud_ = input_cloud.makeShared();
  this->detected_objects_.resize(0);
  set_parameters(opt);
  this->initialized_ = true;
}

void lccp_segmentation::init(PointCloud input_cloud){
  this->cloud_ = input_cloud.makeShared();
  this->detected_objects_.resize(0);
  set_default_parameters();
  this->initialized_ = true;
}

void lccp_segmentation::set_parameters(const supervoxel_parameters &opt){
  this->disable_transform = opt.disable_transform;
  this->voxel_resolution = opt.voxel_resolution;
  this->seed_resolution = opt.seed_resolution;
  this->color_importance = opt.color_importance;
  this->spatial_importance = opt.spatial_importance;
  this->normal_importance = opt.normal_importance;

  this->concavity_tolerance_threshold = opt.concavity_tolerance_threshold;
  this->smoothness_threshold = opt.smoothness_threshold;
  this->min_segment_size = opt.min_segment_size;
  this->use_extended_convexity = opt.use_extended_convexity;
  this->use_sanity_criterion = opt.use_sanity_criterion;

  this->zmin = opt.zmin;
  this->zmax = opt.zmax;
  this->th_points = opt.th_points;
}

void lccp_segmentation::set_default_parameters(){
  this->disable_transform = this->DISABLE_TRANSFORM;
  this->voxel_resolution = this->VOXEL_RESOLUTION;
  this->seed_resolution = this->SEED_RESOLUTION;
  this->color_importance = this->COLOR_IMPORTANCE;
  this->spatial_importance = this->SPATIAL_IMPORTANCE;
  this->normal_importance = this->NORMAL_IMPORTANCE;

  this->concavity_tolerance_threshold = this->CONCAVITY_TOLERANCE_THRESHOLD;
  this->smoothness_threshold = this->SMOOTHNESS_THRESHOLD;
  this->min_segment_size = this->MIN_SEGMENT_SIZE;
  this->use_extended_convexity = this->USE_EXTENDED_CONVEXITY;
  this->use_sanity_criterion = this->USE_SANITY_CRITERION;

  this->zmin = this->ZMIN;
  this->zmax = this->ZMAX;
  this->th_points = this->TH_POINTS;
}

void lccp_segmentation::reset(){
  this->cloud_->points.resize(0);
  this->detected_objects_.resize(0);
  this->lccp_labeled_cloud_->points.resize(0);
  this->labeled_voxel_cloud_->points.resize(0);
  this->normal_cloud_->points.resize(0);
  this->supervoxel_clusters_.clear();
  this->supervoxel_adjacency_.clear();
  set_default_parameters();
}

void lccp_segmentation::print_parameters(){
  std::cout<< "\nsupervoxel_parameters: \n"
                << "disable_transform: "<<(bool)this->disable_transform<<std::endl
                << "voxel_resolution: "<<(double)this->voxel_resolution<<std::endl
                << "seed_resolution:  "<<(double)this->seed_resolution<<std::endl
                << "color_importance: "<<(double)this->color_importance<<std::endl
                << "spatial_importance: "<<(double)this->spatial_importance<<std::endl
                << "normal_importance: "<<(double)this->normal_importance<<std::endl
                << "\n LCCP parameters \n"
                << "concavity_tolerance_threshold: "<<(double)this->concavity_tolerance_threshold<<std::endl
                << "smoothness_threshold: "<<(double)this->smoothness_threshold<<std::endl
                << "min_segment_size: "<<(int)this->min_segment_size<<std::endl
                << "use_extended_convexity: "<<(bool)this->use_extended_convexity<<std::endl
                << "use_sanity_criterion: "<<(bool)this->use_sanity_criterion<<std::endl
                << "\n Other parameters \n"
                << "zmin: "<<(double)this->zmin<<std::endl
                << "zmax: "<<(double)this->zmax<<std::endl
                << "th_points: "<<(int)this->th_points<<std::endl;
}

CloudPtr lccp_segmentation::get_input_cloud(){
  return this->cloud_;
}

CloudPtr lccp_segmentation::get_plane_cloud(){
  return this->table_plane_cloud_;
}

PointCloudl lccp_segmentation::get_labeled_voxel_cloud(){
  return *(this->labeled_voxel_cloud_);
}

std::multimap<uint32_t, uint32_t> lccp_segmentation::get_supervoxel_adjacency(){
  return this->supervoxel_adjacency_;
}

std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> lccp_segmentation::get_supervoxel_clusters(){
  return this->supervoxel_clusters_;
}

pcl::PointCloud<pcl::PointNormal> lccp_segmentation::get_normal_cloud(){
  return *(this->normal_cloud_);
}

void lccp_segmentation::set_disable_transform(bool disable_transform_in){
  this->disable_transform = disable_transform_in;
}

void lccp_segmentation::set_voxel_resolution(double voxel_resolution_in){
  this->voxel_resolution = voxel_resolution_in;
}

void lccp_segmentation::set_seed_resolution(double seed_resolution_in){
  this->seed_resolution = seed_resolution_in;
}

void lccp_segmentation::set_color_importance(double color_importance_in){
  this->color_importance = color_importance_in;
}

void lccp_segmentation::set_spatial_importance(double spatial_importance_in){
  this->spatial_importance = spatial_importance_in;
}

void lccp_segmentation::set_normal_importance(double normal_importance_in){
  this->normal_importance = normal_importance_in;
}

void lccp_segmentation::set_concavity_tolerance_threshold(double concavity_tolerance_threshold_in){
  this->concavity_tolerance_threshold = concavity_tolerance_threshold_in;
}

void lccp_segmentation::set_smoothness_threshold(double smoothness_threshold_in){
  this->smoothness_threshold = smoothness_threshold_in;
}

void lccp_segmentation::set_min_segment_size(int min_segment_size_in){
  this->min_segment_size = min_segment_size_in;
}

void lccp_segmentation::set_use_extended_convexity(bool use_extended_convexity_in){
  this->use_extended_convexity = use_extended_convexity_in;
}

void lccp_segmentation::set_use_sanity_criterion(bool use_sanity_criterion_in){
  this->use_sanity_criterion = use_sanity_criterion_in;
}

void lccp_segmentation::set_zmin(double zmin_in){
  this->zmin = zmin_in;
}

void lccp_segmentation::set_zmax(double zmax_in){
  this->zmax = zmax_in;
}

void lccp_segmentation::set_th_points(int th_points_in){
  this->th_points = th_points_in;
}

bool lccp_segmentation::get_disable_transform(){
  return this->get_disable_transform();
}

double lccp_segmentation::get_voxel_resolution(){
  return this->voxel_resolution;
}

double lccp_segmentation::get_seed_resolution(){
  return this->seed_resolution;
}

double lccp_segmentation::get_color_importance(){
  return this->color_importance;
}

double lccp_segmentation::get_spatial_importance(){
  return this->spatial_importance;
}

double lccp_segmentation::get_normal_importance(){
  return this->normal_importance;
}

double lccp_segmentation::get_concavity_tolerance_threshold(){
  return this->concavity_tolerance_threshold;
}

double lccp_segmentation::get_smoothness_threshold(){
  return this->smoothness_threshold;
}

int lccp_segmentation::get_min_segment_size(){
  return this->min_segment_size;
}

bool lccp_segmentation::get_use_extended_convexity(){
  return this->use_extended_convexity;
}

bool lccp_segmentation::get_use_sanity_criterion(){
  return this->use_sanity_criterion;
}

double lccp_segmentation::get_zmin(){
  return this->zmin;
}

double lccp_segmentation::get_zmax(){
  return this->zmax;
}

int lccp_segmentation::get_th_points(){
  return this->th_points;
}

void lccp_segmentation::addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
                                                         PointCloud &adjacent_supervoxel_centers, std::string supervoxel_name, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer){
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
  //Iterate through all adjacent points, and add a center point to adjacent point pair
  PointCloud::iterator adjacent_itr = adjacent_supervoxel_centers.begin();
  for( ; adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr){
    points->InsertNextPoint(supervoxel_center.data);
    points->InsertNextPoint(adjacent_itr->data);
  }
  //Create a polydata to store everything
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  //Add the points to the dataset
  polyData->SetPoints(points);
  polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
  for(unsigned int i = 0; i<points->GetNumberOfPoints();i++)
    polyLine->GetPointIds()->SetId(i,i);
  cells->InsertNextCell(polyLine);
  //Add the lines to the dataset
  polyData->SetLines(cells);
  viewer->addModelFromPolyData(polyData, supervoxel_name);
}

void lccp_segmentation::detectObjectsOnTable(CloudPtr cloud, double zmin, double zmax, pcl::PointIndices::Ptr objectIndices, bool filter_input_cloud){
  //objects for storing point clouds
  CloudPtr plane(new PointCloud);
  CloudPtr convexHull(new PointCloud);

  //Get the plane model, if present
  pcl::SACSegmentation<PointT> segmentation;
  segmentation.setInputCloud(cloud);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.01);
  segmentation.setOptimizeCoefficients(true);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  segmentation.segment(*planeIndices, this->plane_coefficients_);

  if(planeIndices->indices.size() == 0)
    std::cout<<"Could not find a plane in the scene."<<std::endl;
  else{
    //Copy the points of the plane to a new cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(planeIndices);
    extract.filter(*plane);

    //Retrive the convex hull
    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(plane);
    hull.setDimension(2);
    hull.reconstruct(*convexHull);

    //redundant check
    if(hull.getDimension() == 2){
      //prism object
      pcl::ExtractPolygonalPrismData<PointT> prism;
      prism.setInputCloud(cloud);
      prism.setInputPlanarHull(convexHull);
      prism.setHeightLimits(zmin, zmax);
      prism.segment(*objectIndices);
      extract.setIndices(objectIndices);
      if(filter_input_cloud)
        extract.filter(*cloud);
    }
    else std::cout<<"The chosen hull is not planar."<<std::endl;
    this->table_plane_cloud_ = plane;
  }
}


void lccp_segmentation::show_table_plane(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer){
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(this->table_plane_cloud_);
  viewer->addPointCloud<PointT>(this->table_plane_cloud_, rgb, "table_plane_cloud");
}

void lccp_segmentation::show_segmented_objects(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer){
  pcl::PointCloud<PointTl> objects_cloud;
  for(int i = 0; i < this->detected_objects_.size(); ++i){
    pcl::PointCloud<PointTl> tmp_cloud;
    for(int p = 0; p < this->detected_objects_[i].obj_cloud.size(); ++p){
      PointTl tmp_point;
      tmp_point.x = this->detected_objects_[i].obj_cloud[p].x;
      tmp_point.y = this->detected_objects_[i].obj_cloud[p].y;
      tmp_point.z = this->detected_objects_[i].obj_cloud[p].z;
      tmp_point.label = this->detected_objects_[i].label;
      tmp_cloud.points.push_back(tmp_point);
    }
    objects_cloud += tmp_cloud;
  }
  viewer->addPointCloud(objects_cloud.makeShared(), "segmented_object_cloud");
}

void lccp_segmentation::clean_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer){
  viewer->removePointCloud("supervoxel_cloud");
  viewer->removePointCloud("supervoxel_normals");
  viewer->removePointCloud("segemented_object_cloud");
  viewer->removePointCloud("table_plane_cloud");
  std::multimap<uint32_t, uint32_t>::iterator label_itr = (this->supervoxel_adjacency_).begin();
  for( ; label_itr !=(this->supervoxel_adjacency_).end(); ){
    uint32_t supervoxel_label = label_itr->first;
    std::stringstream ss;
    ss<<"supervoxel_"<<supervoxel_label;
    viewer->removeShape(ss.str());
    label_itr = (this->supervoxel_adjacency_).upper_bound(supervoxel_label);
  }
}

std::vector<PointCloud> lccp_segmentation::get_segmented_objects_simple(){
  std::vector<PointCloud> obj_vec;
  for(int i = 0; i < this->detected_objects_.size(); ++i)
    obj_vec.push_back(detected_objects_[i].obj_cloud);
  return obj_vec;
}






