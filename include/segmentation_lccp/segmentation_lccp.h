#ifndef SEGMENTATION_LCCP_H
#define SEGMENTATION_LCCP_H

#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/console/parse.h>
#include<pcl/io/pcd_io.h>

#include<pcl/segmentation/supervoxel_clustering.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/segmentation/extract_polygonal_prism_data.h>
#include<pcl/segmentation/lccp_segmentation.h>

#include<pcl/ModelCoefficients.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/surface/concave_hull.h>

#include<pcl/visualization/pcl_visualizer.h>

///typedefs
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr CloudPtr;
typedef pcl::PointXYZL PointTl;
typedef pcl::PointCloud<PointTl> PointCloudl;
typedef PointCloudl::Ptr CloudPtrl;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList supervoxelAdjacencyList;

/**
 * \brief Structure containing point cloud of the object and label of the
 * object returned by LCCP algorithm
*/
struct  Object
{
   ///PointCloud of the object
   PointCloud obj_cloud;
   /// label assigned by LCCP algorithm
   int label;
};

/**
 * \brief Class to manage all supervoxel and lccp segmentation parameters
*/
class supervoxel_parameters
{
protected:
  // ------------------ Default Parameters ------------------
  // supervoxel parameters

  ///default value of disable_transform for supervoxel algorithm
  static const bool DISABLE_TRANSFORM = true;
   ///default value of voxel_resolution for supervoxel algorithm
  static const double VOXEL_RESOLUTION = 0.0075f;
  ///default value of seed_resolution for supervoxel algorithm
  static const double SEED_RESOLUTION = 0.03f;
  ///default value of color_importance for supervoxel algorithm
  static const double COLOR_IMPORTANCE = 0.0f;
  ///default value of spatial_importance for supervoxel algorithm
  static const double SPATIAL_IMPORTANCE = 1.0f;
  ///default value of normal_importance for supervoxel algorithm
  static const double NORMAL_IMPORTANCE = 4.0f;

  // LCCP segmentation parameters
  ///default value of concavity_tolerance_threshold of lccp algorithm
  static const double CONCAVITY_TOLERANCE_THRESHOLD = 10;
  ///default value of smoothness_tolerance_threshold of lccp algorithm
  static const double SMOOTHNESS_THRESHOLD = 0.1f;
  ///default value of min_segment_size of lccp algorithm
  static const int MIN_SEGMENT_SIZE = 3;
  ///default value of use_extended_convexity of lccp algorithm
  static const bool USE_EXTENDED_CONVEXITY = false;
  ///default value of use_sanity_criterion of lccp algorithm
  static const bool USE_SANITY_CRITERION = true;

  // other parameters
  ///default value of min distance for object detection on the table
  static const double ZMIN = 0.02;
  ///default value of max distance for object detection on the table
  static const double ZMAX = 2.;
  ///default value of the threshold of min points required to consider a cluster as valid
  static const int TH_POINTS = 400;


public:
  // supervoxel parameters
  ///value of disable_transform for supervoxel algorithm
  bool disable_transform;
  ///value of voxel_resolution for supervoxel algorithm
  double voxel_resolution;
  ///value of seed_resolution for supervoxel algorithm
  double seed_resolution;
  ///value of color_importance for supervoxel algorithm
  double color_importance;
  ///value of spatial_importance for supervoxel algorithm
  double spatial_importance;
  ///value of normal_importance for supervoxel algorithm
  double normal_importance;

  // lccp parameters
  ///value of concavity_tolerance_threshold of lccp algorithm
  double concavity_tolerance_threshold;
  ///value of smoothness_threshold of lccp algorithm
  double smoothness_threshold;
  ///value of min_segment_size of lccp algorithm
  int min_segment_size;
  ///value of use_extended_convexity of lccp algorithm
  bool use_extended_convexity;
  ///value of use_sanity_criterion of lccp algorithm
  bool use_sanity_criterion;

  // other parameters
  ///value of min distance for object detection on the table
  double zmin;
  ///value of max distance for object detection on the table
  double zmax;
  ///value of the threshold of min points required to consider a cluster as valid
  int th_points;



  /**
   * \brief Constructor
   * Sets all parameters to default values
   */
  supervoxel_parameters();

  /**
   * \brief Destructor
   */
  ~supervoxel_parameters();
};


/**
 * \brief Class to detect table top objects in a cluttered scene segmenting the point cloud.
 * Algorithm is based on
 *  http://docs.pointclouds.org/trunk/classpcl_1_1_l_c_c_p_segmentation.html
*/
class lccp_segmentation
{
protected:
  // ------------------ Default Parameters ------------------
  // supervoxel parameters

  ///default value of disable_transform for supervoxel algorithm
  static const bool DISABLE_TRANSFORM = true;
   ///default value of voxel_resolution for supervoxel algorithm
  static const double VOXEL_RESOLUTION = 0.0075f;
  ///default value of seed_resolution for supervoxel algorithm
  static const double SEED_RESOLUTION = 0.03f;
  ///default value of color_importance for supervoxel algorithm
  static const double COLOR_IMPORTANCE = 0.0f;
  ///default value of spatial_importance for supervoxel algorithm
  static const double SPATIAL_IMPORTANCE = 1.0f;
  ///default value of normal_importance for supervoxel algorithm
  static const double NORMAL_IMPORTANCE = 4.0f;

  // LCCP segmentation parameters
  ///default value of concavity_tolerance_threshold of lccp algorithm
  static const double CONCAVITY_TOLERANCE_THRESHOLD = 10;
  ///default value of smoothness_threshold of lccp algorithm
  static const double SMOOTHNESS_THRESHOLD = 0.1f;
  ///default value of min_segment_size of lccp algorithm
  static const int MIN_SEGMENT_SIZE = 3;
  ///default value of use_extended_convexity of lccp algorithm
  static const bool USE_EXTENDED_CONVEXITY = false;
  ///default value of use_sanity_criterion of lccp algorithm
  static const bool USE_SANITY_CRITERION = true;

  // other parameters
  ///default value of min distance for object detection on the table
  static const double ZMIN = 0.02;
  ///default value of max distance for object detection on the table
  static const double ZMAX = 2.;
  ///default value of the threshold of min points required to consider a cluster as valid
  static const int TH_POINTS = 400;

  // supervoxel parameters
  ///value of disable_transform for supervoxel algorithm
  bool disable_transform;
  ///value of voxel_resolution for supervoxel algorithm
  double voxel_resolution;
  ///value of seed_resolution for supervoxel algorithm
  double seed_resolution;
  ///value of color_importance for supervoxel algorithm
  double color_importance;
  ///value of spatial_importance for supervoxel algorithm
  double spatial_importance;
  ///value of normal_importance for supervoxel algorithm
  double normal_importance;

  // lccp parameters
  ///value of concavity_tolerance_threshold of lccp algorithm
  double concavity_tolerance_threshold;
  ///value of smoothness_tolerance_threshold of lccp algorithm
  double smoothness_threshold;
  ///value of min_segment_size of lccp algorithm
  int min_segment_size;
  ///value of use_extended_convexity of lccp algorithm
  bool use_extended_convexity;
  ///value of use_sanity_criterion of lccp algorithm
  bool use_sanity_criterion;

  // other parameters
  ///value of min distance for object detection on the table
  double zmin;
  ///value of max distance for object detection on the table
  double zmax;
  ///value of the threshold of min points required to consider a cluster as valid
  int th_points;

  ///Vector of detected objects
  std::vector<Object> detected_objects;
  ///Input cloud
  CloudPtr cloud_;
  ///Table plane cloud
  CloudPtr table_plane_cloud_;
  ///lccp labeled cloud
  CloudPtrl lccp_labeled_cloud_;
  ///multimap for supervoxel adjacency
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency_;
  ///labeled voxel cloud
  CloudPtrl labeled_voxel_cloud_;
  ///map of supervoxel clusters
  std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxel_clusters_;
  ///normal cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud_;
  ///Coefficients of normal plane
  pcl::ModelCoefficients plane_coefficients_;

  ///variable to keep track if the class is initialized
  bool initialized_;

  /**
   * @brief set default parameters of the algorithm
   */
  void set_default_parameters();

  /**
   * @brief set_parameters set parameters of the algorithm
   * @param opt supervoxel parameters
   */
  void set_parameters(const supervoxel_parameters& opt);

  /**
   * @brief addSupervoxelConnectionsToViewer shows supervoxel connectsions to viewer by poly data shapes
   * @param supervoxel_center center points of the supervoxels
   * @param adjacent_supervoxel_centers point cloud of the adjacent supervoxel centers
   * @param supervoxel_name name of the poly shape
   * @param viewer visualizer
   */
  void addSupervoxelConnectionsToViewer(PointT& supervoxel_center, PointCloud& adjacent_supervoxel_centers, std::string supervoxel_name,
                                        boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);

  /**
   * @brief detectObjectsOnTable detects objects on table
   * @param cloud input cloud
   * @param zmin minimum distance perpendicular to table(meters)
   * @param zmax maximum distance perpendicular to table(meters)
   * @param objectIndices indices of the points belonging to the objects
   * @param filter_input_cloud bool to filter the input cloud
   */
  void detectObjectsOnTable(CloudPtr cloud, double zmin, double zmax, pcl::PointIndices::Ptr objectIndices, bool filter_input_cloud);


public:

  /**
   * @brief Constructor
   */
  lccp_segmentation();

  /**
   * @brief Destructor
   */
  ~lccp_segmentation();

  /**
   * @brief init initializer
   * @param input_cloud input cloud
   * @param opt supervoxel parameters
   */
  void init(PointCloud input_cloud, supervoxel_parameters& opt);

  /**
   * @brief reset resets all public members
   */
  void reset();

  /**
   * @brief get_default_parameters get the default parameters of the algorithm
   * @return supervoxel parameters
   */
  supervoxel_parameters get_default_parameters();

  /**
   * @brief segment Detects and segments objects on the table
   * @return True if there is atleast one object on the table, else false
   */
  bool segment();

  /**
   * @brief show_super_voxels shows supervoxels in the viewer
   * @param viewer viewer on which shows the supervoxels
   * @param show_adjacency_map shows the connection of the supervoxels
   * @param show_super_voxel_normals shows the normals of the supervoxels
   */
  void show_super_voxels(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, bool show_adjacency_map, bool show_super_voxel_normals);

  /**
   * @brief show_super_voxels shows supervoxels in the viewer
   * @param viewer viewer on which shows the supervoxels
   */
  void show_super_voxels(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer);

  /**
   * @brief show_segmented_objects shows segmented objects
   * @param viewer viewer on which shows the semented objects
   */
  void show_segmented_objects(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);

  /**
   * @brief show_table_plane shows the table plane
   * @param viewer viewer on which shows the table plane
   */
  void show_table_plane(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);

  /**
   * @brief clean_viewer cleans the viewer
   * @param viewer viewer to clean up
   */
  void clean_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);

  /**
   * @brief get_segmented_objects get detected objects
   * @return a vector of struct Object
   */
  std::vector<Object> get_segmented_objects();

  /**
   * @brief get_segmented_objects get detected objects
   * @return  a vector of point clouds
   */
  std::vector<PointCloud> get_segmented_objects_simple();

  /**
   * @brief print_parameters prints the parameters of the algorithm
   */
  void print_parameters();

  /**
   * @brief get_input_cloud returns input cloud
   * @return pointcloud
   */
  CloudPtr get_input_cloud();

  /**
   * @brief get_plane_cloud returns plane cloud
   * @return pointcloud
   */
  CloudPtr get_plane_cloud();

  /**
   * @brief get_labeled_voxel_cloud returns labeled voxel cloud
   * @return PointCloud<PointXYZL>
   */
  PointCloudl get_labeled_voxel_cloud();

  /**
   * @brief get_supervoxel_adjacency returns supervoxel adjacency
   */
  std::multimap<uint32_t, uint32_t> get_supervoxel_adjacency();

  /**
   * @brief get_supervoxel_clusters returns supervoxel clusters
   */
  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> get_supervoxel_clusters();

  /**
   * @brief get_normal_cloud returns normal point cloud of the supervoxels
   * @return
   */
  pcl::PointCloud<pcl::PointNormal> get_normal_cloud();

  /**
   * @brief get_plane_coefficients returns table plane coefficients
   * @return
   */
  pcl::ModelCoefficients get_plane_coefficients();

  // setting parameters
  ///set disable transform
  void set_disable_transform(bool disable_transform_in);
  ///set voxel resolution
  void set_voxel_resolution(double voxel_resolution_in);
  ///set seed resolution
  void set_seed_resolution(double seed_resolution_in);
  ///set color importance
  void set_color_importance(double color_importance_in);
  ///set spatial importance
  void set_spatial_importance(double spatial_importance_in);
  ///set normal importance
  void set_normal_importance(double normal_importance_in);

  ///set concavity_tolerance_threshold
  void set_concavity_tolerance_threshold(double concavity_tolerance_threshold_in);
  ///set smoothness threshold
  void set_smoothness_threshold(double smoothness_threshold_in);
  ///set min segment size
  void set_min_segment_size(int min_segment_size_in);
  ///set use extended convexity
  void set_use_extended_convexity(bool use_extended_convexity_in);
  ///set use sanity criterion
  void set_use_sanity_criterion(bool use_sanity_criterion_in);

  /// set zmin
  void set_zmin(double zmin_in);
  ///set zmax
  void set_zmax(double zmax_in);
  ///set th points
  void set_th_points(int th_points_in);


  // Getting parameters
  ///get disable transform
  bool get_disable_transform();
  ///get voxel resolution
  double get_voxel_resolution();
  ///get seed resolution
  double get_seed_resolution();
  ///get color importance
  double get_color_importance();
  ///get spatial importance
  double get_spatial_importance();
  ///get normal importance
  double get_normal_importance();

  ///get concavity tolerance threshold
  double get_concavity_tolerance_threshold();
  ///get smoothness threshold
  double get_smoothness_threshold();
  ///get min segment size
  int get_min_segment_size();
  ///get use extended convexity
  bool get_use_extended_convexity();
  ///get use sanity criterion
  bool get_use_sanity_criterion();

  ///get zmin
  double get_zmin();
  ///get zmax
  double get_zmax();
  ///get th_points
  int get_th_points();
};





#endif // SEGMENTATION_LCCP_H
