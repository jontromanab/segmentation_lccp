#ifndef SEGMENTATION_LCCP_H
#define SEGMENTATION_LCCP_H

#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/console/parse.h>
#include<pcl/io/pcd_io.h>

#include<pcl/segmentation/supervoxel_clustering.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/segmentation/extract_polygonal_prism_data.h>

#include<pcl/ModelCoefficients.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/surface/concave_hull.h>

///typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr CloudPtr;

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
  ///value of smoothness_tolerance_threshold of lccp algorithm
  double smoothness_tolerance_hreshold;
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
}





#endif // SEGMENTATION_LCCP_H
