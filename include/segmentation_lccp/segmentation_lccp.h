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
   PointCloud obj_cloud;
   int label; /// label assigned by LCCP algorithm
};

/**
 * \brief Class to manage all supervoxel and lccp segmentation parameters
*/
class supervoxel_parameters
{
protected:
  // ------------------ Default Parameters ------------------
  /// supervoxel parameters
  static const bool DISABLE_TRANSFORM = true; ///default value of disable_transform for supervoxel algorithm
  static const double VOXEL_RESOLUTION = 0.0075f; ///default value of voxel_resolution for supervoxel algorithm
  static const double SEED_RESOLUTION = 0.03f; ///default value of seed_resolution for supervoxel algorithm
  static const double COLOR_IMPORTANCE = 0.0f; ///default value of color_importance for supervoxel algorithm
  static const double SPATIAL_IMPORTANCE = 1.0f; ///default value of spatial_importance for supervoxel algorithm
  static const double NORMAL_IMPORTANCE = 4.0f; ///default value of normal_importance for supervoxel algorithm

public:
  //supervoxel parameters
  bool disable_transform; ///value of disable_transform for supervoxel algorithm
  double voxel_resolution; ///value of voxel_resolution for supervoxel algorithm
}





#endif // SEGMENTATION_LCCP_H
