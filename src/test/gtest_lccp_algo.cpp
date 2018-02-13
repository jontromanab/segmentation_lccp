#include<segmentation_lccp/segmentation_lccp.h>
#include<gtest/gtest.h>

/* Calling default Supervoxel parameters
 *
*/

TEST(SuperVoxelParam, checkDefaultParams){
  std::unique_ptr<supervoxel_parameters> param(new supervoxel_parameters);
  EXPECT_TRUE(param->disable_transform);
  EXPECT_FLOAT_EQ(0.0075, param->voxel_resolution);
  EXPECT_FLOAT_EQ(0.03, param->seed_resolution);
  EXPECT_FLOAT_EQ(0.0, param->color_importance);
  EXPECT_FLOAT_EQ(1.0, param->spatial_importance);
  EXPECT_FLOAT_EQ(4.0, param->normal_importance);
  EXPECT_FLOAT_EQ(10.0, param->concavity_tolerance_threshold);
  EXPECT_FLOAT_EQ(0.1, param->smoothness_threshold);
  EXPECT_EQ(3, param->min_segment_size);
  EXPECT_FALSE(param->use_extended_convexity);
  EXPECT_TRUE(param->use_sanity_criterion);
  EXPECT_FLOAT_EQ(0.02, param->zmin);
  EXPECT_FLOAT_EQ(2.0, param->zmax);
  EXPECT_EQ(400, param->th_points);
}

/*
 * LCCP segmentation tests
*/
std::string pcd_file_;

TEST(LccpAlgo, checkDefaultParams){
  std::unique_ptr<lccp_segmentation> seg(new lccp_segmentation);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcd_file_, *cloud);
  seg->init(*cloud);
  EXPECT_TRUE(seg->get_disable_transform());
  EXPECT_FLOAT_EQ(0.0075, seg->get_voxel_resolution());
  EXPECT_FLOAT_EQ(0.03, seg->get_seed_resolution());
  EXPECT_FLOAT_EQ(0.0, seg->get_color_importance());
  EXPECT_FLOAT_EQ(1.0, seg->get_spatial_importance());
  EXPECT_FLOAT_EQ(4.0, seg->get_normal_importance());
  EXPECT_FLOAT_EQ(10.0, seg->get_concavity_tolerance_threshold());
  EXPECT_FLOAT_EQ(0.1, seg->get_smoothness_threshold());
  EXPECT_EQ(3, seg->get_min_segment_size());
  EXPECT_FALSE(seg->get_use_extended_convexity());
  EXPECT_TRUE(seg->get_use_sanity_criterion());
  EXPECT_FLOAT_EQ(0.02, seg->get_zmin());
  EXPECT_FLOAT_EQ(2.0, seg->get_zmax());
  EXPECT_EQ(400, seg->get_th_points());


}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  if(argc<2){
    std::cerr<<"No test files were given. Please add the path of .pcd file to the test"<<std::endl;
    return(-1);
  }
  pcd_file_ = argv[1];
  return RUN_ALL_TESTS();
}
