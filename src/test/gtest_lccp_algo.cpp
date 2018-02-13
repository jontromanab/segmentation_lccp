#include<segmentation_lccp/segmentation_lccp.h>
#include<gtest/gtest.h>

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

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
