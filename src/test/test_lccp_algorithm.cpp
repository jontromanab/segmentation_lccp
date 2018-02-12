#include "segmentation_lccp/segmentation_lccp.h"
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/console/parse.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

bool pressed = false;

void keyboardEventOccured(const pcl::visualization::KeyboardEvent& event,
                          void* viewer_void){
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
      *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
  if(event.getKeySym() == "n" && event.keyDown())
    pressed = true;
}

int main(int argc, char** argv){
  if(argc<2){
    pcl::console::print_error("Syntax is: %s <pcd-file> \n"
    " --- supervoxel parameters ---\n"
    "-NT disables the single cloud transform \n"
    "-v <voxel_resolution>\n-s <seed resolution>\n"
    "-c <color weight> \n-z <spatial weight> \n"
    "-n <normal weight> \n"
    "--- LCCP parameters ---\n"
    "-sc disable sanity criterion\n"
    "-ct concavity tolerance\n"
    "-st smoothness threshold\n"
    "-ec enable extended criterion\n"
    "-min_segment min segment size\n"
    " --- other parameters ---\n"
    "-zmin minimum distance orthogonal to table plane\n"
    "-zmax maximum distance orthogonal to table plane\n"
    "-th_points amounts of points to consider as cluster\n"
    "\n", argv[0]);
    return(1);
  }

  //parameters for the algorithm
  supervoxel_parameters param;
  //parsing inputs
  param.disable_transform = pcl::console::find_switch(argc, argv, "-NT");
  pcl::console::parse(argc, argv, "-v", param.voxel_resolution);
  pcl::console::parse(argc, argv, "-s", param.seed_resolution);
  pcl::console::parse(argc, argv, "-c", param.color_importance);
  pcl::console::parse(argc, argv, "-z", param.spatial_importance);
  pcl::console::parse(argc, argv, "-n", param.normal_importance);
  pcl::console::parse(argc, argv, "-ct", param.concavity_tolerance_threshold);
  pcl::console::parse(argc, argv, "-st", param.smoothness_threshold);
  pcl::console::parse(argc, argv, "-min_segment", param.min_segment_size);
  pcl::console::parse(argc, argv, "-zmin", param.zmin);
  pcl::console::parse(argc, argv, "-zmax", param.zmax);
  pcl::console::parse(argc, argv, "-th_points", param.th_points);
  param.use_extended_convexity = pcl::console::find_switch(argc, argv, "-ec");
  param.use_sanity_criterion = pcl::console::find_switch(argc, argv, "-sc");

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::console::print_highlight("Loading point cloud...\n");
  if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud)){
    pcl::console::print_error("Error loading cloud file!\n");
    return(1);
  }

  lccp_segmentation seg;
  std::vector<Object> seg_objs;
  seg.init(*cloud, param);
  seg.print_parameters();
  seg.segment();
  seg_objs = seg.get_segmented_objects();
  std::cout<<"\n Detected "<<seg_objs.size()<<" Objects.\n\n";

  //Visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer
                                                ("3D Viewer"));
  viewer->setBackgroundColor(0,0,0);
  viewer->registerKeyboardCallback(keyboardEventOccured,(void*)&viewer);

  seg.show_table_plane(viewer);
  seg.show_segmented_objects(viewer);
  bool show_adjacency_map = true;
  bool show_super_voxel_normals = false;
  //seg.show_super_voxels(viewer, show_adjacency_map, show_super_voxel_normals);

  std::cout<<"Press 'n' to close\n";
  while(!viewer->wasStopped() && !pressed)
    viewer->spinOnce(100);

  seg.clean_viewer(viewer);



  return 0;
}
