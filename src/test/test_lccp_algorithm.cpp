#include<segmentation_lccp/segmentation_lccp.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/console/parse.h>

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
    "--NT disables the single cloud transform \n"
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
  return 0;
}
