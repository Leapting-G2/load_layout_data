#include "global_path_node.h"

int main(int argc, char**argv){

  ros::init(argc, argv,"global_path_node");
  ros::NodeHandle NodeHandle;
  std::unique_ptr<global_path_node::Global_path_node> global_path_node_(new global_path_node::Global_path_node(NodeHandle));
  constexpr size_t num_threads = 2;
  if (num_threads > 1) {
    ros::AsyncSpinner spinner(num_threads); // Uses multi threads
    spinner.start();
    ros::waitForShutdown();
  } else {
    ros::spin();  // Uses single thread
  }
  return 0;
}