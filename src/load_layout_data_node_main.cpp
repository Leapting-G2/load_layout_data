/******************************************************************************
 * Copyright 2020-2025, zhangsai. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "load_layout_data_node.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "load_layout_data_node");
  ros::NodeHandle NodeHandle;
  std::unique_ptr<load_layout_data_node::Load_Layout_Data_Node> load_layout_data_node_(new load_layout_data_node::Load_Layout_Data_Node(NodeHandle));
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