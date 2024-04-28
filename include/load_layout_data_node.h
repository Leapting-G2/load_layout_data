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

#ifndef _LOAD_LAYOUT_DATA_NODE_H_
#define _LOAD_LAYOUT_DATA_NODE_H_

#include "ros/ros.h"
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <thread>
#include <map>
#include <utility>
#include <cstdlib>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "std_msgs/Header.h"
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h> 

namespace load_layout_data_node {
typedef pcl::PointXYZ PointType;
class Load_Layout_Data_Node {
  public:
    Load_Layout_Data_Node(ros::NodeHandle nh);
    ~Load_Layout_Data_Node();

  private:    

    ros::Publisher pub_stop_trig;
    ros::Publisher pub_arm_trig;
    ros::Publisher pub_arm_turn;
    ros::Publisher pub_target_path;
    ros::Publisher pub_arm_add_height;
    ros::Subscriber sub_robot_pose;
    ros::Timer hz5_timer;
    // ros::Timer hz50_timer;
    ros::Subscriber sub_joy_status;
    ros::Subscriber sub_target_trig;
    ros::Subscriber sub_target_finish_trig;
    ros::Publisher pub_stop_cloud;
    ros::Publisher pub_slow_trig;
    
    ros::Publisher pub_point;
    ros::Publisher road_node_pub;
    ros::Publisher pub_point_road_map;
    ros::Publisher pub_point_LA;

  private:   
    void init_data();
    void Timer5hzCallback(const ros::TimerEvent &);
    // void Timer50hzCallback(const ros::TimerEvent &);
    // void process();
    void robot_pose_subCallback(const geometry_msgs::Pose msg);
    void target_trig_callback(const std_msgs::HeaderConstPtr &msg);
    void target_finish_trig_callback(const std_msgs::HeaderConstPtr &msg);
    void joy_status_callback(const std_msgs::HeaderConstPtr &msg);
};
}  // namespace
#endif  //
