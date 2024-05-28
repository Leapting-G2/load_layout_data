#ifndef __GLOBAL_PATH_NODE_H__
#define __GLOBAL_PATH_NODE_H__

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "ros/time.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "dynamic_reconfigure/server.h"
#include <dynamic_reconfigure/client.h>
#include <move_base/MoveBaseConfig.h>

namespace global_path_node {

typedef pcl::PointXYZ PointType;
class Global_path_node {
   public:
    Global_path_node(ros::NodeHandle nh);
    ~Global_path_node();

   private:
    ros::Publisher road_node_pub;
    ros::Publisher clean_path_cloud_pub;
    ros::Publisher pub_target_path;
    ros::Publisher pub_global_path_trig;
    ros::Publisher pub_nav_pose;
    ros::Publisher move_goal;
    ros::Publisher charge_path;
    ros::Publisher pub_charge_trig;

    ros::Subscriber sub_robot_pose;
    ros::Subscriber sub_cleaner_nav_path;
    ros::Subscriber sub_charge;

    ros::Timer Hz1_timer;

   private:
    void init_data();
    void cleaner_nav_path_callback(const nav_msgs::Path::ConstPtr& msg);
    void robot_pose_subCallback(const geometry_msgs::Pose& msg);
    void charge_go_callback(const std_msgs::Header& msg);
    // void Timer1hzCallback(const ros::TimerEvent &);
};
}  // namespace global_path_node

#endif