#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_pose_remap");
    ros::NodeHandle nh;

    ros::Publisher pub_robot_pose = nh.advertise<geometry_msgs::Pose>("/robot_pose", 1);
    ros::Subscriber initpose_sub =
        nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
            pub_robot_pose.publish(msg->pose.pose);

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
            transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                                 msg->pose.pose.orientation.w));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
        });

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
