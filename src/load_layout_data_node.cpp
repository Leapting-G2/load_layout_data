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
namespace load_layout_data_node {

double path_out_offset_begin_rand, path_out_offset_end_rand;
std::string are_name;
// double block_properties1, block_properties2, block_properties3;
// std::vector<double> ext_T;
pcl::PointCloud<PointType>::Ptr positionCloud(new pcl::PointCloud<PointType>());
double path_out_offset_begin, path_out_offset_end, cleaner_width;
std::string target_name, target_block_name, target_row_name, target_row_header, solar_add_height_topic_frameid;
int target_start_row_number;
bool start_end;
nav_msgs::Path target_path;
nav_msgs::Path global_path;
nav_msgs::Path back_path;
bool path_init;
std::vector<std::string> block_name;
bool back_to_station;
double brake_dis;
bool target_finish;
int joy_status;  // 1急停按下;2手动;3自动
bool debug;

struct Solar_Session {
    std::pair<double, double> start_point;
    std::pair<double, double> end_point;
    double height;
};

struct gap_action {
    std::pair<double, double> gap_point;
    int up_down;  // cleaner up:1 ;cleaner down 2; hold,turn_arm: 0; lock_arm: 3; arm_up: 4; arm_down: 5;
    double solar_height;
    double arm_angle;
    bool done;  // have pub trig
};

struct road_node {
    std::string node_name;
    std::pair<double, double> position;
    std::vector<std::string> connect_list;
    std::vector<double> lenght_list;
};

std::map<std::string, std::vector<Solar_Session>> map_data;
int row_size;
int action_index = -1;
std::vector<gap_action> gap_actions;
std::vector<gap_action> global_gap_actions;
std::vector<gap_action> row_gap_actions;
std::vector<road_node> road_nodes;
std::vector<std::pair<std::string, std::string>> segline_names;
road_node station_in, station_out, base_node;
geometry_msgs::Pose curr_robot_pose;

inline Eigen::Matrix4d vec_matrix(const std::vector<double> ex_transfer) {
    Eigen::Matrix4d result_matrix;
    result_matrix.setIdentity(4, 4);
    if (ex_transfer.size() != 6) {
        return result_matrix;
    }
    Eigen::Vector3d eulerAngle(ex_transfer[3], ex_transfer[4], ex_transfer[5]);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    Eigen::Vector4d trans_tmp;
    trans_tmp << ex_transfer[0], ex_transfer[1], ex_transfer[2], 1.0;
    result_matrix.block<3, 3>(0, 0) = rotation_matrix;
    result_matrix.block<4, 1>(0, 3) = trans_tmp;
    return result_matrix;
}

inline double pair_dis(std::pair<double, double> pos1, std::pair<double, double> pos2) {
    return sqrt((pos1.first - pos2.first) * (pos1.first - pos2.first) + (pos1.second - pos2.second) * (pos1.second - pos2.second));
}

/**
 * 检查给定的线段是否已经存在于线段列表中，或者存在其反向的线段，以避免重复添加相同的线段。
 */
inline bool in_seglines(const std::vector<std::pair<std::string, std::string>>& list, std::pair<std::string, std::string> q_name) {
    if (q_name.first == q_name.second) {
        return true;
    }
    bool in_list = false;
    if (list.size() > 0) {
        for (int i = 0; i < list.size(); i++) {
            if (((q_name.first == list[i].first) && (q_name.second == list[i].second)) ||
                ((q_name.first == list[i].second) && (q_name.second == list[i].first))) {
                in_list = true;
            }
        }
    }
    return in_list;
}

Load_Layout_Data_Node::Load_Layout_Data_Node(ros::NodeHandle nh) {
    ros::NodeHandle nh_param("~");
    std::string robot_pose_topic, cleaner_path_topic, stop_trig_topic, arm_turn_topic, solar_add_height_topic, cleaner_targrt_finish_topic,
        cleaner_mission_targrt_topic, arm_trig_topic;
    nh_param.param<std::string>("are_name", are_name, "leapting_site");
    nh_param.param<std::string>("robot_pose_topic", robot_pose_topic, "/robot_pose");
    nh_param.param<std::string>("cleaner_path_topic", cleaner_path_topic, "/cleaner_path");
    nh_param.param<std::string>("stop_trig_topic", stop_trig_topic, "/stop_trig");
    nh_param.param<std::string>("arm_trig_topic", arm_trig_topic, "/arm_up_down");
    nh_param.param<std::string>("arm_turn_topic", arm_turn_topic, "/arm_turn_pose");
    nh_param.param<std::string>("solar_add_height_topic", solar_add_height_topic, "/solar_add_height");
    nh_param.param<std::string>("solar_add_height_topic_frameid", solar_add_height_topic_frameid, "tag_pose_cleaner_grab");
    nh_param.param<std::string>("cleaner_mission_targrt_topic", cleaner_mission_targrt_topic, "/cleaner_target");
    nh_param.param<std::string>("cleaner_targrt_finish_topic", cleaner_targrt_finish_topic, "/target_finish");
    nh_param.param<double>("path_out_offset_begin", path_out_offset_begin, 0.0);
    nh_param.param<double>("path_out_offset_end", path_out_offset_end, 0.0);
    nh_param.param<double>("cleaner_width", cleaner_width, 0.5);
    nh_param.param<double>("brake_dis", brake_dis, 0.2);
    nh_param.param<bool>("debug", debug, false);
    // nh_param.param<std::vector<double>>("layoutATmap_T", ext_T, std::vector<double>());
    // ROS_INFO_STREAM("are_name: " << are_name);

    std::string are_heard = "/" + are_name;  // are_leapting
    Eigen::Matrix4d are_to_map;
    std::vector<double> v_are_to_map;
    nh_param.param<std::vector<double>>(are_heard + "/are_to_map", v_are_to_map, std::vector<double>());  // 从参数服务器获取变换矩阵
    are_to_map = vec_matrix(v_are_to_map);
    std::vector<double> station_param;
    nh_param.param<std::vector<double>>(are_heard + "/station", station_param, std::vector<double>());  // 从参数服务器获取进出集装箱的位置

    XmlRpc::XmlRpcValue param_list_are;

    /**
     * 根据rosparam参数，生成拓扑地图
     */
    if (nh_param.getParam(are_heard + "/node_list", param_list_are) == true) {
        std::map<std::string, std::pair<double, double>> temp_map;
        for (int i = 0; i < param_list_are.size(); i++) {
            std::string node_name = param_list_are[i]["node_name"];
            std::pair<double, double> temp_pos(param_list_are[i]["param"][0], param_list_are[i]["param"][1]);
            std::pair<std::string, std::pair<double, double>> temp_map_chip(node_name, temp_pos);
            temp_map.insert(temp_map_chip);
        }
        /**
         * 查询相邻点，计算距离，并将连接关系放入到road_node_list中
         */
        for (int i = 0; i < (int)param_list_are.size(); i++) {
            road_node road_node_chip;
            std::string node_name = param_list_are[i]["node_name"];
            road_node_chip.node_name = node_name;
            Eigen::Vector4d temp_pose, result_pose;
            temp_pose[0] = param_list_are[i]["param"][0];
            temp_pose[1] = param_list_are[i]["param"][1];
            temp_pose[2] = 0.0;
            temp_pose[3] = 1.0;
            result_pose = are_to_map * temp_pose;  // 转换为地图坐标系下坐标
            road_node_chip.position.first = result_pose[0];
            road_node_chip.position.second = result_pose[1];
            // if (param_list_are[i]["param"].size() % 2 != 0) {
            //     ROS_ERROR_STREAM("Data error in this node: " << node_name);
            // }
            // for (int k = 2; k < param_list_are[i]["param"].size(); k = k + 2) {
            //     std::pair<double, double> pos_temp(param_list_are[i]["param"][k], param_list_are[i]["param"][k + 1]);
            //     for (auto iter = temp_map.begin(); iter != temp_map.end(); iter++) {
            //         if (iter->second == pos_temp) {
            //             road_node_chip.connect_list.push_back(iter->first);
            //             road_node_chip.lenght_list.push_back(pair_dis(road_node_chip.position, pos_temp));
            //             std::pair<std::string, std::string> segline_name_chip(iter->first, road_node_chip.node_name);
            //             if (false == in_seglines(segline_names, segline_name_chip)) {
            //                 segline_names.push_back(segline_name_chip);
            //             }
            //         }
            //     }
            // }

            /**
             * 查询连接节点
             */
            for (int k = 2; k < param_list_are[i]["param"].size(); k++) {
                std::string connect_node_name = param_list_are[i]["param"][k];
                std::cout << "connect_node_name " << connect_node_name << std::endl;
                // 便利所有节点，找到连接节点
                for (auto iter = temp_map.begin(); iter != temp_map.end(); iter++) {
                    if (iter->first == connect_node_name) {
                        road_node_chip.connect_list.push_back(iter->first);                                            // 放入连接名称
                        road_node_chip.lenght_list.push_back(pair_dis(road_node_chip.position, iter->second));         // 放入欧式距离
                        std::pair<std::string, std::string> segline_name_chip(iter->first, road_node_chip.node_name);  // 连接的线段
                        if (false == in_seglines(segline_names, segline_name_chip)) {
                            segline_names.push_back(segline_name_chip);
                        }
                    }
                }
            }
            if (node_name == "base_node") {
                road_node_chip.connect_list.push_back("station_out");
                std::pair<double, double> p_temp(station_param[2], station_param[3]);
                road_node_chip.lenght_list.push_back(pair_dis(road_node_chip.position, p_temp));
                base_node = road_node_chip;
            }
            road_nodes.push_back(road_node_chip);
        }
        // station_in, station_out
        station_in.node_name = "station_in";
        std::pair<double, double> p_in(station_param[0], station_param[1]);
        std::pair<double, double> p_out(station_param[2], station_param[3]);
        station_in.position = p_in;
        station_in.connect_list.push_back("station_out");
        station_in.lenght_list.push_back(pair_dis(p_in, p_out));
        road_nodes.push_back(station_in);
        station_out.node_name = "station_out";
        station_out.position = p_out;
        station_out.connect_list.push_back("station_in");
        station_out.lenght_list.push_back(pair_dis(p_in, p_out));
        station_out.connect_list.push_back("base_node");
        station_out.lenght_list.push_back(pair_dis(base_node.position, p_out));
        road_nodes.push_back(station_out);

        std::cout << "road_nodes.size() " << road_nodes.size() << std::endl;
        std::cout << "segline_names.size() " << segline_names.size() << std::endl;
        for (int i = 0; i < segline_names.size(); i++) {
            std::cout << "i " << i << std::endl;
            std::cout << "segline_names[i].first " << segline_names[i].first << std::endl;
            std::cout << "segline_names[i].second " << segline_names[i].second << std::endl;
        }
        for (int i = 0; i < road_nodes.size(); i++) {
            std::cout << "i " << i << std::endl;
            std::cout << "road_nodes[i].node_name " << road_nodes[i].node_name << std::endl;
            for (int j = 0; j < road_nodes[i].connect_list.size(); j++) {
                std::cout << "road_nodes[i].connect_list[j] " << road_nodes[i].connect_list[j] << std::endl;
            }
        }
    } else {
        ROS_ERROR("Failed to get parameter from %s ", are_name.c_str());
    }

    nh_param.param<std::vector<std::string>>(are_heard + "/block_list", block_name, std::vector<std::string>());
    init_data();

    /**
     * 加载地图数据，并将每一排，每一块放入到map_data中
     */
    if (block_name.size() > 0) {
        for (int m = 0; m < (int)(block_name.size()); m++) {  // huzhou_block
            ROS_INFO_STREAM("block_name: " << block_name[m]);
            Eigen::Matrix4d block_to_map;
            std::string param_heard = "/" + block_name[m];
            // nh_param.param<double>(param_heard + "/block_properties1", block_properties1, 0.0);
            // nh_param.param<double>(param_heard + "/block_properties2", block_properties2, 0.0);
            std::vector<double> v_block_to_map;
            nh_param.param<std::vector<double>>(param_heard + "/block_to_map", v_block_to_map, std::vector<double>());
            // Eigen::Vector3d eulerAngle(v_block_to_map[3], v_block_to_map[4], v_block_to_map[5]);
            // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
            // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
            // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
            // Eigen::Matrix3d rotation_matrix;
            // rotation_matrix = yawAngle * pitchAngle * rollAngle;
            // Eigen::Vector4d trans_tmp;
            // trans_tmp << v_block_to_map[0], v_block_to_map[1], v_block_to_map[2], 1.0;
            // block_to_map.setIdentity(4, 4);
            // block_to_map.block<3, 3>(0, 0) = rotation_matrix;
            // block_to_map.block<4, 1>(0, 3) = trans_tmp;
            block_to_map = vec_matrix(v_block_to_map);
            double path_offset_solar_x, path_offset_solar_y;
            nh_param.param<double>(param_heard + "/path_offset_solar_x", path_offset_solar_x, 0.0);
            nh_param.param<double>(param_heard + "/path_offset_solar_y", path_offset_solar_y, 0.0);

            XmlRpc::XmlRpcValue param_list;
            // std::map<std::string, std::vector<Solar_Session>> map_data;
            if (nh_param.getParam(param_heard + "/solar_row_list", param_list) == true) {
                // std::cout << "param_list.size() " << param_list.size() << std::endl;
                // std::cout << "param_list[0][param].size() " << param_list[0]["param"].size() << std::endl;
                // std::cout << "Test 1: " << param_list[0]["row_name"] << std::endl;
                // std::cout << "Test 2: " << param_list[0]["param"] << std::endl;
                // std::cout << "Test 3: " << param_list[0]["param"][0] << std::endl;
                std::cout << "row_size: " << param_list.size() << std::endl;

                for (int i = 0; i < param_list.size(); i++) {
                    std::string row_name = param_list[i]["row_name"];
                    std::vector<Solar_Session> session_value;  // start，end，heigh
                    int param_size = param_list[i]["param"].size();
                    if (param_size % 5 != 0) {
                        ROS_ERROR_STREAM("Data error in this row: " << (block_name[m] + "/" + row_name));
                    }
                    // 处理每一块
                    for (int j = 0; j < param_size; j = j + 5) {
                        Solar_Session session_value_chip;
                        PointType pt;
                        Eigen::Vector4d temp_pose, result_pose;
                        temp_pose[0] = param_list[i]["param"][j];
                        temp_pose[1] = param_list[i]["param"][j + 1];
                        temp_pose[0] = temp_pose[0] + path_offset_solar_x;  // 偏移
                        temp_pose[1] = temp_pose[1] + path_offset_solar_y;
                        temp_pose[2] = 0.0;
                        temp_pose[3] = 1.0;
                        result_pose = block_to_map * temp_pose;  // 坐标转换
                        session_value_chip.start_point.first = result_pose[0];
                        session_value_chip.start_point.second = result_pose[1];
                        pt.x = result_pose[0];
                        pt.y = result_pose[1];
                        pt.z = 0.0;
                        positionCloud->push_back(pt);  // sec_start
                        temp_pose[0] = param_list[i]["param"][j + 2];
                        temp_pose[1] = param_list[i]["param"][j + 3];
                        temp_pose[0] = temp_pose[0] + path_offset_solar_x;
                        temp_pose[1] = temp_pose[1] + path_offset_solar_y;
                        result_pose = block_to_map * temp_pose;
                        session_value_chip.end_point.first = result_pose[0];
                        session_value_chip.end_point.second = result_pose[1];
                        pt.x = result_pose[0];
                        pt.y = result_pose[1];
                        positionCloud->push_back(pt);                               // sec_end
                        session_value_chip.height = param_list[i]["param"][j + 4];  // sec_heigh
                        session_value.push_back(session_value_chip);                // 存放所有sec（start posetion，end position，heigh）
                    }
                    std::string full_row_name = block_name[m] + "/" + row_name;
                    std::pair<std::string, std::vector<Solar_Session>> map_data_chip(
                        full_row_name, session_value);  // 存放所有sec（start posetion，end position，heigh）
                    row_size++;
                    map_data.insert(map_data_chip);
                }

                std::cout << "row_size: " << row_size << std::endl;
                std::cout << "map_data.size(): " << map_data.size() << std::endl;

                for (auto iter = map_data.begin(); iter != map_data.end(); iter++) {
                    std::cout << iter->first << std::endl;
                    for (int i = 0; i < iter->second.size(); i++) {
                        std::cout << "Session: " << i + 1 << std::endl;
                        std::cout << "row_start_point: " << iter->second[i].start_point.first << ", " << iter->second[i].start_point.second
                                  << std::endl;
                        std::cout << "row_end_point: " << iter->second[i].end_point.first << ", " << iter->second[i].end_point.second << std::endl;
                        std::cout << "Session_height: " << iter->second[i].height << std::endl;
                    }
                    std::cout << std::endl;
                }
            } else {
                ROS_ERROR("Failed to get parameter from %s ", block_name[m].c_str());
            }
        }
    } else {
        ROS_ERROR("Failed to get block name list!!! ");
    }

    float start_x, start_y, end_x, end_y;
    std::vector<std::pair<float, float>> point_sec;
    for (auto iter = map_data.begin(); iter != map_data.end(); iter++) {
        for (int i = 0; i < iter->second.size(); i++) {
            start_x = iter->second[i].start_point.first;
            start_y = iter->second[i].start_point.second;
            end_x = iter->second[i].end_point.first;
            end_y = iter->second[i].end_point.second;
            std::pair<float, float> start_point_sec(start_x, start_y);
            std::pair<float, float> end_point_sec(end_x, end_y);
            point_sec.push_back(start_point_sec);
            point_sec.push_back(end_point_sec);
        }
    }
    // 发布点云
    sensor_msgs::PointCloud2 pub_cloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pub_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto iter = point_sec.begin(); iter != point_sec.end(); iter++) {
        PointType pt;
        pt.x = iter->first;
        pt.y = iter->second;
        pt.z = 0.0;
        pub_cloud->push_back(pt);
    }

    pcl::toROSMsg(*pub_cloud, pub_cloud2);
    pub_cloud2.header.frame_id = "map";
    // pub_point.publish(pub_cloud2);

    pub_stop_trig = nh.advertise<std_msgs::Header>(stop_trig_topic.c_str(), 10, true);
    pub_arm_trig = nh.advertise<std_msgs::Header>(arm_trig_topic.c_str(), 10, true);
    pub_target_path = nh.advertise<nav_msgs::Path>(cleaner_path_topic.c_str(), 10, true);
    pub_arm_turn = nh.advertise<geometry_msgs::Twist>(arm_turn_topic.c_str(), 10, true);
    pub_arm_add_height = nh.advertise<geometry_msgs::PoseStamped>(solar_add_height_topic.c_str(), 10, true);
    pub_stop_cloud = nh.advertise<sensor_msgs::PointCloud2>("/gap_point_cloud", 10);
    pub_slow_trig = nh.advertise<std_msgs::Header>("/slow_trig", 10);
    sub_robot_pose = nh.subscribe(robot_pose_topic.c_str(), 10, &Load_Layout_Data_Node::robot_pose_subCallback, this);
    pub_point = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 10, true);

    pub_point.publish(pub_cloud2);
    printf("pub_cloud2:size: %d\n", pub_cloud2.data.size());

    // hz5_timer = nh.createTimer(ros::Duration(0.2), &Load_Layout_Data_Node::Timer5hzCallback, this);
    // hz50_timer = nh.createTimer(ros::Duration(0.02), &Load_Layout_Data_Node::Timer50hzCallback, this);

    sub_target_trig = nh.subscribe(cleaner_mission_targrt_topic.c_str(), 10, &Load_Layout_Data_Node::target_trig_callback, this);
    sub_target_finish_trig = nh.subscribe(cleaner_targrt_finish_topic.c_str(), 10, &Load_Layout_Data_Node::target_finish_trig_callback, this);
    sub_joy_status = nh.subscribe("/joy_status", 10, &Load_Layout_Data_Node::joy_status_callback, this);

    // std::thread data_process(&Load_Layout_Data_Node::process, this);
    // data_process.detach();
}
Load_Layout_Data_Node::~Load_Layout_Data_Node() {}

void Load_Layout_Data_Node::joy_status_callback(const std_msgs::HeaderConstPtr& msg) {
    joy_status = msg->seq;  // 1急停按下;2手动;3自动
}

template <class Type>
Type stringToNum(const std::string& str) {
    std::istringstream iss(str);
    Type num;
    iss >> num;

    return num;
}

typedef std::string::size_type string_size;
std::vector<std::string> splitString(const std::string& s, const std::string& seperator) {
    std::vector<std::string> result;
    string_size i = 0;
    while (i != s.size()) {
        int flag = 0;
        while (i != s.size() && flag == 0) {
            flag = 1;
            for (string_size k = 0; k < seperator.size(); k++) {
                if (s[i] == seperator[k]) {
                    i++;
                    flag = 0;
                    break;
                }
            }
        }
        flag = 0;
        string_size j = i;
        while (j != s.size() && flag == 0) {
            for (string_size k = 0; k < seperator.size(); k++) {
                if (s[j] == seperator[k]) {
                    flag = 1;
                    break;
                }
            }
            if (flag == 0)
                j++;
        }
        if (i != j) {
            result.push_back(s.substr(i, j - i));
            i = j;
        }
    }
    return result;
}

// two point dis
double point_dis(std::pair<double, double> a, std::pair<double, double> b) {
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

// ex_pa true: extent out pa; false: extent in pa
std::pair<double, double> extension_line(std::pair<double, double> pa, std::pair<double, double> pb, double ex_lenght, bool ex_pa) {
    std::pair<double, double> result_p;
    // if (pa.first == pb.first) {
    //     result_p.first = pa.first;
    //     if (ex_pa == true) {
    //         result_p.second = pa.second > pb.second ? (pa.second + ex_lenght) : (pa.second - ex_lenght);
    //     } else {
    //         result_p.second = pa.second > pb.second ? (pa.second - ex_lenght) : (pa.second + ex_lenght);
    //     }
    // } else if (pa.second == pb.second) {
    //     result_p.second = pa.second;
    //     if (ex_pa == true) {
    //         result_p.first = pa.first > pb.first ? (pa.first + ex_lenght) : (pa.first - ex_lenght);
    //     } else {
    //         result_p.first = pa.first > pb.first ? (pa.first - ex_lenght) : (pa.first + ex_lenght);
    //     }
    // } else {
    double dis = point_dis(pa, pb);
    double normal_x = (pa.first - pb.first) / dis;
    double normal_y = (pa.second - pb.second) / dis;
    if (ex_pa == true) {
        result_p.first = (dis + ex_lenght) * normal_x + pb.first;
        result_p.second = (dis + ex_lenght) * normal_y + pb.second;
    } else {
        result_p.first = (dis - ex_lenght) * normal_x + pb.first;
        result_p.second = (dis - ex_lenght) * normal_y + pb.second;
    }
    // }
    return result_p;
}

// a to b
void get_angle(std::pair<double, double> pa, std::pair<double, double> pb, double& roll, double& pitch, double& yaw) {
    double dis = point_dis(pa, pb);
    double normal_x = (pb.first - pa.first) / dis;
    double normal_y = (pb.second - pa.second) / dis;
    if (fabs(normal_x) > 0.001) {
        yaw = acos(normal_x);
        if (normal_y < 0) {
            yaw = -1.0 * yaw;
        }
    } else {
        if (normal_y >= 0) {
            yaw = 1.570796;
        } else {
            yaw = -1.570796;
        }
    }
    pitch = 0.0;
    roll = 0.0;
    // std::cout << "roll " << roll << std::endl;
    // std::cout << "pitch " << pitch << std::endl;
    // std::cout << "pa " << pa.first << std::endl;
    // std::cout << "normal_x " << normal_x << std::endl;
    // std::cout << "normal_y " << normal_y << std::endl;
    // std::cout << "yaw " << yaw << std::endl;
}

geometry_msgs::Quaternion get_quaternion(double roll_rad, double pitch_rad, double yaw_rad) {
    Eigen::Vector3d eulerAngle(roll_rad, pitch_rad, yaw_rad);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle;
    geometry_msgs::Quaternion g_quaternion;
    g_quaternion.x = quaternion.x();
    g_quaternion.y = quaternion.y();
    g_quaternion.z = quaternion.z();
    g_quaternion.w = quaternion.w();
    return g_quaternion;
}

struct Node {
    road_node x;
    double F, G, H;  // F=G+H
    std::shared_ptr<Node> parent;
    Node(road_node _x) : x(_x), F(0), G(0), H(0), parent(NULL) {}
};
double calcG(const std::shared_ptr<Node> lastNode, const std::shared_ptr<Node> targetNode) {
    int index = -1;
    for (int i = 0; i < lastNode->x.connect_list.size(); i++) {
        if (lastNode->x.connect_list[i] == targetNode->x.node_name) {
            index = i;
        }
    }
    double addG;
    if (index == -1) {
        printf("%s and %s are not connect! \n", (lastNode->x.node_name).c_str(), (targetNode->x.node_name).c_str());
        addG = 9999999.0;
    } else {
        addG = lastNode->x.lenght_list[index];
    }
    double parentG = lastNode->parent == NULL ? 0 : lastNode->parent->G;  // first node
    return parentG + addG;
}
double calcH(const std::shared_ptr<Node> theNode, const std::shared_ptr<Node> endNode) {
    return pair_dis(theNode->x.position, endNode->x.position);
}
double calcF(const std::shared_ptr<Node> node) {
    return node->G + node->H;
}
std::shared_ptr<Node> getLeastFnode(const std::list<std::shared_ptr<Node>>& inList) {
    if (!inList.empty()) {
        auto resNode = inList.front();
        for (auto& node : inList) {
            if (node->F < resNode->F) {
                resNode = node;
            }
        }
        return resNode;
    }
    return NULL;
}
bool isInList(const std::list<std::shared_ptr<Node>>& list, const std::shared_ptr<Node> node) {
    for (auto& p : list) {
        if (p->x.node_name == node->x.node_name) {
            return true;
        }
    }
    return false;
}
std::vector<std::shared_ptr<Node>> getSurroundNodes(const std::shared_ptr<Node> node) {
    std::vector<std::shared_ptr<Node>> surroundNodes;
    int num_s = node->x.connect_list.size();
    if (num_s < 1) {
        printf("can not find %s connections data error! \n", (node->x.node_name).c_str());
        return surroundNodes;
    }
    for (int k = 0; k < num_s; k++) {
        for (int i = 0; i < road_nodes.size(); i++) {
            if (node->x.connect_list[k] == road_nodes[i].node_name) {
                std::shared_ptr<Node> get_node(new Node(road_nodes[i]));
                surroundNodes.push_back(get_node);
                break;
            }
        }
    }
    return surroundNodes;
}

/**
 * a星算法查找最短路径
 */
bool find_path(const road_node road_node_start, const road_node road_node_end, std::vector<road_node>& result_list) {
    ros::Time start_time = ros::Time::now();
    int result_len = 0;
    result_list.clear();
    int index_s = -1, index_e = -1;
    for (int i = 0; i < road_nodes.size(); i++) {
        if (road_node_start.node_name == road_nodes[i].node_name) {
            index_s = i;
        }
        if (road_node_end.node_name == road_nodes[i].node_name) {
            index_e = i;
        }
    }
    if ((index_s == -1) || (index_e == -1)) {
        printf("can not find %s and %s node in map! \n", (road_node_start.node_name).c_str(), (road_node_end.node_name).c_str());
        return false;
    }
    if (index_s == index_e) {
        result_list.push_back(road_node_end);
        return true;
    }
    std::shared_ptr<Node> start_node(new Node(road_nodes[index_s]));
    std::shared_ptr<Node> end_node(new Node(road_nodes[index_e]));
    std::list<std::shared_ptr<Node>> openList, closeList;
    openList.push_back(start_node);
    bool get_target = false;
    while ((get_target == false) && (!openList.empty())) {
        if (ros::Time::now() - start_time > ros::Duration(10.0)) {
            ROS_ERROR_STREAM("Path search timeout!");
            break;
        }
        auto curNode = getLeastFnode(openList);  // find the min
        openList.remove(curNode);
        closeList.push_back(curNode);
        // 1,get surround
        auto surroundNodes = getSurroundNodes(curNode);
        for (auto& target : surroundNodes) {
            // 2,not in openlist, renew G H F
            if (!isInList(closeList, target)) {
                if (!isInList(openList, target)) {
                    target->parent = curNode;
                    target->G = calcG(curNode, target);   // before
                    target->H = calcH(target, end_node);  // predict
                    target->F = calcF(target);            // sum
                    openList.push_back(target);
                } else {  // 3, in openlist ,compare G, renew
                    int tempG = calcG(curNode, target);
                    if (tempG < target->G) {
                        target->parent = curNode;
                        target->G = tempG;
                        target->F = calcF(target);
                    }
                }
            }
        }
        // 4. find the end
        for (auto p : openList) {
            if (p->x.node_name == end_node->x.node_name) {
                openList.remove(p);
                closeList.push_back(p);
                get_target = true;
                break;
            }
        }
    }
    if (get_target == true) {
        result_len = closeList.size();
        std::list<road_node> temp_list;
        std::shared_ptr<Node> result = closeList.back();
        while (result) {
            temp_list.push_front(result->x);  // start .... end
            result = result->parent;
        }
        for (auto it = temp_list.begin(); it != temp_list.end(); ++it) {
            result_list.push_back(*it);
        }
        openList.clear();
        closeList.clear();
        // delete start_node;
        // delete end_node;
        return true;
    }
    result_list.clear();
    return false;
}

// point S to line ab, a-->b
double disPoint2Line(std::pair<double, double> a, std::pair<double, double> b, std::pair<double, double> s) {
    double x1 = a.first;
    double y1 = a.second;
    double x2 = b.first;
    double y2 = b.second;
    double x0 = s.first;
    double y0 = s.second;
    double distance = fabs(((y1 - y2) * x0 - (x1 - x2) * y0 + (x1 * y2 - x2 * y1)) / sqrt(pow(y1 - y2, 2) + pow(x1 - x2, 2)));
    return distance;
}

void get_line(std::pair<double, double> p1, std::pair<double, double> p2, double& a, double& b, double& c) {
    if (fabs(p2.second - p1.second) < 0.001) {
        a = 0.0;
        b = 1.0;
        c = -p2.second;
    } else if (fabs(p2.first - p1.first) < 0.001) {
        a = 1.0;
        b = 0.0;
        c = -p2.first;
    } else {
        a = 1.0;
        b = -1.0 * (p2.first - p1.first) / (p2.second - p1.second);
        c = -1.0 * (p1.first + b * p1.second);
    }
}

// ax+by+c=0
bool getCrossPoint(PointType l1, PointType l2, std::pair<double, double>& crossPoint) {
    if (l1.y == l2.y && (fabs(l1.y) < 0.000001)) {
        return false;
    } else if ((!(fabs(l1.y) < 0.000001)) && (!(fabs(l2.y) < 0.000001)) && (l1.x / l1.y == l2.x / l2.y)) {
        return false;
    } else {
        if ((fabs(l1.y) < 0.000001)) {
            double cx = -1.0 * l1.z / l1.x;
            crossPoint.second = -1.0 * (l2.x * cx + l2.z) / l2.y;
            crossPoint.first = cx;
        } else if ((fabs(l2.y) < 0.000001)) {
            double cx = -1.0 * l2.z / l2.x;
            crossPoint.second = -1.0 * (l1.x * cx + l1.z) / l1.y;
            crossPoint.first = cx;
        } else {
            double cx = (l2.z / l2.y - l1.z / l1.y) / (l1.x / l1.y - l2.x / l2.y);
            crossPoint.second = -1.0 * l1.x / l1.y * cx - l1.z / l1.y;
            crossPoint.first = cx;
        }
    }
    return true;
}

// point s, line: ax+by+c = 0
std::pair<double, double> GetFootOfPerpendicular(std::pair<double, double> s, double a, double b, double c) {
    std::pair<double, double> retVal;
    if (fabs(b) < 0.000001) {
        retVal.first = (int)(-1.0 * c / a);
        retVal.second = s.second;
        return retVal;
    } else if (fabs(a) < 0.000001) {
        retVal.first = s.first;
        retVal.second = (int)(-1.0 * c / b);
        return retVal;
    } else if (fabs(a * s.first + b * s.second + c) < 0.000001) {
        retVal.first = s.first;
        retVal.second = s.second;
        return retVal;
    } else {
        double tempx = (-1.0) * (a * c + a * b * s.second - b * b * s.first) / (a * a + b * b);
        retVal.first = (int)(tempx);
        retVal.second = (int)((-1.0) * (a * tempx + c) / b);
        return retVal;
    }
}

// c in range of a and b
bool in_range(double a, double b, double c) {
    bool in_r = false;
    if (a >= b) {
        if (c <= a && c >= b) {
            in_r = true;
        }
    } else {
        if (c <= b && c >= a) {
            in_r = true;
        }
    }
    return in_r;
}

// pc between pa and pb
bool between_line(std::pair<double, double> pa, std::pair<double, double> pb, std::pair<double, double> pc) {
    // double a, b, c;
    // get_line(pa, pb, a, b, c);
    // std::pair<double, double> foot_p = GetFootOfPerpendicular(pc, a, b, c);
    // bool between_flag = false;
    // if ((in_range(pa.first, pb.first, foot_p.first)) && (in_range(pa.second, pb.second, foot_p.second))) {
    //     between_flag = true;
    // }
    // return between_flag;
    bool between_flag = true;
    std::pair<double, double> v_ab(pb.first - pa.first, pb.second - pa.second);
    std::pair<double, double> v_ac(pc.first - pa.first, pc.second - pa.second);
    std::pair<double, double> v_ba(pa.first - pb.first, pa.second - pb.second);
    std::pair<double, double> v_bc(pc.first - pb.first, pc.second - pb.second);
    double angle_a = v_ab.first * v_ac.first + v_ab.second * v_ac.second;
    double angle_b = v_ba.first * v_bc.first + v_ba.second * v_bc.second;
    if (angle_a < 0 || angle_b < 0) {
        between_flag = false;
    }
    return between_flag;
}
/**
 * 在一组道路段（由节点对表示）中找到与给定线段（由两个点表示）相交的道路段
 */
bool find_seg(const std::vector<std::pair<std::string, std::string>>& seg_list,
              const std::vector<road_node>& node_list,
              std::pair<double, double> pa,
              std::pair<double, double> pb,
              std::pair<road_node, road_node>& seg_line) {
    std::vector<std::pair<road_node, road_node>> closet_node;
    for (int i = 0; i < seg_list.size(); i++) {
        road_node road_node1, road_node2;
        for (int j = 0; j < node_list.size(); j++) {
            if (seg_list[i].first == node_list[j].node_name) {
                road_node1 = node_list[j];  // base_node,LA_1
            }
            if (seg_list[i].second == node_list[j].node_name) {
                road_node2 = node_list[j];  // LA_1,LA_2
            }
        }
        if (between_line(road_node1.position, road_node2.position, pa)) {
            closet_node.push_back(std::pair<road_node, road_node>(road_node1, road_node2));
        }
    }
    if (closet_node.size() < 1) {
        printf("closet_node zero! \n");
        return false;
    }
    // std::cout << "closet_node.size() " << closet_node.size() << std::endl;
    if (closet_node.size() == 1) {
        seg_line = closet_node[0];
        return true;
    } else {
        double a, b, c;
        get_line(pa, pb, a, b, c);
        PointType l_ab(a, b, c);
        double min_dis = 9999999999;
        int index = -1;
        std::pair<double, double> cross_p;
        for (int i = 0; i < closet_node.size(); i++) {
            get_line(closet_node[i].first.position, closet_node[i].second.position, a, b, c);
            PointType l_node(a, b, c);
            // 直线方程获取直线交点
            if (getCrossPoint(l_ab, l_node, cross_p)) {
                double dis = point_dis(cross_p, pa);
                if (dis < min_dis) {
                    min_dis = dis;
                    index = i;
                }
            }
        }
        if (index != -1) {
            seg_line = closet_node[index];
            return true;
        } else {
            printf("Maybe road node data wrong! \n");
            return false;
        }
    }
}
bool find_seg(const std::vector<std::pair<std::string, std::string>>& seg_list,
              const std::vector<road_node>& node_list,
              std::pair<double, double> pa,
              std::pair<road_node, road_node>& seg_line) {
    std::vector<std::pair<road_node, road_node>> closet_node;
    for (int i = 0; i < seg_list.size(); i++) {
        road_node road_node1, road_node2;
        for (int j = 0; j < node_list.size(); j++) {
            if (seg_list[i].first == node_list[j].node_name) {
                road_node1 = node_list[j];
            }
            if (seg_list[i].second == node_list[j].node_name) {
                road_node2 = node_list[j];
            }
        }
        if (between_line(road_node1.position, road_node2.position, pa)) {
            closet_node.push_back(std::pair<road_node, road_node>(road_node1, road_node2));
        }
    }
    if (closet_node.size() < 1) {
        printf("closet_node zero! \n");
        return false;
    }
    // std::cout << "closet_node.size() " << closet_node.size() << std::endl;
    if (closet_node.size() == 1) {
        seg_line = closet_node[0];
        return true;
    } else {
        double a, b, c;
        double min_dis = 9999999999;
        int index = -1;
        for (int i = 0; i < closet_node.size(); i++) {
            double dis = disPoint2Line(closet_node[i].first.position, closet_node[i].second.position, pa);
            if (dis < min_dis) {
                min_dis = dis;
                index = i;
            }
        }
        if (index != -1) {
            seg_line = closet_node[index];
            return true;
        } else {
            printf("Maybe road node data wrong! \n");
            return false;
        }
    }
}

void Load_Layout_Data_Node::target_finish_trig_callback(const std_msgs::HeaderConstPtr& msg) {
    target_finish = true;
}

void Load_Layout_Data_Node::target_trig_callback(const std_msgs::HeaderConstPtr& msg) {
    // rand
    srand(time(0));  // 初始化随机数种子
    double random_num = static_cast<double>(rand()) / RAND_MAX * 2.0;
    path_out_offset_begin_rand = path_out_offset_begin + random_num;
    path_out_offset_end_rand = path_out_offset_end + random_num;

    // target row name
    target_name = msg->frame_id;
    std::vector<std::string> results, results_number;
    results = splitString(target_name, "/");
    try {
        target_block_name = results[0];  // huzhou
        target_row_name = results[1];    // A_3
        results_number = splitString(results[1], "_");
        target_row_header = results_number[0];                          // A
        target_start_row_number = stringToNum<int>(results_number[1]);  // 3
    } catch (std::exception& e) {
        ROS_ERROR("Target string foemat ERROR! ");
        return;
    }
    // back to station
    if (results[1] != "0_0") {
        // 0:start--end,1:end--start
        if ((msg->seq % 10) == 0) {
            start_end = true;
            ROS_INFO("From start to end!");
        } else {
            start_end = false;
            ROS_INFO("From end to start!");
        }
        int target_row_size = 0;
        int block_row_size = 0;
        // 查询目标行的行数
        for (auto iter = map_data.begin(); iter != map_data.end(); iter++) {
            std::vector<std::string> results_temp;
            results_temp = splitString(iter->first, "/");
            if (results_temp[0] == target_block_name) {
                block_row_size++;
            }
        }
        if (block_row_size == 0) {
            ROS_ERROR("Maybe block_name ERROR! ");
            return;
        }

        std::vector<std::string> row_names;
        int ten_number = msg->seq % 100;  // 221
        if ((ten_number / 10) == 0) {     // default row_up mode
            ROS_INFO("From row up!");
            target_row_size = block_row_size - target_start_row_number + 1;  // 6-4=2
            std::string row_name_chip;
            // huzhou/A_4
            // huzhou/A_5
            for (int i = 0; i < target_row_size; i++) {
                row_name_chip = target_block_name + "/" + target_row_header + "_" + std::to_string(target_start_row_number + i);
                row_names.push_back(row_name_chip);
                std::cout << "row_name_chip " << row_name_chip << std::endl;
            }
        } else if ((ten_number / 10) == 1) {  // row down mode
            ROS_INFO("From row down!");
            target_row_size = target_start_row_number;  // 3
            std::string row_name_chip;
            // huzhou/A_3
            // huzhou/A_2
            // huzhou/A_1
            for (int i = 0; i < target_row_size; i++) {
                row_name_chip = target_block_name + "/" + target_row_header + "_" + std::to_string(target_start_row_number - i);
                row_names.push_back(row_name_chip);
                std::cout << "row_name_chip " << row_name_chip << std::endl;
            }
        } else if ((ten_number / 10) == 2) {  // to that row
            ROS_INFO("To that row!");
            int end_row_number = (msg->seq / 100);  // 2
            if (end_row_number <= 0) {
                end_row_number = 1;
            }
            target_row_size = abs(end_row_number - target_start_row_number) + 1;  // 2
            std::string row_name_chip;
            if (end_row_number > target_start_row_number) {
                for (int i = 0; i < target_row_size; i++) {
                    row_name_chip = target_block_name + "/" + target_row_header + "_" + std::to_string(target_start_row_number + i);
                    row_names.push_back(row_name_chip);
                }
            } else {
                // huzhou/A_3
                // huzhou/A_2
                for (int i = 0; i < target_row_size; i++) {
                    row_name_chip = target_block_name + "/" + target_row_header + "_" + std::to_string(target_start_row_number - i);
                    row_names.push_back(row_name_chip);
                }
            }
        } else if ((ten_number / 10) == 3) {  // back to station
        } else {
        }

        target_path.poses.clear();
        global_path.poses.clear();
        gap_actions.clear();
        global_gap_actions.clear();
        row_gap_actions.clear();
        std::map<std::string, std::vector<Solar_Session>>::iterator iter;
        bool flag_s_e = true;
        if (start_end == false) {  // false
            flag_s_e = false;
        }

        /**
         * 生成清扫路径
        */
        for (int i = 0; i < target_row_size; i++) {                     // 2
            std::cout << "row_names[i] " << row_names[i] << std::endl;  // huzhou/A_3,huzhou/A_2
            iter = map_data.find(row_names[i]);
            if (iter != map_data.end()) {
                std::pair<double, double> start_p_raw, end_p_raw, start_p, end_p;

                if (flag_s_e == false) {                                            // end to start
                    start_p_raw = iter->second[iter->second.size() - 1].end_point;  // 最后一块板子的终点
                    end_p_raw = iter->second[0].start_point;                        // 第一块板子的起点
                    // 延长线(准备点)
                    start_p = extension_line(start_p_raw, end_p_raw, path_out_offset_end_rand, true);
                    end_p = extension_line(end_p_raw, start_p_raw, path_out_offset_begin_rand, true);

                    // 准备点动作
                    gap_action gap_action_chip0;
                    gap_action_chip0.gap_point = start_p;
                    gap_action_chip0.up_down = 0;  // hold arm turn
                    gap_action_chip0.solar_height = iter->second[iter->second.size() - 1].height;
                    gap_action_chip0.done = false;
                    gap_action_chip0.arm_angle = -1.570796;
                    row_gap_actions.push_back(gap_action_chip0);
                    // for gap detection
                    int j = 0;
                    for (j = 0; j < iter->second.size(); j++) {  // 遍历所有sec
                        gap_action gap_action_chip1;
                        gap_action_chip1.gap_point =
                            extension_line(iter->second[iter->second.size() - 1 - j].end_point, iter->second[iter->second.size() - 1 - j].start_point,
                                           cleaner_width - brake_dis, false);
                        gap_action_chip1.up_down = 2;  // cleaner down
                        gap_action_chip1.solar_height = iter->second[iter->second.size() - 1 - j].height;
                        gap_action_chip1.done = false;
                        gap_action_chip1.arm_angle = -1.570796;
                        row_gap_actions.push_back(gap_action_chip1);
                        gap_action gap_action_chip2;
                        gap_action_chip2.gap_point =
                            extension_line(iter->second[iter->second.size() - 1 - j].start_point, iter->second[iter->second.size() - 1 - j].end_point,
                                           cleaner_width + brake_dis, false);
                        gap_action_chip2.up_down = 1;  // cleaner up
                        gap_action_chip2.solar_height = iter->second[iter->second.size() - 1 - j].height;
                        gap_action_chip2.done = false;
                        gap_action_chip2.arm_angle = -1.570796;
                        row_gap_actions.push_back(gap_action_chip2);
                    }
                    gap_action gap_action_chip3;
                    gap_action_chip3.gap_point = end_p;
                    // if the last row, the last sec, the last chip
                    if ((i == (target_row_size - 1) && (j == iter->second.size()))) {
                        gap_action_chip3.up_down = 0;  // hold arm turn
                    } else {
                        gap_action_chip3.up_down = 3;  // lock arm
                    }
                    gap_action_chip3.solar_height = iter->second[0].height;
                    gap_action_chip3.done = false;
                    gap_action_chip3.arm_angle = -1.570796;
                    row_gap_actions.push_back(gap_action_chip3);

                    flag_s_e = true;
                } else {  // flag_s_e = true start to end
                    start_p_raw = iter->second[0].start_point;
                    end_p_raw = iter->second[iter->second.size() - 1].end_point;
                    start_p = extension_line(start_p_raw, end_p_raw, path_out_offset_begin_rand, true);
                    end_p = extension_line(end_p_raw, start_p_raw, path_out_offset_end_rand, true);
                    gap_action gap_action_chip0;
                    gap_action_chip0.gap_point = start_p;
                    gap_action_chip0.up_down = 0;  // hold arm turn
                    gap_action_chip0.solar_height = iter->second[0].height;
                    gap_action_chip0.done = false;
                    gap_action_chip0.arm_angle = 1.570796;
                    row_gap_actions.push_back(gap_action_chip0);
                    // for gap detection
                    int j = 0;
                    // 遍历所有sec
                    for (j = 0; j < iter->second.size(); j++) {
                        gap_action gap_action_chip1;
                        gap_action_chip1.gap_point =
                            extension_line(iter->second[j].start_point, iter->second[j].end_point, cleaner_width - brake_dis, false);
                        gap_action_chip1.up_down = 2;  // cleaner down
                        gap_action_chip1.solar_height = iter->second[j].height;
                        gap_action_chip1.done = false;
                        gap_action_chip1.arm_angle = 1.570796;
                        row_gap_actions.push_back(gap_action_chip1);
                        gap_action gap_action_chip2;
                        gap_action_chip2.gap_point =
                            extension_line(iter->second[j].end_point, iter->second[j].start_point, cleaner_width + brake_dis, false);
                        gap_action_chip2.up_down = 1;  // cleaner up
                        gap_action_chip2.solar_height = iter->second[j].height;
                        gap_action_chip2.done = false;
                        gap_action_chip2.arm_angle = 1.570796;
                        row_gap_actions.push_back(gap_action_chip2);
                    }
                    gap_action gap_action_chip3;
                    gap_action_chip3.gap_point = end_p;
                    if ((i == (target_row_size - 1) && (j == iter->second.size()))) {
                        gap_action_chip3.up_down = 0;  // hold arm turn
                    } else {
                        gap_action_chip3.up_down = 3;  // lock arm
                    }
                    gap_action_chip3.solar_height = iter->second[iter->second.size() - 1].height;
                    gap_action_chip3.done = false;
                    gap_action_chip3.arm_angle = 1.570796;
                    row_gap_actions.push_back(gap_action_chip3);

                    flag_s_e = false;
                }

                // std::cout << "start_p.first " << start_p.first << std::endl;
                // std::cout << "start_p.second " << start_p.second << std::endl;
                // std::cout << "end_p.first " << end_p.first << std::endl;
                // std::cout << "end_p.second " << end_p.second << std::endl;
                geometry_msgs::PoseStamped pose_start, pose_end;
                pose_start.pose.position.x = start_p.first;
                pose_start.pose.position.y = start_p.second;
                pose_start.pose.position.z = 0.0;
                pose_end.pose.position.x = end_p.first;
                pose_end.pose.position.y = end_p.second;
                pose_end.pose.position.z = 0.0;
                double ang_roll, ang_pitch, ang_yaw;
                get_angle(start_p, end_p, ang_roll, ang_pitch, ang_yaw);
                pose_start.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
                pose_end.pose.orientation = pose_start.pose.orientation;
                target_path.poses.push_back(pose_start);
                target_path.poses.push_back(pose_end);
                path_init = true;
            } else {
                path_init = false;
                target_path.poses.clear();
                row_gap_actions.clear();
                ROS_ERROR_STREAM("Can not find solar row in this name: " << row_names[i]);
                return;
            }
        }

        /**
         * 全局路径
        */
        if (path_init) {
            std::pair<road_node, road_node> turn_seg;
            global_gap_actions.clear();
            /*查询距离清洗起点或终点最短的路径线*/
            if (false == find_seg(segline_names, road_nodes,
                                  std::pair<double, double>(target_path.poses[0].pose.position.x, target_path.poses[0].pose.position.y),
                                  std::pair<double, double>(target_path.poses[1].pose.position.x, target_path.poses[1].pose.position.y), turn_seg)) {
                ROS_ERROR_STREAM("Can not find global segment line near this point: " << target_path.poses[0].pose.position);
                return;
            }
            std::cout << "turn_seg.first.node_name " << turn_seg.first.node_name << std::endl;//LA_2
            std::cout << "turn_seg.second.node_name " << turn_seg.second.node_name << std::endl;//LA_1
            // 根据欧式距离查找最近线段的点
            road_node end_node = point_dis(turn_seg.first.position, base_node.position) > point_dis(turn_seg.second.position, base_node.position)
                                     ? turn_seg.first
                                     : turn_seg.second;
            std::vector<road_node> global_node_list;
            if (find_path(station_in, end_node, global_node_list)) {
                for (int i = 0; i < global_node_list.size() - 1; i++) {
                    geometry_msgs::PoseStamped g_pose_start, g_pose_end;
                    g_pose_start.pose.position.x = global_node_list[i].position.first;
                    g_pose_start.pose.position.y = global_node_list[i].position.second;
                    g_pose_start.pose.position.z = 0.0;
                    g_pose_end.pose.position.x = global_node_list[i + 1].position.first;
                    g_pose_end.pose.position.y = global_node_list[i + 1].position.second;
                    g_pose_end.pose.position.z = 0.0;
                    double ang_roll, ang_pitch, ang_yaw;
                    get_angle(global_node_list[i].position, global_node_list[i + 1].position, ang_roll, ang_pitch, ang_yaw);
                    g_pose_start.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
                    global_path.poses.push_back(g_pose_start);
                    if (i == (global_node_list.size() - 2)) {
                        g_pose_end.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
                        global_path.poses.push_back(g_pose_end);
                    }
                }
                // for (int i = 0; i < global_node_list.size(); i++) {
                //     std::cout << "i " << i << std::endl;
                //     std::cout << "global_node_list[i].node_name " << global_node_list[i].node_name << std::endl;
                // }
                // station out, up arm
                for (int i = 0; i < global_node_list.size(); i++) {
                    if (global_node_list[i].node_name == "station_out") {
                        if (i > 0 && (global_node_list[i - 1].node_name == "station_in")) {
                            gap_action gap_action_chip1;
                            gap_action_chip1.gap_point = global_node_list[i].position;
                            gap_action_chip1.up_down = 4;
                            gap_action_chip1.solar_height = 0;
                            gap_action_chip1.done = false;
                            gap_action_chip1.arm_angle = -3.141592653;
                            global_gap_actions.push_back(gap_action_chip1);
                        } else if ((i < (global_node_list.size() - 1)) && (global_node_list[i + 1].node_name == "station_in")) {
                            gap_action gap_action_chip1;
                            gap_action_chip1.gap_point = global_node_list[i].position;
                            gap_action_chip1.up_down = 5;
                            gap_action_chip1.solar_height = 0;
                            gap_action_chip1.done = false;
                            gap_action_chip1.arm_angle = -3.141592653;
                            global_gap_actions.push_back(gap_action_chip1);
                        } else {
                        }
                    }
                }
            } else {
                path_init = false;
                global_path.poses.clear();
                target_path.poses.clear();
                row_gap_actions.clear();
                global_gap_actions.clear();
                ROS_ERROR_STREAM("Can not find path to: " << end_node.node_name);
            }
        }

        if (path_init == false) {
            target_path.poses.clear();
            row_gap_actions.clear();
            global_gap_actions.clear();
            global_path.poses.clear();
        }

        if (path_init) {
            nav_msgs::Path path_combine;
            path_combine.header.stamp = ros::Time::now();
            path_combine.header.frame_id = "map";
            int solar_path_size = target_path.poses.size();
            std::cout << "solar_path_size " << solar_path_size << std::endl;
            int global_path_size = global_path.poses.size();
            std::cout << "global_path_size " << global_path_size << std::endl;
            if (solar_path_size < 1) {
                ROS_ERROR_STREAM("solar Path node number error: " << solar_path_size);
                return;
            }
            if (global_path_size < 1) {
                ROS_ERROR_STREAM("global Path node number error: " << global_path_size);
                return;
            }
            for (int i = 0; i < global_path_size - 1; i++) {
                path_combine.poses.push_back(global_path.poses[i]);
            }
            for (int i = 0; i < solar_path_size - 1; i++) {
                if (i == 0) {
                    std::pair<double, double> g1(global_path.poses[global_path_size - 1].pose.position.x,
                                                 global_path.poses[global_path_size - 1].pose.position.y);
                    std::pair<double, double> g2(global_path.poses[global_path_size - 2].pose.position.x,
                                                 global_path.poses[global_path_size - 2].pose.position.y);
                    std::pair<double, double> s1(target_path.poses[0].pose.position.x, target_path.poses[0].pose.position.y);
                    std::pair<double, double> s2(target_path.poses[1].pose.position.x, target_path.poses[1].pose.position.y);
                    double a, b, c;
                    get_line(g1, g2, a, b, c);
                    PointType l_g(a, b, c);
                    get_line(s1, s2, a, b, c);
                    PointType l_s(a, b, c);
                    // std::cout << "l_g " << l_g  << std::endl;
                    // std::cout << "l_s " << l_s  << std::endl;
                    std::pair<double, double> cross_p;
                    if (getCrossPoint(l_g, l_s, cross_p)) {
                        if (point_dis(cross_p, s1) > 5.0) {
                            if (start_end) {
                                ROS_ERROR_STREAM("ERROR! Can not get path start from this row: " << target_start_row_number << "'s start position!");
                            } else {
                                ROS_ERROR_STREAM("ERROR! Can not get path start from this row: " << target_start_row_number << "'s end position!");
                            }
                            return;
                        }
                        geometry_msgs::PoseStamped connetct_pose;
                        connetct_pose = target_path.poses[0];
                        connetct_pose.pose.position.x = cross_p.first;
                        connetct_pose.pose.position.y = cross_p.second;
                        // std::cout << "connetct_pose " << connetct_pose  << std::endl;
                        path_combine.poses.push_back(connetct_pose);
                        // cross point:lock arm; cross point+1m: turn arm
                        gap_action gap_action_chip1;
                        gap_action_chip1.gap_point = cross_p;
                        gap_action_chip1.up_down = 3;
                        gap_action_chip1.solar_height = 0;
                        gap_action_chip1.done = false;
                        gap_action_chip1.arm_angle = -3.141592653;
                        global_gap_actions.push_back(gap_action_chip1);
                        // change gap_actions
                        //  for (int i = 0; i < gap_actions.size(); i++) {
                        //      if (gap_actions[i].gap_point == s1) {
                        //          gap_actions[i].gap_point = extension_line(cross_p, gap_actions[i + 1].gap_point, -1.0, true);
                        //      }
                        //  }
                    } else {
                        ROS_ERROR("ERROR! solar path and global path not cross!");
                        return;
                    }
                } else {
                    if ((i % 2) != 0) {
                        geometry_msgs::PoseStamped temp_pose_end;
                        temp_pose_end.pose.position.x = target_path.poses[i].pose.position.x;
                        temp_pose_end.pose.position.y = target_path.poses[i].pose.position.y;
                        temp_pose_end.pose.position.z = target_path.poses[i].pose.position.z;
                        std::pair<double, double> line_start, line_end;
                        line_start.first = target_path.poses[i].pose.position.x;
                        line_start.second = target_path.poses[i].pose.position.y;
                        line_end.first = target_path.poses[i + 1].pose.position.x;
                        line_end.second = target_path.poses[i + 1].pose.position.y;
                        double ang_roll, ang_pitch, ang_yaw;
                        get_angle(line_start, line_end, ang_roll, ang_pitch, ang_yaw);
                        temp_pose_end.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
                        path_combine.poses.push_back(temp_pose_end);
                    } else {
                        path_combine.poses.push_back(target_path.poses[i]);
                    }
                }
            }
            path_combine.poses.push_back(target_path.poses[solar_path_size - 1]);
            std::cout << "path_combine_size " << path_combine.poses.size() << std::endl;
            pub_target_path.publish(path_combine);
            gap_actions.clear();
            for (int i = 0; i < global_gap_actions.size(); i++) {
                gap_actions.push_back(global_gap_actions[i]);
            }
            for (int i = 0; i < row_gap_actions.size(); i++) {
                gap_actions.push_back(row_gap_actions[i]);
            }
            target_finish = false;
        }

        // std::cout << "gap_actions.size() " << gap_actions.size() << std::endl;
        // for (int i = 0; i < gap_actions.size(); i++) {
        //     std::cout << "gap_actions: " << i << std::endl;
        //     std::cout << "gap_actions.gap_point: " << gap_actions[i].gap_point.first << ", " << gap_actions[i].gap_point.second << std::endl;
        //     std::cout << "gap_actions.up_down: " << gap_actions[i].up_down << std::endl;
        //     std::cout << "gap_actions.solar_height: " << gap_actions[i].solar_height << std::endl;
        //     std::cout << "gap_actions.arm_angle: " << gap_actions[i].arm_angle << std::endl;
        //     std::cout << "gap_actions.done: " << gap_actions[i].done << std::endl;
        // }
        // std::cout << std::endl;

        /**
         * 回桩路径
         */
    } else if (results[1] == "0_0") {  // back_to_station == true
        if ((gap_actions.size() != 0) && (target_finish == false)) {
            ROS_WARN_STREAM("Can not back to station, target is not empty!");
            return;
        }
        // clean data
        back_to_station == true;
        // generate new path
        //  back_to_station = false;
        gap_actions.clear();

        std::pair<road_node, road_node> nearest_seg;
        //(线段，节点列表，当前点，最近线段起点与终点)
        // 查询最近的线段
        if (false ==
            find_seg(segline_names, road_nodes, std::pair<double, double>(curr_robot_pose.position.x, curr_robot_pose.position.y), nearest_seg)) {
            ROS_ERROR_STREAM("Can not find global segment line near current robot pose: " << curr_robot_pose.position);
            return;
        }
        // 查询距离充电站最近的节点
        road_node back_start_node =
            point_dis(nearest_seg.first.position, station_in.position) > point_dis(nearest_seg.second.position, station_in.position)
                ? nearest_seg.first
                : nearest_seg.second;
        std::vector<road_node> back_node_list;
        back_path.poses.clear();
        // 利用a start算法查询最短路径
        if (find_path(back_start_node, station_in, back_node_list)) {
            for (int i = 0; i < back_node_list.size() - 1; i++) {
                geometry_msgs::PoseStamped g_pose_start, g_pose_end;
                g_pose_start.pose.position.x = back_node_list[i].position.first;
                g_pose_start.pose.position.y = back_node_list[i].position.second;
                g_pose_start.pose.position.z = 0.0;
                g_pose_end.pose.position.x = back_node_list[i + 1].position.first;
                g_pose_end.pose.position.y = back_node_list[i + 1].position.second;
                g_pose_end.pose.position.z = 0.0;
                double ang_roll, ang_pitch, ang_yaw;
                // get_angle(back_node_list[i].position, back_node_list[i + 1].position, ang_roll, ang_pitch, ang_yaw);
                get_angle(back_node_list[i + 1].position, back_node_list[i].position, ang_roll, ang_pitch, ang_yaw);
                g_pose_start.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
                back_path.poses.push_back(g_pose_start);
                // last point
                if (i == (back_node_list.size() - 2)) {
                    g_pose_end.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
                    back_path.poses.push_back(g_pose_end);
                }
            }
            // cross point
            double a, b, c;
            // 计算直线方程
            get_line(back_node_list[0].position, back_node_list[1].position, a, b, c);
            // 计算垂足
            std::pair<double, double> cross_p =
                GetFootOfPerpendicular(std::pair<double, double>(curr_robot_pose.position.x, curr_robot_pose.position.y), a, b, c);
            // 计算垂足到当前点的距离，保证不出现斜线
            back_path.poses[0].pose.position.x = cross_p.first;
            back_path.poses[0].pose.position.y = cross_p.second;

            // station out, up arm
            for (int i = 0; i < back_node_list.size(); i++) {
                // start point
                if (i == 0) {  // arm trun to 0
                    gap_action gap_action_chip;
                    gap_action_chip.gap_point = cross_p;
                    gap_action_chip.up_down = 0;  // hold
                    gap_action_chip.solar_height = 0;
                    gap_action_chip.done = false;
                    gap_action_chip.arm_angle = -3.141592653;
                    gap_actions.push_back(gap_action_chip);
                }
                if (back_node_list[i].node_name == "station_out") {
                    // station in
                    if (i > 0 && (back_node_list[i - 1].node_name == "station_in")) {
                        gap_action gap_action_chip1;
                        gap_action_chip1.gap_point = back_node_list[i].position;
                        gap_action_chip1.up_down = 4;  // arm_down
                        gap_action_chip1.solar_height = 0;
                        gap_action_chip1.done = false;
                        gap_action_chip1.arm_angle = -3.141592653;
                        gap_actions.push_back(gap_action_chip1);
                    } else if ((i < (back_node_list.size() - 1)) && (back_node_list[i + 1].node_name == "station_in")) {
                        gap_action gap_action_chip1;
                        gap_action_chip1.gap_point = back_node_list[i].position;
                        gap_action_chip1.up_down = 5;  // arm_up
                        gap_action_chip1.solar_height = 0;
                        gap_action_chip1.done = false;
                        gap_action_chip1.arm_angle = -3.141592653;
                        gap_actions.push_back(gap_action_chip1);
                    } else {
                    }
                }
            }
            back_path.header.stamp = ros::Time::now();
            back_path.header.frame_id = "map";
            pub_target_path.publish(back_path);
            action_index = -1;
            target_finish = false;
        } else {
            back_path.poses.clear();
            gap_actions.clear();
            ROS_ERROR_STREAM("Can not find path to: " << station_in.node_name);
        }
    } else {
    }

    if (debug && (gap_actions.size() > 0)) {
        pcl::PointCloud<PointType>::Ptr laserCloud_point(new pcl::PointCloud<PointType>());
        PointType pi;
        for (int i = 0; i < gap_actions.size(); i++) {
            pi.x = gap_actions[i].gap_point.first;
            pi.y = gap_actions[i].gap_point.second;
            pi.z = 0.0;
            laserCloud_point->push_back(pi);
        }
        if (pub_stop_cloud.getNumSubscribers() > 0) {
            sensor_msgs::PointCloud2 laserCloudPoint;
            pcl::toROSMsg(*laserCloud_point, laserCloudPoint);
            laserCloudPoint.header.stamp = ros::Time::now();
            laserCloudPoint.header.frame_id = "map";
            pub_stop_cloud.publish(laserCloudPoint);
        }
    }
}

void Load_Layout_Data_Node::robot_pose_subCallback(const geometry_msgs::Pose msg) {
    curr_robot_pose = msg;
    if (joy_status != 3) {  // 1急停按下;2手动;3自动
        return;
    }
    // if (back_to_station == true) {
    //     //clean data
    //     target_path.poses.clear();
    //     global_path.poses.clear();
    //     gap_actions.clear();
    //     // back_to_station = false;
    //     //publish stop trig
    //     std_msgs::Header stop_trig;
    //     stop_trig.stamp = ros::Time::now();
    //     stop_trig.frame_id = "stop_trig";
    //     stop_trig.seq = 1;
    //     pub_stop_trig.publish(stop_trig);
    // }
    // std::cout << "gap_actions.size() " << gap_actions.size() << std::endl;
    if (gap_actions.size() < 1) {
        return;
    }
    // std::cout << "action_index: " << action_index << std::endl;

    std::pair<double, double> vehicle_pose(msg.position.x, msg.position.y);
    double min_dis = 999.0;
    int min_index = -1;
    for (int i = 0; i < gap_actions.size(); i++) {
        double temp_dis = point_dis(vehicle_pose, gap_actions[i].gap_point);
        if (min_dis > temp_dis) {
            min_dis = temp_dis;
            min_index = i;
        }
    }
    std::cout << "action_index: " << action_index << std::endl;
    std::cout << "min_index: " << min_index << std::endl;
    if (min_dis < 1.0) {
        std::cout << "to_action: " << gap_actions[min_index].up_down << std::endl;
        double stop_dis;
        if (gap_actions[min_index].up_down == 2) {  // down
            double dis_line = disPoint2Line(gap_actions[min_index].gap_point, gap_actions[min_index + 1].gap_point, vehicle_pose);
            stop_dis = sqrt(min_dis * min_dis - dis_line * dis_line);
        } else if (gap_actions[min_index].up_down == 1) {  // up
            double dis_line = disPoint2Line(gap_actions[min_index].gap_point, gap_actions[min_index - 1].gap_point, vehicle_pose);
            stop_dis = sqrt(min_dis * min_dis - dis_line * dis_line);
        } else {
            stop_dis = min_dis;
        }

        if (gap_actions[min_index].up_down == 2 || gap_actions[min_index].up_down == 1) {
            std_msgs::Header slow_trig;
            slow_trig.stamp = ros::Time::now();
            slow_trig.frame_id = "slow_trig";
            slow_trig.seq = 1;
            pub_slow_trig.publish(slow_trig);
        }
        if ((gap_actions[min_index].done == false) && (action_index == -1 ? true : (action_index + 1 == min_index ? true : false))) {
            std::cout << "do_action: " << gap_actions[min_index].up_down << std::endl;
            // publish stop trig, and up or down, one time
            std_msgs::Header stop_trig;
            stop_trig.stamp = ros::Time::now();

            if (action_index == gap_actions.size() - 1) {
                target_finish = true;
            }

            if (gap_actions[min_index].up_down == 1) {  // pick up
                if (stop_dis < 0.10) {
                    stop_trig.frame_id = "stop_trig";
                    stop_trig.seq = 1;
                    pub_stop_trig.publish(stop_trig);
                    gap_actions[min_index].done = true;
                    action_index = min_index;
                }
            } else if (gap_actions[min_index].up_down == 2) {  // put down
                if (stop_dis < 0.10) {
                    stop_trig.frame_id = "stop_trig";
                    stop_trig.seq = 2;
                    pub_stop_trig.publish(stop_trig);
                    gap_actions[min_index].done = true;
                    action_index = min_index;
                }
            } else if (gap_actions[min_index].up_down == 3) {  // lock arm, arm up
                if (stop_dis < 1.0) {
                    // stop_trig.frame_id = "lock_arm";
                    // stop_trig.seq = 10;
                    // pub_stop_trig.publish(stop_trig);
                    // pub arm_up
                    stop_trig.frame_id = "stop_trig";
                    stop_trig.seq = 1;
                    pub_arm_trig.publish(stop_trig);
                    gap_actions[min_index].done = true;
                    action_index = min_index;
                }
            } else if (gap_actions[min_index].up_down == 0) {  // unlock arm, turn arm
                if (stop_dis < 1.0) {
                    // stop_trig.frame_id = "lock_arm";
                    // stop_trig.seq = 0;
                    // pub_stop_trig.publish(stop_trig);
                    // pub arm_turn_pose
                    geometry_msgs::Twist arm_trun_angle;
                    arm_trun_angle.angular.x = -1.0;
                    arm_trun_angle.angular.z = gap_actions[min_index].arm_angle * 180.0 / 3.141592653;
                    pub_arm_turn.publish(arm_trun_angle);
                    gap_actions[min_index].done = true;
                    action_index = min_index;
                }
            } else if (gap_actions[min_index].up_down == 4) {  // up arm
                if (stop_dis < 1.0) {
                    // pub arm_up
                    stop_trig.frame_id = "stop_trig";
                    stop_trig.seq = 1;
                    pub_arm_trig.publish(stop_trig);
                    gap_actions[min_index].done = true;
                    action_index = min_index;
                }
            } else if (gap_actions[min_index].up_down == 5) {  // down arm
                if (stop_dis < 1.0) {
                    // pub arm_up
                    stop_trig.frame_id = "stop_trig";
                    stop_trig.seq = 2;
                    pub_arm_trig.publish(stop_trig);
                    gap_actions[min_index].done = true;
                    action_index = min_index;
                }
            } else {
            }
        }

        // use height
        if (min_index < (gap_actions.size() - 1)) {
            double next_height = gap_actions[min_index + 1].solar_height;
            double this_height = gap_actions[min_index].solar_height;
            geometry_msgs::PoseStamped arm_add_pose;
            arm_add_pose.header.stamp = ros::Time::now();
            arm_add_pose.header.frame_id = solar_add_height_topic_frameid;
            arm_add_pose.pose.position.x = 0.03;
            arm_add_pose.pose.position.y = 0.0;
            if (next_height - this_height <= 0.0) {
                arm_add_pose.pose.position.z = 0.0;
            } else {
                arm_add_pose.pose.position.z = next_height - this_height;
            }
            arm_add_pose.pose.position.z += 0.25;  // 在每排高度基础上增加距离来保证清洗机不挂蹭太阳能板
            arm_add_pose.pose.orientation.x = 0.0;
            arm_add_pose.pose.orientation.y = -0.01745;
            arm_add_pose.pose.orientation.z = 0.0;
            arm_add_pose.pose.orientation.w = 1.0;
            pub_arm_add_height.publish(arm_add_pose);
        }
    } else {
        std_msgs::Header slow_trig;
        slow_trig.stamp = ros::Time::now();
        slow_trig.frame_id = "slow_trig";
        slow_trig.seq = 0;
        pub_slow_trig.publish(slow_trig);
    }
}

// void Load_Layout_Data_Node::Timer50hzCallback(const ros::TimerEvent &) {

// }

// void Load_Layout_Data_Node::Timer5hzCallback(const ros::TimerEvent &) {
//     // ros::Time timenow = ros::Time::now();
//     // if (timenow - can_receive_time > ros::Duration(1.0)) {

//     // }
//     // if (path_init) {
//     //     target_path.header.stamp = timenow;
//     //     target_path.header.frame_id = "map";
//     //     pub_target_path.publish(target_path);
//     // }
// }

// void Load_Layout_Data_Node::process() {
//     ros::Rate rate(10);
//     while (ros::ok()) {

//         rate.sleep();
//     }
// }

void Load_Layout_Data_Node::init_data() {
    row_size = 0;
    path_init = false;
    back_to_station = false;
    target_finish = false;
}

}  // namespace load_layout_data_node
