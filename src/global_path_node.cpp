#include "global_path_node.h"

namespace global_path_node {

/*-----------------------------------------------------------------------------*/
std::string are_name;
bool debug;
bool path_init = false;
bool layout_path_finish = false;
bool charge_path_finish = false;
int do_index = 0;
bool local_bz = false;
double extended_dis = 0.0;
bool last_charge_trig;

int T3, T4, out, pre, in;

/*拓扑点*/
struct road_node {
    std::string node_name;
    std::pair<double, double> position;
    std::vector<std::string> connect_list;
    std::vector<double> lenght_list;
};

struct Node {
    road_node x;
    double F, G, H;  // F=G+H
    std::shared_ptr<Node> parent;
    Node(road_node _x) : x(_x), F(0), G(0), H(0), parent(NULL) {}  // 结构体构造函数
};

std::vector<std::pair<std::string, std::string>> segline_names;  // 线段
std::vector<road_node> road_nodes;                               // 拓扑点
geometry_msgs::Pose curr_robot_pose;                             // 当前机器人位置

nav_msgs::Path cleaner_nav_path;  // 清扫路径
nav_msgs::Path global_path;       // 全局路径
nav_msgs::Path path_combine;      // 合并路径
nav_msgs::Path charge_combine;    // 充电路径

/*------------------------------------------------------------------------------*/

double geometry_dis(geometry_msgs::Pose pos1, geometry_msgs::Pose pos2) {
    return sqrt((pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
                (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
}

/**
 * 欧式距离
 */
inline double pair_dis(std::pair<double, double> pos1, std::pair<double, double> pos2) {
    return sqrt((pos1.first - pos2.first) * (pos1.first - pos2.first) + (pos1.second - pos2.second) * (pos1.second - pos2.second));
}
/**
 * 判断线段是否在列表中
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

/**
 * 判断点是否在线段之间
 */
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
 * 获取直线方程
 */
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

// two point dis
double point_dis(std::pair<double, double> a, std::pair<double, double> b) {
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

/**
 * 计算两直线交点
  ax+by+c=0
*/

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

/**
 * 计算点到直线的垂足
 */
std::pair<double, double> GetFootOfPerpendicular(std::pair<double, double> s, double a, double b, double c) {
    std::pair<double, double> retVal;
    if (fabs(b) < 0.000001) {
        retVal.first = (-1.0 * c / a);
        retVal.second = s.second;
        return retVal;
    } else if (fabs(a) < 0.000001) {
        retVal.first = s.first;
        retVal.second = (-1.0 * c / b);
        return retVal;
    } else if (fabs(a * s.first + b * s.second + c) < 0.000001) {
        retVal.first = s.first;
        retVal.second = s.second;
        return retVal;
    } else {
        double tempx = (-1.0) * (a * c + a * b * s.second - b * b * s.first) / (a * a + b * b);
        retVal.first = tempx;
        retVal.second = (-1.0) * (a * tempx + c) / b;
        return retVal;
    }
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
    // 遍历所有seg_list，node_list，查找所有的道路段
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
        // 判断pa,pb是否在road_node1,road_node2之间
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
        get_line(pa, pb, a, b, c);  // 获取直线方程
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

/**
 * 从给定节点的连接列表中获取周围的节点，并返回这些节点的共享指针向量
 */
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
    std::shared_ptr<Node> start_node(new Node(road_nodes[index_s]));  // start
    std::shared_ptr<Node> end_node(new Node(road_nodes[index_e]));    // end
    std::list<std::shared_ptr<Node>> openList, closeList;
    openList.push_back(start_node);
    bool get_target = false;
    while ((get_target == false) && (!openList.empty())) {
        if (ros::Time::now() - start_time > ros::Duration(10.0)) {
            ROS_ERROR_STREAM("Path search timeout!");
            break;
        }
        auto curNode = getLeastFnode(openList);  // find the min
        openList.remove(curNode);                //
        closeList.push_back(curNode);
        // 1,get surround
        auto surroundNodes = getSurroundNodes(curNode);  // 获取周围node
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

void addPoseToGlobalPath(nav_msgs::Path& global_path, const geometry_msgs::PoseStamped& new_pose) {
    // 在开始之前，确保global_path.pose至少有一个元素
    if (global_path.poses.empty()) {
        global_path.poses.push_back(new_pose);
        return;
    }

    // 将现有的 pose 后移一位
    for (int i = global_path.poses.size() - 1; i >= 0; --i) {
        global_path.poses[i + 1] = global_path.poses[i];
    }

    // 将新的 pose 插入到首位
    global_path.poses[0] = new_pose;
}
/**
 * 1:left
 * -1:right
 * 0: on the line
 */
int pointPosition(std::pair<double, double> a, std::pair<double, double> b, std::pair<double, double> p) {
    std::pair<double, double> ab, ap;
    ab.first = b.first - a.first;
    ab.second = b.second - a.second;
    ap.first = p.first - a.first;
    ap.second = p.second - a.second;
    double crossProduct = ab.first * ap.second - ab.second * ap.first;

    if (crossProduct > 0) {
        return 1;
        std::cout << "p is on the left of ab" << std::endl;
    } else if (crossProduct < 0) {
        return -1;
        std::cout << "p is on the right of ab" << std::endl;
    } else {
        return 0;
        std::cout << "p is on the ab" << std::endl;
    }
}

/**
 * 计算向量延伸点
 */
geometry_msgs::Point computeExtendedPoint(const geometry_msgs::Point& in, const geometry_msgs::Point& out, double distance) {
    // Calculate the direction vector from in to out
    double dx = out.x - in.x;
    double dy = out.y - in.y;
    double dz = out.z - in.z;

    // Calculate the length of the vector
    double length = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Calculate the unit vector components for the direction
    double unit_x = dx / length;
    double unit_y = dy / length;
    double unit_z = dz / length;

    // Calculate the new point by extending along the direction vector
    geometry_msgs::Point pre;
    pre.x = out.x + unit_x * distance;
    pre.y = out.y + unit_y * distance;
    pre.z = out.z + unit_z * distance;

    return pre;
}

// 判断点 c 是否在矩形范围内
bool isPointInRectangle(const std::pair<double, double>& a1, const std::pair<double, double>& a2, const std::pair<double, double>& c, double offset) {
    // 计算方向向量
    double dx = a2.first - a1.first;
    double dy = a2.second - a1.second;
    double length = std::sqrt(dx * dx + dy * dy);

    // 计算单位向量
    double ux = dx / length;
    double uy = dy / length;

    // 计算垂直方向的单位向量
    double vx = -uy;
    double vy = ux;

    // 计算矩形的四个顶点
    std::pair<double, double> p1 = {a1.first + offset * vx, a1.second + offset * vy};
    std::pair<double, double> p2 = {a1.first - offset * vx, a1.second - offset * vy};
    std::pair<double, double> p3 = {a2.first + offset * vx, a2.second + offset * vy};
    std::pair<double, double> p4 = {a2.first - offset * vx, a2.second - offset * vy};

    // 计算点 c 相对于 p1 的向量
    double vcx = c.first - p1.first;
    double vcy = c.second - p1.second;

    // 计算矩形的边向量
    double vpx1 = p2.first - p1.first;
    double vpy1 = p2.second - p1.second;
    double vpx2 = p3.first - p1.first;
    double vpy2 = p3.second - p1.second;

    // 检查 c 是否在矩形范围内
    double dot1 = (vcx * vpx1 + vcy * vpy1) / (vpx1 * vpx1 + vpy1 * vpy1);
    double dot2 = (vcx * vpx2 + vcy * vpy2) / (vpx2 * vpx2 + vpy2 * vpy2);

    return (0 <= dot1 && dot1 <= 1) && (0 <= dot2 && dot2 <= 1);
}

/*------------------------------------------------------------------------------*/

Global_path_node::Global_path_node(ros::NodeHandle nh) {
    ros::NodeHandle nh_param("~");
    nh_param.param<bool>("debug", debug, false);
    nh_param.param<std::string>("are_name", are_name, "leapting_site");
    nh_param.param<double>("extended_dis", extended_dis, 0.5);
    std::string are_head = "/" + are_name;

    road_node_pub = nh.advertise<sensor_msgs::PointCloud2>("/road_node", 10, true);
    clean_path_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cleaner_path_cloud", 10, true);
    pub_target_path = nh.advertise<nav_msgs::Path>("target_path_global", 10, true);
    pub_nav_pose = nh.advertise<geometry_msgs::Pose>("/nav_pose", 10);
    pub_global_path_trig = nh.advertise<std_msgs::Header>("global_path_trig", 10);
    move_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    charge_path = nh.advertise<nav_msgs::Path>("charge_path", 10, true);
    pub_charge_trig = nh.advertise<std_msgs::Header>("charge_trig", 10);

    sub_cleaner_nav_path = nh.subscribe("/cleaner_nav_path", 10, &Global_path_node::cleaner_nav_path_callback, this);
    sub_robot_pose = nh.subscribe("/robot_pose", 10, &Global_path_node::robot_pose_subCallback, this);
    sub_charge = nh.subscribe("/charge_go", 10, &Global_path_node::charge_go_callback, this);

    // Hz1_timer = nh.createTimer(ros::Duration(1.0), &Global_path_node::Timer1hzCallback, this);

    XmlRpc::XmlRpcValue param_list_are;
    if (nh_param.getParam(are_head + "/node_list", param_list_are) == true) {
        // 计算拓扑地图
        std::map<std::string, std::pair<double, double>> temp_map;
        for (int i = 0; i < param_list_are.size(); i++) {
            std::string node_name = param_list_are[i]["node_name"];                                            // node_name
            std::pair<double, double> temp_pos(param_list_are[i]["param"][0], param_list_are[i]["param"][1]);  // x,y
            std::pair<std::string, std::pair<double, double>> temp_map_chip(node_name, temp_pos);
            temp_map.insert(temp_map_chip);
        }
        // 计算相邻点以及距离
        for (int i = 0; i < (int)param_list_are.size(); i++) {
            road_node road_node_chip;
            std::string node_name = param_list_are[i]["node_name"];
            road_node_chip.node_name = node_name;  // t1,t2,,,,,tn
            road_node_chip.position.first = param_list_are[i]["param"][0];
            road_node_chip.position.second = param_list_are[i]["param"][1];

            // 查询连接点
            for (int k = 2; k < param_list_are[i]["param"].size(); k++) {
                std::string connect_node_name = param_list_are[i]["param"][k];
                for (auto iter = temp_map.begin(); iter != temp_map.end(); iter++) {
                    if (iter->first == connect_node_name) {
                        road_node_chip.connect_list.push_back(iter->first);                                            // name
                        road_node_chip.lenght_list.push_back(pair_dis(road_node_chip.position, iter->second));         // dis
                        std::pair<std::string, std::string> segline_name_chip(iter->first, road_node_chip.node_name);  // segline name
                        if (false == in_seglines(segline_names, segline_name_chip)) {
                            segline_names.push_back(segline_name_chip);
                        }
                    }
                }
            }
            road_nodes.push_back(road_node_chip);  // t1,t2,t3,t4,t5
        }
        for (auto iter = road_nodes.begin(); iter != road_nodes.end(); iter++) {
            if (iter->node_name == "pre") {
                road_nodes.erase(iter);
            }
        }

        /**station*/
        if (nh_param.getParam(are_head + "/station", param_list_are) == true) {
            for (int i = 0; i < param_list_are.size(); i++) {
                std::string node_name = param_list_are[i]["node_name"];
                std::pair<double, double> temp_pos(param_list_are[i]["param"][0], param_list_are[i]["param"][1]);
                std::pair<std::string, std::pair<double, double>> temp_map_chip(node_name, temp_pos);
                temp_map.insert(temp_map_chip);
            }
            for (int i = 0; i < (int)param_list_are.size(); i++) {
                road_node road_node_chip;
                std::string node_name = param_list_are[i]["node_name"];
                road_node_chip.node_name = node_name;
                if (node_name == "pre") {
                    geometry_msgs::Point in, out;
                    in.x = temp_map["in"].first;
                    in.y = temp_map["in"].second;
                    in.z = 0.0;
                    out.x = temp_map["out"].first;
                    out.y = temp_map["out"].second;
                    out.z = 0.0;
                    geometry_msgs::Point pre_point = computeExtendedPoint(in, out, extended_dis);
                    road_node_chip.position.first = pre_point.x;
                    road_node_chip.position.second = pre_point.y;
                    temp_map["pre"].first = pre_point.x;
                    temp_map["pre"].second = pre_point.y;

                } else {
                    road_node_chip.position.first = param_list_are[i]["param"][0];
                    road_node_chip.position.second = param_list_are[i]["param"][1];
                }

                for (int k = 2; k < param_list_are[i]["param"].size(); k++) {
                    std::string connect_node_name = param_list_are[i]["param"][k];
                    for (auto iter = temp_map.begin(); iter != temp_map.end(); iter++) {
                        if (iter->first == connect_node_name) {
                            road_node_chip.connect_list.push_back(iter->first);
                            road_node_chip.lenght_list.push_back(pair_dis(road_node_chip.position, iter->second));
                        }
                    }
                }
                road_nodes.push_back(road_node_chip);
            }
        }

        for (int i = 0; i < road_nodes.size(); i++) {
            if (road_nodes[i].node_name == "t3")
                T3 = i;
            if (road_nodes[i].node_name == "t4")
                T4 = i;
            if (road_nodes[i].node_name == "out")
                out = i;
            if (road_nodes[i].node_name == "pre")
                pre = i;
            if (road_nodes[i].node_name == "in")
                in = i;
        }

        if (debug) {
            std::cout << "road_nodes.size() " << road_nodes.size() << std::endl;
            std::cout << "segline_names.size() " << segline_names.size() << std::endl;

            for (int i = 0; i < segline_names.size(); i++) {
                std::cout << "i " << i << std::endl;
                std::cout << "segline_names[i].first " << segline_names[i].first << std::endl;
                std::cout << "segline_names[i].second " << segline_names[i].second << std::endl;
            }

            pcl::PointCloud<pcl::PointXYZ> cloud;
            for (int i = 0; i < road_nodes.size(); i++) {
                std::cout << "i" << i << std::endl;
                std::cout << "road_name[i].node_name: " << road_nodes[i].node_name << std::endl;
                for (int j = 0; j < road_nodes[i].connect_list.size(); j++) {
                    std::cout << "road_nodes[i].connect_list[j]: " << road_nodes[i].connect_list[j] << std::endl;
                    std::cout << "length: " << road_nodes[i].lenght_list[j] << std::endl;
                }
                cloud.push_back(pcl::PointXYZ(road_nodes[i].position.first, road_nodes[i].position.second, 0));
            }

            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(cloud, cloud_ros);
            cloud_ros.header.frame_id = "map";
            road_node_pub.publish(cloud_ros);
        }
    }
}

Global_path_node::~Global_path_node() {}

void Global_path_node::cleaner_nav_path_callback(const nav_msgs::Path::ConstPtr& msg) {
    if (msg->poses.size() == 0) {
        global_path.poses.clear();
        path_combine.poses.clear();
        charge_combine.poses.clear();

        layout_path_finish = false;
        charge_path_finish = false;
        std::cout << "....................................." << std::endl;
        std::cout << ".............stop action........................" << std::endl;
        std::cout << "....................................." << std::endl;

        geometry_msgs::PoseStamped g_pose;
        g_pose.header.frame_id = "map";
        pub_target_path.publish(g_pose);
        return;
    }
    std::cout << "....................................." << std::endl;
    std::cout << ".............start action........................" << std::endl;
    std::cout << "....................................." << std::endl;
    cleaner_nav_path = *msg;
    global_path.poses.clear();
    layout_path_finish = false;
    charge_path_finish = false;
    path_combine.poses.clear();

    last_charge_trig = isPointInRectangle(std::pair<double, double>(road_nodes[out].position.first, road_nodes[out].position.second),
                                          std::pair<double, double>(road_nodes[in].position.first-1.0, road_nodes[in].position.second),
                                          std::pair<double, double>(curr_robot_pose.position.x, curr_robot_pose.position.y), 1.5);
    if (last_charge_trig)
        std::cout << "库内" << std::endl;
    else
        std::cout << "库外" << std::endl;

    if (debug) {
        std::cout << "cleaner_nav_path.size()" << cleaner_nav_path.poses.size() << std::endl;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (int i = 0; i < cleaner_nav_path.poses.size(); i++) {
            cloud.push_back(pcl::PointXYZ(cleaner_nav_path.poses[i].pose.position.x, cleaner_nav_path.poses[i].pose.position.y, 0));
        }
        sensor_msgs::PointCloud2 cloud_ros;
        pcl::toROSMsg(cloud, cloud_ros);
        cloud_ros.header.frame_id = "map";
        clean_path_cloud_pub.publish(cloud_ros);
    }

    // 查询距离清洗路径最短的线段
    std::pair<road_node, road_node> turn_seg;
    if (false == find_seg(segline_names, road_nodes,
                          std::pair<double, double>(cleaner_nav_path.poses[0].pose.position.x, cleaner_nav_path.poses[0].pose.position.y),
                          std::pair<double, double>(cleaner_nav_path.poses[1].pose.position.x, cleaner_nav_path.poses[1].pose.position.y),
                          turn_seg)) {
        ROS_ERROR_STREAM("Can not find global segment line near this point: " << cleaner_nav_path.poses[0].pose.position);
        return;
    }

    std::cout << "turn_seg.first.node_name " << turn_seg.first.node_name << std::endl;    // t3
    std::cout << "turn_seg.second.node_name " << turn_seg.second.node_name << std::endl;  // t2

    std::vector<road_node> global_node_list;

    // 查询距离车最近的线段点
    road_node nearst_road;
    road_node nearst_segline_nodes =
        point_dis(turn_seg.first.position, std::pair<double, double>(curr_robot_pose.position.x, curr_robot_pose.position.y)) <
                point_dis(turn_seg.second.position, std::pair<double, double>(curr_robot_pose.position.x, curr_robot_pose.position.y))
            ? turn_seg.first
            : turn_seg.second;
    std::cout<<"nearst_segline_nodes: "<<nearst_segline_nodes.node_name<<std::endl;

    // 查询最近的拓扑点
    double min_dis = 9999999;
    int index = -1;
    for (int i = 0; i < road_nodes.size(); i++) {
        double dis = point_dis(road_nodes[i].position, std::pair<double, double>(curr_robot_pose.position.x, curr_robot_pose.position.y));
        if (dis < min_dis) {
            min_dis = dis;
            index = i;
        }
    }
    if (index != -1) {
        nearst_road = road_nodes[index];
        std::cout<<"nearst_road: "<<nearst_road.node_name<<std::endl;
    } else {
        std::cout << "Can not find the nearest node" << std::endl;
        path_init = false;
        return;
    }

    if (nearst_road.node_name != nearst_segline_nodes.node_name) {
        bool find_path_trig;
        if ((nearst_road.node_name == "out" || nearst_road.node_name == "in")&&(!last_charge_trig)) {
            find_path_trig = find_path(road_nodes[pre], nearst_segline_nodes, global_node_list);
        } else {
            find_path_trig = find_path(nearst_road, nearst_segline_nodes, global_node_list);
        }
        if (find_path_trig) {
            for (int i = 0; i < global_node_list.size() - 1; i++) {
                geometry_msgs::PoseStamped g_pose_start, g_pose_end;
                g_pose_start.pose.position.x = global_node_list[i].position.first;
                g_pose_start.pose.position.y = global_node_list[i].position.second;
                g_pose_start.pose.position.z = 0;
                g_pose_end.pose.position.x = cleaner_nav_path.poses[0].pose.position.x;
                g_pose_end.pose.position.y = cleaner_nav_path.poses[0].pose.position.y;
                g_pose_end.pose.position.z = 0;

                double ang_roll, ang_pitch, ang_yaw;
                get_angle(global_node_list[i].position, global_node_list[i + 1].position, ang_roll, ang_pitch, ang_yaw);
                g_pose_start.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
                global_path.poses.push_back(g_pose_start);
                if (i == (global_node_list.size() - 2)) {
                    g_pose_end.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
                    global_path.poses.push_back(g_pose_end);
                }
            }

            std::cout << "global_node: " << std::endl;
            for (int i = 0; i < global_node_list.size(); i++) {
                std::cout << "global_node_list[i].node_name " << global_node_list[i].node_name << std::endl;
            }

            double a, b, c;
            get_line(global_node_list[0].position, global_node_list[1].position, a, b, c);
            std::pair<double, double> foot_point =
                GetFootOfPerpendicular(std::pair<double, double>(curr_robot_pose.position.x, curr_robot_pose.position.y), a, b, c);
            // std::cout << "base: " << global_node_list[0].position.first << " " << global_node_list[0].position.second << std::endl;
            // std::cout << "base2: " << global_node_list[1].position.first << " " << global_node_list[1].position.second << std::endl;
            // std::cout << "current: " << curr_robot_pose.position.x << " " << curr_robot_pose.position.y << std::endl;
            // std::cout << "foot: " << foot_point.first << " " << foot_point.second << std::endl;
            // std::cout << "1->2: " << global_node_list[0].node_name << "->" << global_node_list[1].node_name << std::endl;

            if (between_line(global_node_list[0].position, global_node_list[1].position, foot_point)) {
                std::cout << "int the line...." << std::endl;
                global_path.poses[0].pose.position.x = foot_point.first;
                global_path.poses[0].pose.position.y = foot_point.second;
            } else {
                std::cout << "not int the line...." << std::endl;
                geometry_msgs::PoseStamped start;
                start = global_path.poses[0];
                start.pose.position.x = foot_point.first;
                start.pose.position.y = foot_point.second;
                global_path.poses.insert(global_path.poses.begin(), start);
                global_path.poses[0].pose.position.x = foot_point.first;
                global_path.poses[0].pose.position.y = foot_point.second;
            }

        } else {
            std::cout << "Can not find the path from " << nearst_road.node_name << " to " << nearst_segline_nodes.node_name << std::endl;
            path_init = false;
            return;
        }
    } else {
        double a, b, c;
        auto iter =
            std::find_if(road_nodes.begin(), road_nodes.end(), [&](const auto& node) { return (node.node_name == nearst_segline_nodes.node_name); });
        if (iter != road_nodes.end())
            iter = std::next(iter);
        else
            iter = std::prev(iter);
        get_line(nearst_road.position, iter->position, a, b, c);
        std::pair<double, double> foot_point = GetFootOfPerpendicular(
            std::pair<double, double>(cleaner_nav_path.poses[0].pose.position.x, cleaner_nav_path.poses[0].pose.position.y), a, b, c);

        get_line(nearst_road.position, foot_point, a, b, c);
        std::pair<double, double> car_foot_point =
            GetFootOfPerpendicular(std::pair<double, double>(curr_robot_pose.position.x, curr_robot_pose.position.y), a, b, c);

        if (between_line(car_foot_point, nearst_road.position, foot_point)) {
            geometry_msgs::PoseStamped g_pose_start, g_pose_end;
            g_pose_start.pose.position.x = car_foot_point.first;
            g_pose_start.pose.position.y = car_foot_point.second;
            g_pose_start.pose.position.z = 0;
            g_pose_end.pose.position.x = cleaner_nav_path.poses[0].pose.position.x;
            g_pose_end.pose.position.y = cleaner_nav_path.poses[0].pose.position.y;
            g_pose_end.pose.position.z = 0;
            double ang_roll, ang_pitch, ang_yaw;
            get_angle(car_foot_point, foot_point, ang_roll, ang_pitch, ang_yaw);
            g_pose_start.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
            global_path.poses.push_back(g_pose_start);
            g_pose_end.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
            global_path.poses.push_back(g_pose_end);
        } else {
            geometry_msgs::PoseStamped g_pose_start, g_pose_end;
            g_pose_start.pose.position.x = foot_point.first;
            g_pose_start.pose.position.y = foot_point.second;
            g_pose_start.pose.position.z = 0;
            g_pose_end.pose.position.x = cleaner_nav_path.poses[0].pose.position.x;
            g_pose_end.pose.position.y = cleaner_nav_path.poses[0].pose.position.y;
            g_pose_end.pose.position.z = 0;
            double ang_roll, ang_pitch, ang_yaw;
            get_angle(foot_point, nearst_road.position, ang_roll, ang_pitch, ang_yaw);
            g_pose_start.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
            global_path.poses.push_back(g_pose_start);
            g_pose_end.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
            global_path.poses.push_back(g_pose_end);
            g_pose_end.pose.position.x = foot_point.first;
            g_pose_end.pose.position.y = foot_point.second;
            global_path.poses.push_back(g_pose_end);
        }
    }

    if (debug) {
        std::cout << "global_node: " << std::endl;
        for (int i = 0; i < global_node_list.size(); i++) {
            std::cout << "global_node_list[i].node_name " << global_node_list[i].node_name << std::endl;
        }
    }

    layout_path_finish = true;
    do_index = 0;
    path_combine.header.frame_id = "map";
    path_combine.header.stamp = ros::Time::now();

    for (int i = 0; i < global_path.poses.size(); i++) {
        path_combine.poses.push_back(global_path.poses[i]);
    }
    pub_target_path.publish(path_combine);
}

void Global_path_node::charge_go_callback(const std_msgs::Header& msg) {
    charge_path_finish = false;
    charge_combine.poses.clear();

    nav_msgs::Path charge_path_msg;
    charge_path_msg.header.frame_id = "map";
    charge_path_msg.header.stamp = ros::Time::now();

    double min_dis = 9999999;
    std::pair<road_node, road_node> turn_seg;
    int index = -1;
    int out_index = -1;
    road_node nearst_road;
    std::vector<road_node> global_node_list;
    for (int i = 0; i < road_nodes.size(); i++) {
        if (road_nodes[i].node_name == "pre") {
            out_index = i;
        }
        double dis = point_dis(road_nodes[i].position, std::pair<double, double>(curr_robot_pose.position.x, curr_robot_pose.position.y));
        if (dis < min_dis) {
            min_dis = dis;
            index = i;
        }
    }
    std::cout << "out node: " << road_nodes[out_index].node_name << std::endl;
    std::cout << "nearst node: " << road_nodes[index].node_name << std::endl;
    bool find_path_trig = false;
    if ((out_index != index) && (road_nodes[index].node_name != "out") && (road_nodes[index].node_name != "in")) {
        find_path_trig = find_path(road_nodes[index], road_nodes[out_index], global_node_list);
    } else {
        find_path_trig = find_path(road_nodes[T4], road_nodes[out_index], global_node_list);
    }
    if (find_path_trig) {
        std::cout << "befor..." << std::endl;
        for (auto iter : global_node_list) {
            std::cout << "iter.node_name: " << iter.node_name << std::endl;
        }
        for (auto iter = global_node_list.begin(); iter != global_node_list.end(); iter++) {
            if (iter->node_name == "t3") {
                iter = iter + 1;
                *(iter) = road_nodes[T4];
                global_node_list.push_back(road_nodes[out]);
                break;

            } else if (iter->node_name == "t4") {
                iter = iter + 1;
                *(iter) = road_nodes[T3];
                global_node_list.push_back(road_nodes[out]);
                break;
            }
        }
        std::cout << "after..." << std::endl;
        for (auto iter : global_node_list) {
            std::cout << "iter.node_name: " << iter.node_name << std::endl;
        }
        // 添加四元数
        for (int i = 0; i < global_node_list.size() - 2; i++) {  // 0，1，（2，3）
            geometry_msgs::PoseStamped g_pose_start, g_pose_end, g_pose_pre;
            g_pose_start.pose.position.x = global_node_list[i].position.first;
            g_pose_start.pose.position.y = global_node_list[i].position.second;
            g_pose_start.pose.position.z = 0;

            g_pose_end.pose.position.x = global_node_list[i + 1].position.first;
            g_pose_end.pose.position.y = global_node_list[i + 1].position.second;
            g_pose_end.pose.position.z = 0;

            double ang_roll, ang_pitch, ang_yaw;
            get_angle(global_node_list[i].position, global_node_list[i + 1].position, ang_roll, ang_pitch, ang_yaw);
            g_pose_start.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
            charge_path_msg.poses.push_back(g_pose_start);

            if (i == (global_node_list.size() - 3)) {
                g_pose_end.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
                charge_path_msg.poses.push_back(g_pose_end);

                get_angle(global_node_list[i + 2].position, road_nodes[pre].position, ang_roll, ang_pitch, ang_yaw);
                g_pose_end.pose.position.x = global_node_list[i + 2].position.first;
                g_pose_end.pose.position.y = global_node_list[i + 2].position.second;
                g_pose_end.pose.position.z = 0;
                g_pose_end.pose.orientation = get_quaternion(ang_roll, ang_pitch, ang_yaw);
                charge_path_msg.poses.push_back(g_pose_end);

                g_pose_pre = g_pose_end;
                g_pose_pre.pose.position.x = road_nodes[pre].position.first;
                g_pose_pre.pose.position.y = road_nodes[pre].position.second;
                g_pose_pre.pose.position.z = 0;
                charge_path_msg;
                charge_path_msg.poses.push_back(g_pose_pre);
                charge_path_msg.poses.push_back(g_pose_end);
                g_pose_pre.pose.position.x = road_nodes[in].position.first;
                g_pose_pre.pose.position.y = road_nodes[in].position.second;
                charge_path_msg.poses.push_back(g_pose_pre);
            }
        }

        double a, b, c;
        get_line(global_node_list[0].position, global_node_list[1].position, a, b, c);
        std::pair<double, double> foot_point =
            GetFootOfPerpendicular(std::pair<double, double>(curr_robot_pose.position.x, curr_robot_pose.position.y), a, b, c);
        if (between_line(global_node_list[0].position, global_node_list[1].position, foot_point)) {
            std::cout << "int the line...." << std::endl;
            charge_path_msg.poses[0].pose.position.x = foot_point.first;
            charge_path_msg.poses[0].pose.position.y = foot_point.second;
        } else {
            std::cout << "not int the line...." << std::endl;
            geometry_msgs::PoseStamped start;
            start.pose.position.x = foot_point.first;
            start.pose.position.y = foot_point.second;
            start.pose.orientation = charge_path_msg.poses[0].pose.orientation;
            charge_path_msg.poses.insert(charge_path_msg.poses.begin(), start);
        }

    } else {
        std::cout << "find nearst node to out failed" << std::endl;
        return;
    }

    charge_path.publish(charge_path_msg);
    charge_combine = charge_path_msg;
    charge_combine.header.frame_id = "map";
    charge_combine.header.stamp = ros::Time::now();
    charge_path_finish = true;
    do_index = 0;
}

void Global_path_node::robot_pose_subCallback(const geometry_msgs::Pose& msg) {
    curr_robot_pose = msg;
    static bool arm_trig = false;

    // 全局清洗路径规划
    if ((layout_path_finish) && (!charge_path_finish)) {
        if ((do_index == 0) && !last_charge_trig) {  // 库外
            std::cout << "las_charge_trig: " << last_charge_trig << std::endl;
            dynamic_reconfigure::Client<move_base::MoveBaseConfig> client("/move_base");
            move_base::MoveBaseConfig cur_movebase_config;
            client.getCurrentConfiguration(cur_movebase_config, ros::Duration(0.5));
            client.setConfiguration(cur_movebase_config);  //???
            cur_movebase_config.base_global_planner = "navfn/NavfnROS";
            // cur_movebase_config.base_global_planner = "global_planner/MixedPlanner";
            // cur_movebase_config.base_global_planner = "global_planner/FixedGlobalPlanner";
            cur_movebase_config.base_local_planner = "teb_local_planner/TebLocalPlannerROS";
            // cur_movebase_config.base_local_planner = "bz_local_planner/BZPlannerROS";
            ros::param::set("/move_base/BZPlannerROS/max_vel_x", 0.35); 
            ros::param::set("/move_base/BZPlannerROS/x_offset_pos", 5.0);
            ros::param::set("/move_base/BZPlannerROS/x_offset_neg", 5.0);
            client.setConfiguration(cur_movebase_config);
            std::cout << "local_planner: teb_local_planner" << std::endl; 
  
            geometry_msgs::Pose target_pose = path_combine.poses[0].pose;
            std::cout << "do_index:" << do_index << std::endl;
            std::cout << "target_pose:" << target_pose << std::endl;
            pub_nav_pose.publish(target_pose);
            do_index = do_index + 1;

            std_msgs::Header charge_trig_msg;
            charge_trig_msg.frame_id = "stop_trig";
            charge_trig_msg.seq = 1;  // pick
            pub_charge_trig.publish(charge_trig_msg);
        } else if ((do_index == 0) && last_charge_trig) {  // 库内
            std::cout << "las_charge_trig: " << last_charge_trig << std::endl;
            dynamic_reconfigure::Client<move_base::MoveBaseConfig> client("/move_base");
            move_base::MoveBaseConfig cur_movebase_config;
            client.getCurrentConfiguration(cur_movebase_config, ros::Duration(0.5));
            client.setConfiguration(cur_movebase_config);  //???
            cur_movebase_config.base_global_planner = "navfn/NavfnROS";
            // cur_movebase_config.base_global_planner = "global_planner/MixedPlanner";
            // cur_movebase_config.base_global_planner = "global_planner/FixedGlobalPlanner";
            // cur_movebase_config.base_local_planner = "teb_local_planner/TebLocalPlannerROS";
            cur_movebase_config.base_local_planner = "bz_local_planner/BZPlannerROS";
            ros::param::set("/move_base/BZPlannerROS/max_vel_x", 0.1);
            ros::param::set("/move_base/BZPlannerROS/x_offset_pos", 2.0);
            ros::param::set("/move_base/BZPlannerROS/x_offset_neg", 2.0);
            client.setConfiguration(cur_movebase_config);
            std::cout << "local_planner: bz_local_planner" << std::endl;

            geometry_msgs::Pose target_pose = path_combine.poses[0].pose;
            std::cout << "do_index:" << do_index << std::endl;
            std::cout << "target_pose:" << target_pose << std::endl;
            pub_nav_pose.publish(target_pose);
            do_index = do_index + 1;
        }

        geometry_msgs::Pose target_pose = path_combine.poses[do_index - 1].pose;
        if ((do_index > 0) && (geometry_dis(curr_robot_pose, target_pose) < 0.5)) {
            if (last_charge_trig && (geometry_dis(curr_robot_pose, path_combine.poses[0].pose) > 5.0)) {  // 出库
                dynamic_reconfigure::Client<move_base::MoveBaseConfig> client("/move_base");
                move_base::MoveBaseConfig cur_movebase_config;
                client.getCurrentConfiguration(cur_movebase_config, ros::Duration(0.5));
                client.setConfiguration(cur_movebase_config);  //???
                cur_movebase_config.base_global_planner = "navfn/NavfnROS";
                // cur_movebase_config.base_global_planner = "global_planner/MixedPlanner";
                // cur_movebase_config.base_global_planner = "global_planner/FixedGlobalPlanner";
                cur_movebase_config.base_local_planner = "teb_local_planner/TebLocalPlannerROS";
                // cur_movebase_config.base_local_planner = "bz_local_planner/BZPlannerROS";
                ros::param::set("/move_base/BZPlannerROS/max_vel_x", 0.3);
                ros::param::set("/move_base/BZPlannerROS/x_offset_pos", 5.0);
                ros::param::set("/move_base/BZPlannerROS/x_offset_neg", 5.0);
                client.setConfiguration(cur_movebase_config);
                std::cout << "local_planner: teb_local_planner" << std::endl;

                std_msgs::Header charge_trig_msg;
                charge_trig_msg.frame_id = "stop_trig";
                charge_trig_msg.seq = 1;  // pick
                for (int i = 0; i < 3; i++) {
                    pub_charge_trig.publish(charge_trig_msg);
                    std::cout << "arm action....." << std::endl;
                    ros::Duration(0.1).sleep();
                }
                last_charge_trig = false;
            }

            geometry_msgs::Pose target_pose = path_combine.poses[do_index].pose;
            std::cout << "do_index:" << do_index << std::endl;
            std::cout << "target_pose:" << target_pose << std::endl;
            pub_nav_pose.publish(target_pose);

            do_index = do_index + 1;
            // last pose
            if (do_index == global_path.poses.size()) {
                do_index = -1;
                layout_path_finish = false;
                std_msgs::Header head;
                head.frame_id = "cleaner";
                pub_global_path_trig.publish(head);
            }
        }

        // 回库充电路径规划
    } else if ((!layout_path_finish) && (charge_path_finish)) {
        if (do_index == 0) {
            dynamic_reconfigure::Client<move_base::MoveBaseConfig> client("/move_base");
            move_base::MoveBaseConfig cur_movebase_config;
            client.getCurrentConfiguration(cur_movebase_config, ros::Duration(0.5));
            client.setConfiguration(cur_movebase_config);  //???
            cur_movebase_config.base_global_planner = "navfn/NavfnROS";
            // cur_movebase_config.base_global_planner = "global_planner/MixedPlanner";
            // cur_movebase_config.base_global_planner = "global_planner/FixedGlobalPlanner";
            cur_movebase_config.base_local_planner = "teb_local_planner/TebLocalPlannerROS";
            // cur_movebase_config.base_local_planner = "bz_local_planner/BZPlannerROS";
            client.setConfiguration(cur_movebase_config);
            std::cout << "local_planner: teb_local_planner" << std::endl;

            local_bz = false;

            geometry_msgs::Pose target_pose = charge_combine.poses[0].pose;
            pub_nav_pose.publish(target_pose);
            std::cout << "do_index size:" << charge_combine.poses.size() << std::endl;
            std::cout << "do_index:" << do_index << std::endl;
            do_index = do_index + 1;

            std_msgs::Header charge_trig_msg;
            charge_trig_msg.frame_id = "stop_trig";
            charge_trig_msg.seq = 1;  // pick
            pub_charge_trig.publish(charge_trig_msg);
        }

        if (do_index > 0) {
            if ((do_index == abs(3 - (int)(charge_combine.poses.size()))) && (!local_bz)) {
                printf("do_index: %d\n", do_index);
                printf("do_index size: %ld\n", charge_combine.poses.size());
                dynamic_reconfigure::Client<move_base::MoveBaseConfig> client("/move_base");
                move_base::MoveBaseConfig cur_movebase_config;
                client.getCurrentConfiguration(cur_movebase_config, ros::Duration(0.5));
                client.setConfiguration(cur_movebase_config);  //???
                cur_movebase_config.base_global_planner = "navfn/NavfnROS";
                // cur_movebase_config.base_global_planner = "global_planner/MixedPlanner";
                // cur_movebase_config.base_global_planner = "global_planner/FixedGlobalPlanner";
                // cur_movebase_config.base_local_planner = "teb_local_planner/TebLocalPlannerROS";
                cur_movebase_config.base_local_planner = "bz_local_planner/BZPlannerROS";
                ros::param::set("/move_base/BZPlannerROS/max_vel_x", 0.1);
                ros::param::set("/move_base/BZPlannerROS/x_offset_pos", 2);
                ros::param::set("/move_base/BZPlannerROS/x_offset_neg", 2);
                client.setConfiguration(cur_movebase_config);
                std::cout << "local_planner: bz_local_planner" << std::endl;
                local_bz = true;
            }
            // arm down

            geometry_msgs::Pose target_pose = charge_combine.poses[do_index - 1].pose;
            if ((do_index == abs(2 - (int)(charge_combine.poses.size()))) && (!arm_trig)) {
                std_msgs::Header charge_trig_msg;
                charge_trig_msg.frame_id = "stop_trig";
                charge_trig_msg.seq = 2;  // place
                for (int i = 0; i < 3; i++) {
                    pub_charge_trig.publish(charge_trig_msg);
                    std::cout << "arm action....." << std::endl;
                    ros::Duration(0.1).sleep();
                }
                arm_trig = true;
            }
            if ((geometry_dis(curr_robot_pose, target_pose) < 0.5)) {
                geometry_msgs::Pose target_pose = charge_combine.poses[do_index].pose;
                pub_nav_pose.publish(target_pose);
                std::cout << "do_index:" << do_index << std::endl;
                do_index = do_index + 1;
            }
            if (do_index == charge_combine.poses.size()) {
                do_index = -1;
                charge_path_finish = false;
                std_msgs::Header head;
                head.frame_id = "cleaner";
                pub_global_path_trig.publish(head);
                arm_trig = false;
            }
        }
    }
}

void Global_path_node::init_data() {
    // dynamic_reconfigure::Server<layout_nav_interface::map_offsetConfig>* configServer;
    // configServer = new dynamic_reconfigure::Server<layout_nav_interface::map_offsetConfig>(ros::NodeHandle("~"));
    // dynamic_reconfigure::Server<layout_nav_interface::map_offsetConfig>::CallbackType cb;
    // cb = boost::bind(&configCb, _1, _2);
    // configServer->setCallback(cb);
}

}  // namespace global_path_node