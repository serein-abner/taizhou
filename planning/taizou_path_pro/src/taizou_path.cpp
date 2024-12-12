#include <ros/ros.h>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Geometry>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "dijkstra.h"

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::vector<std::vector<pcl::PointXYZ>> file_data; // 全局的路径点,在yaml里面
vector<pcl::PointXYZ> path2show, path2show1, path2show2;

pcl::PointXYZ self_pt = pcl::PointXYZ(0, 0, 0);
pcl::PointXYZ goal_pt = pcl::PointXYZ(0, 0, 0);

int dij_on = 0;
int ifmove = 0;
bool if_visual = true;
int st_idx, en_idx;
int got_st = 0;
int tem_idx1, tem_idx2; // 路径点两个方向对应的序号
pcl::PointXYZ en_pt;
int robot_idx, goal_idx;

// 自定义函数：从两个向量创建四元数
Eigen::Quaterniond quaternionFromTwoVectors(const Eigen::Vector3d &to, const Eigen::Vector3d &from = Eigen::Vector3d(1.0, 0.0, 0.0))
{
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d axis = from.cross(to);
    double angle = std::acos(from.dot(to) / (from.norm() * to.norm()));
    quaternion = Eigen::AngleAxisd(angle, axis.normalized());
    return quaternion;
}

//
vector<int> calculate_idx(int row_id, int col_id) // 获得同一个路径点在graph中不同方向的两个序号
{
    int sta_num = 0;
    for (int i = 0; i < row_id; i++)
    {
        sta_num = sta_num + file_data[i].size(); // 计算graph中起始的序号
    }
    int cal_idx1 = sta_num * 2 + col_id;
    int cal_idx2 = sta_num * 2 + file_data[row_id].size() + col_id;
    vector<int> out_vector;
    out_vector.push_back(cal_idx1);
    out_vector.push_back(cal_idx2);
    cout << "cal_idx1 = " << cal_idx1 << endl;
    cout << "cal_idx2 = " << cal_idx2 << endl;
    return out_vector; // 返回的是同一个路径点在graph中不同方向的两个序号
}

// 计算两个 pcl::PointXYZ 向量的点积
double dotProduct(const pcl::PointXYZ &v1, const pcl::PointXYZ &v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ROS_INFO("Received 2D pose estimate: x=%f, y=%f, theta=%f",
             msg->pose.position.x, msg->pose.position.y, yaw * 180.0 / M_PI);
    // if (got_st == 0)
    if (1)
    {
        en_pt.x = msg->pose.position.x;
        en_pt.y = msg->pose.position.y;
        en_pt.z = msg->pose.position.z;
    }
    if (1)
    {
        double distance = 10000;
        int row_id, col_id;
        for (int i = 0; i < file_data.size(); i++) // 此处寻找最近的路径点
        {
            for (int j = 0; j < file_data[i].size(); j++)
            {
                pcl::PointXYZ tem_pt = file_data[i][j];
                double tem_dist = sqrt((en_pt.x - tem_pt.x) * (en_pt.x - tem_pt.x) + (en_pt.y - tem_pt.y) * (en_pt.y - tem_pt.y));

                if (tem_dist < distance)
                {
                    row_id = i;
                    col_id = j;
                    distance = tem_dist;
                }
            }
        }
        int sta_num = 0;
        for (int i = 0; i < row_id; i++) // 找到起始的序号
        {
            sta_num = sta_num + file_data[i].size();
        }
        tem_idx1 = sta_num * 2 + col_id;
        tem_idx2 = sta_num * 2 + file_data[row_id].size() + col_id; // 路径点两个方向对应的序号
        dij_on = 1;                                                 // 在main的主循环中开始寻找全局最短路径
    }
}

/*** 
 * @description: 对给定的路径进行插值，使路径上相邻点之间的距离不超过一定的阈值
 * @param {Path} path_msg
 * @return {*}
 */
nav_msgs::Path path_interpolation(nav_msgs::Path path_msg)
{
    nav_msgs::Path new_path;
    new_path.header.frame_id = "map";
    new_path.header.stamp = ros::Time::now();
    for (size_t i = 0; i < path_msg.poses.size() - 1; i++)
    {
        auto prev_pt = path_msg.poses[i].pose.position;
        auto next_pt = path_msg.poses[i + 1].pose.position;
        double dis = sqrt(pow(prev_pt.x - next_pt.x, 2) + pow(prev_pt.y - next_pt.y, 2));
        if (dis > 1)
        {
            int idx = int(dis / 0.2);
            double vect_x = (-prev_pt.x + next_pt.x) / idx;
            double vect_y = (-prev_pt.y + next_pt.y) / idx;
            for (size_t j = 0; j < idx; j++)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = prev_pt.x + j * vect_x;
                pose.pose.position.y = prev_pt.y + j * vect_y;
                pose.pose.orientation.w = 1;
                new_path.poses.push_back(pose);
            }
        }
    }
    return new_path;
}

/*** 
 * @description: 为路径中的每个点设置方向（姿态），使得路径中的点能够正确地表示其朝向
 * @param {Path} &path
 * @return {*}
 */
void setPathOrientations(nav_msgs::Path &path)
{
    if (path.poses.size() < 2)
    {
        return; // 如果路径中少于两个点，则不需要计算方向
    }

    for (size_t i = 0; i < path.poses.size() - 1; ++i)
    {
        auto &current_pose = path.poses[i].pose;
        auto &next_pose = path.poses[i + 1].pose;
        Eigen::Vector3d direction(
            next_pose.position.x - current_pose.position.x,
            next_pose.position.y - current_pose.position.y,
            0); // 在平面上，我们不关心z方向的变化

        // 避免零向量的情况
        if (direction.norm() == 0)
        {
            continue;
        }

        Eigen::Quaterniond quaternion = quaternionFromTwoVectors(direction.normalized());//归一化方向向量

        tf::Quaternion tf_quat(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
        tf::quaternionTFToMsg(tf_quat, current_pose.orientation);
    }

    // 将最后一个点的朝向设置为倒数第二个点的朝向
    path.poses.back().pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
}

/*** 
 * @description: 一个点云路径（std::vector<pcl::PointXYZ>）转换为ROS中的路径消息（nav_msgs::Path），对路径进行插值和方向设置，
 * 使得路径在地图坐标系中更加平滑和完整
 * @param {vector<pcl::PointXYZ>} path_pts
 * @param {StampedTransform} transform
 * @return {*}
 */
nav_msgs::Path process_path(std::vector<pcl::PointXYZ> path_pts, tf::StampedTransform transform)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = transform.getOrigin().getX();
    pose.pose.position.y = transform.getOrigin().getY();
    pose.pose.position.z = 0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
    for (size_t j = 0; j < path_pts.size(); j++)
    {
        pose.pose.position.x = path2show[j].x;
        pose.pose.position.y = path2show[j].y;
        pose.pose.position.z = 0;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }
    nav_msgs::Path new_path_msg = path_interpolation(path_msg);
    setPathOrientations(new_path_msg);
    return new_path_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 10, true);
    ros::Publisher pub_point = nh.advertise<visualization_msgs::Marker>("points", 10);
    ros::Publisher pub_arrow = nh.advertise<visualization_msgs::Marker>("arrow", 10);
    ros::Publisher robot_position = nh.advertise<visualization_msgs::Marker>("robot_position", 10);
    ros::Publisher robot_goal = nh.advertise<visualization_msgs::Marker>("robot_goal", 10);
    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 1, poseCallback);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // load path file
    int end_pt = -1;
    std::string config_filename;
    std::vector<pcl::PointXYZ> point_in_line;
    std::vector<std::vector<pcl::PointXYZ>> path_pts;
    nh_private.getParam("path_file", config_filename);
    YAML::Node config = YAML::LoadFile(config_filename);
    //从一个YAML配置文件中读取一些位置信息，解析这些信息并存储在一个点云路径（path_pts）中
    for (const auto &box : config)
    {
        std::string name = box.first.as<std::string>();
        YAML::Node subnode = config[name];
        double pos_x = box.second["position"]["x"].as<double>();
        double pos_y = box.second["position"]["y"].as<double>();
        double pos_z = box.second["position"]["z"].as<double>();
        point_in_line.push_back(pcl::PointXYZ(pos_x, pos_y, pos_z));
        // determine wether entering a new node
        if (box.second["type"])
        {
            // get the first node of the line
            if (end_pt == -1)
            {
                end_pt = 0;
            }
            // get the end node of the line
            else if (end_pt == 0)
            {
                // save the whole line node
                path_pts.push_back(point_in_line);
                point_in_line.clear();
                end_pt = -1;
            }
        }
    }
    file_data = path_pts;

    // construct graph
    dijkstra path(path_pts);

    // visualisation of graph map
    /*** 
     * @description: 二维数组file_data中的每个点（包含x, y, z坐标）转换为geometry_msgs::Point对象
     * @return {*}
     */    
    int temp_count = 0;
    std::vector<geometry_msgs::Point> vis_points;
    for (int i = 0; i < file_data.size(); i++)
    {
        for (int j = 0; j < file_data[i].size(); j++)
        {
            vis_points.push_back(geometry_msgs::Point());
            vis_points[temp_count].x = file_data[i][j].x;
            vis_points[temp_count].y = file_data[i][j].y;
            vis_points[temp_count].z = file_data[i][j].z;
            temp_count++;
        }
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points = vis_points;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.75;
    marker.scale.y = marker.scale.x;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.750;
    marker.color.b = 0.0;
    pub_point.publish(marker);

    // planning loop
    int path_pt_count = 0;
    int path_pt_count2 = 0;
    tf::TransformListener listener;
    tf::Vector3 robot_direction(1.0, 0.0, 0.0); // 初始化一个单位向量
    pcl::PointXYZ temp_robot_dir;

    while (ros::ok())
    {
        tf::StampedTransform transform;
        if (dij_on)
        {
            //在一个循环中不断尝试从TF树（Transform Tree）中获取从"map"坐标系到"base_link"坐标系的变换
            while (ros::ok())
            {
                try
                {
                    listener.lookupTransform("map", "base_link", ros::Time(0), transform);
                    break;
                }
                catch (tf::TransformException ex)
                {
                    ROS_WARN("TF-Tree error, waiting for TF input.");
                }
                ros::Duration(0.1).sleep();
            }

            // planning  功能是获取机器人的当前位置和朝向，并在平面上表示出来
            //获取机器人位置
            pcl::PointXYZ robot_p = pcl::PointXYZ(transform.getOrigin().getX(), transform.getOrigin().getY(), 0);
            //获取机器人朝向的四元数
            tf::Quaternion rotation = transform.getRotation();
            // 使用四元数来获取机器人的朝向向量
            tf::Matrix3x3 rotation_matrix(rotation);
            //定义初始朝向向量
            tf::Vector3 temp_dir = tf::Vector3(1.0, 0.0, 0.0);
            robot_direction = rotation_matrix * temp_dir;
            // 将 z 分量设置为 0，以得到 xy 平面上的朝向向量
            double distance = 10000;
            int row_id, col_id;
            robot_direction.setZ(0);
            temp_robot_dir = pcl::PointXYZ(robot_direction.getX(), robot_direction.getY(), robot_direction.getZ());
            //
            for (int i = 0; i < file_data.size(); i++)
            {
                for (int j = 0; j < file_data[i].size(); j++)
                {
                    pcl::PointXYZ tem_pt = file_data[i][j];
                    // 计算从 point1 到 point2 的向量
                    pcl::PointXYZ temp_vector;
                    temp_vector.x = tem_pt.x - robot_p.x;
                    temp_vector.y = tem_pt.y - robot_p.y;
                    temp_vector.z = tem_pt.z - robot_p.z;
                    // 计算两个向量的点积
                    double product = dotProduct(temp_vector, temp_robot_dir);
                    // 与机器人同向的目标点才考虑，迭代搜索最近的路径点
                    if (product > 0)
                    {
                        double tem_dist = sqrt((robot_p.x - tem_pt.x) * (robot_p.x - tem_pt.x) + (robot_p.y - tem_pt.y) * (robot_p.y - tem_pt.y));
                        if (tem_dist < distance)
                        {
                            row_id = i;
                            col_id = j;
                            distance = tem_dist;
                        }
                    }
                }
            }
            // distance预设是10000，发生改变代表找得到附近路径点的意思
            //在确定了机器人最近的路径点之后，计算该路径点在图中的序号，并根据路径点在走道中的位置和朝向选择合适的序号
            if (distance != 10000)
            {
                int temp_stidx = 0;
                for (int i = 0; i < row_id; i++)
                {
                    temp_stidx = temp_stidx + file_data[i].size(); // 计算所在走道在graph中的起始序号
                }
                if ((col_id == 0) || (col_id == (file_data[row_id].size() - 1))) // 如果是在走道的两个端点
                {
                    
                    st_idx = calculate_idx(row_id, col_id)[0];
                }
                    
                else
                {
                    pcl::PointXYZ nex_pt = file_data[row_id][col_id + 1]; // 寻找道上的下一个路径点
                    pcl::PointXYZ temp_vector;
                    temp_vector.x = file_data[row_id][col_id + 1].x - file_data[row_id][col_id].x;
                    temp_vector.y = file_data[row_id][col_id + 1].y - file_data[row_id][col_id].y;
                    temp_vector.z = 0;
                    double product2 = dotProduct(temp_vector, temp_robot_dir);
                    // 或许没有必要，file_data[row_id][col_id]已经是在机器人前进方向上的了同向的了
                    if (product2 > 0)
                    {
                        vector<int> temp1 = calculate_idx(row_id, col_id);
                        st_idx = temp1[0];
                    }
                    else
                    {
                        vector<int> temp1 = calculate_idx(row_id, col_id);
                        st_idx = temp1[1];
                    }
                }
            }
            else
            {
                ROS_WARN("find start station fail!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }
            // search path from two direction, choose the shortest one.
            path_pt_count2 = 0;
            path_pt_count = 0;
            // set start and end from one direction
            path.find(st_idx, tem_idx1);
            path2show1 = path.get_path_points();

            std::cout << "(path2show1.size()) = " << (path2show1.size()) << std::endl;

            // set start and end from another direction
            path.find(st_idx, tem_idx2);
            path2show2 = path.get_path_points();
            std::cout << "(path2show2.size()) = " << (path2show2.size()) << std::endl;
            // case 1 
            if (path2show1.size() == 0 && path2show2.size() != 0)
            {
                path2show = path2show2;
            }
            // case 2
            if (path2show2.size() == 0 && path2show1.size() != 0)
            {
                path2show = path2show1;
            }
            // case 3
            if (path2show2.size() != 0 && path2show1.size() != 0)
            {
                // compare the path length
                if ((path2show1.size() > path2show2.size()) && (path2show2.size() != 0))
                {
                    en_idx = tem_idx2;
                    path2show = path2show2;
                }
                else
                {
                    en_idx = tem_idx1;
                    path2show = path2show1;
                }
            }
            // case 4
            if (path2show2.size() == 0 && path2show1.size() == 0)
            {
                ROS_ERROR("cannot find path, both paths are empty!!!!");
            }

            // 代表最短路径搜索成功了
            if (path2show.size() >= 2)
            {
                vector<pcl::PointXYZ> temp_path2show = path2show;
                // 把目标点存到最后一个
                temp_path2show.push_back(en_pt);
                int temp_size = temp_path2show.size();
                // 取出倒数的三个点
                pcl::PointXYZ temp_pt1 = temp_path2show[temp_size - 1];
                pcl::PointXYZ temp_pt2 = temp_path2show[temp_size - 2];
                pcl::PointXYZ temp_pt3 = temp_path2show[temp_size - 3];
                pcl::PointXYZ vector1;
                vector1.x = temp_pt1.x - temp_pt2.x;
                vector1.y = temp_pt1.y - temp_pt2.y;
                vector1.z = temp_pt1.z - temp_pt2.z;
                pcl::PointXYZ vector2;
                vector2.x = temp_pt2.x - temp_pt3.x;
                vector2.y = temp_pt2.y - temp_pt3.y;
                vector2.z = temp_pt2.z - temp_pt3.z;
                // 如果符合，说明，最后的路径点已经越过目标点了，需要去除一个路径点
                double product3 = dotProduct(vector1, vector2);
                if (product3 < 0)
                {
                    path2show.pop_back();
                    path2show.push_back(en_pt);
                }
                else
                {
                    path2show.push_back(en_pt);
                }
            }
            // send path
            nav_msgs::Path path_msg = process_path(path2show, transform);
            pub_path.publish(path_msg);
            // reset
            dij_on = 0;
        }
        //
        ros::Duration(0.1).sleep();
    }
    ros::waitForShutdown();
}