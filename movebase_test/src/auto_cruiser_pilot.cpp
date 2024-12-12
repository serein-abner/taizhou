#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <math.h>

bool if_returnhome = false;

void read_config_file()
{
}

void callback(const std_msgs::StringConstPtr &string)
{
  if_returnhome = true;
  ROS_INFO_STREAM("recieved returnhome command");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base_client");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);
  ros::Publisher return_pub = nh.advertise<std_msgs::String>("/phoneControlerRecive", 1, true);
  ros::Subscriber returnhome_sub = nh.subscribe<std_msgs::String>("return_home", 1, callback);

  // Create a TransformListener object to listen to TFs
  std::vector<std::vector<double>> pos_list, alt_list;
  std::vector<double> type_list;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string config_filename;
  nh_private.getParam("path_file", config_filename);
  YAML::Node config = YAML::LoadFile(config_filename);

  for (const auto &box : config)
  {
    std::string name = box.first.as<std::string>();
    YAML::Node subnode = config[name];
    //
    double pos_x = box.second["pos"][0].as<double>();
    double pos_y = box.second["pos"][1].as<double>();
    double pos_z = box.second["pos"][2].as<double>();
    pos_list.push_back(std::vector<double>{pos_x, pos_y, pos_z});
    //
    double pose_x = box.second["alt"][0].as<double>();
    double pose_y = box.second["alt"][1].as<double>();
    double pose_z = box.second["alt"][2].as<double>();
    double pose_w = box.second["alt"][3].as<double>();
    alt_list.push_back(std::vector<double>{pose_x, pose_y, pose_z, pose_w});
    //
    double tt = box.second["type"].as<double>();
    type_list.push_back(tt);
  }

  // create a series of goals from YAML files
  std::vector<geometry_msgs::PoseStamped> goals;
  for (int i = 0; i < pos_list.size(); i++)
  {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = pos_list[i][0];
    goal.pose.position.y = pos_list[i][1];
    goal.pose.orientation.z = alt_list[i][2];
    goal.pose.orientation.w = alt_list[i][3];
    goals.push_back(goal);
  }


  // 发送一系列目标位置
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  // planning loop
  while (ros::ok())
  {
    for (size_t i = 0; i < goals.size(); i++)
    {
      goal_pub.publish(goals[i]);
      //
      while (ros::ok())
      {
        //
        try
        {
          // Look up the transform from "map" to "base_link" (latest available transform)
          geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
          // Extract position and orientation information from the transform
          geometry_msgs::Vector3 position = transformStamped.transform.translation;
          geometry_msgs::Quaternion orientation = transformStamped.transform.rotation;
          //
          double dis = sqrt(pow(position.x - goals[i].pose.position.x, 2) +
                            pow(position.y - goals[i].pose.position.y, 2));
          double dis_pose = sqrt(pow(orientation.z - goals[i].pose.orientation.z, 2) +
                                 pow(orientation.w - goals[i].pose.orientation.w, 2));
          // type 0 is path-pass point
          if (type_list[i] == 2)
          {
            if (dis <= 3)
            {
              break;
            }
          }
          // type 0 is end point
          if (type_list[i] == 1)
          {
            if (dis <= 1 && dis_pose < 0.5)
            {
              break;
            }
          }
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("%s", ex.what()); // If there's an exception, print the error message
        }
        //
        ros::Duration(0.1).sleep();
      }
    }
    //
    if (if_returnhome)
    {
      std_msgs::String srrr;
      return_pub.publish(srrr);
      ros::Duration(0.1).sleep();
      break;
      ROS_INFO_STREAM("task finished");
    }
  //
  }
  ros::shutdown();
}