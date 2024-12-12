#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <ros/package.h>

ros::Publisher phone_pub;

void startCallBack(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Message arrived: Topic:/phoneControler/cruise; Paylod:{ %s }", msg->data.c_str());

    const char *launch = "roslaunch ";

    std::string data = msg->data.c_str();
    std::string launchCmd;

    std_msgs::String msg1;

    msg1.data = "MainNodeRecived Message";
    phone_pub.publish(msg1);
    ROS_INFO("CallBackMessage has beed sent");

    launchCmd = msg->data.c_str();

    std::string cmd = launch + launchCmd;
    ROS_INFO("cmd: %s ", cmd.c_str());
    if (system(cmd.c_str()) == 0)
    {
        ROS_INFO("launch started");
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "order_cruise");
    ROS_INFO("Node:order_cruise,started");
    ros::NodeHandle h;
    ros::Subscriber startListener = h.subscribe<std_msgs::String>("/phoneControler/cruise", 10, startCallBack);
    phone_pub = h.advertise<std_msgs::String>("/phoneControlerRecive", 10);
    ros::spin();
    return 0;
}
