#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "return_home");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Publisher returnhome_pub = nh.advertise<std_msgs::String>("return_home", 1, true);//

    ros::Duration(0.5).sleep();
    std_msgs::String st;
    returnhome_pub.publish(st);

    ros::shutdown();
}