#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_connection_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher return_pub = nh.advertise<std_msgs::String>("/phoneControlerRecive", 1, true);
    while(ros::ok()){
        std_msgs::String srrr;
        return_pub.publish(srrr);
        ros::Duration(0.1).sleep();
        break;
    }
    return 0;
}