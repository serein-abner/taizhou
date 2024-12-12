#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "cmd_test");
    ros::NodeHandle nh;

    geometry_msgs::Twist cmd_vel;
    ros::Publisher cmdPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    while(ros::ok()){
        double x = ((rand() % 10) / (double)50)-0.05;

        cmd_vel.linear.x = -0.1;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0.1+x;

        cmdPub.publish(cmd_vel);

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
}