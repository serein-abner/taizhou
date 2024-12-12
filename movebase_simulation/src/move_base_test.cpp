#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <pluginlib/class_loader.hpp>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>

#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "move_base_test");
    ros::NodeHandle simple_nh("move_base");
    ros::NodeHandle nh;

    ros::Publisher action_goal_pub_ = simple_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1, true);

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

    move_base_msgs::MoveBaseActionGoal goal;
    // goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.goal.target_pose.header.seq = 1;
    goal.goal.target_pose.header.frame_id = "map";
    goal.goal.target_pose.header.stamp = ros::Time::now();
    goal.goal.target_pose.pose.position.x = 5;
    goal.goal.target_pose.pose.position.y = 2;
    goal.goal.target_pose.pose.position.z = 0;
    goal.goal.target_pose.pose.orientation.w = 1;
    goal.goal.target_pose.pose.orientation.x = 0;
    goal.goal.target_pose.pose.orientation.y = 0;
    goal.goal.target_pose.pose.orientation.z = 0;

    // ROS_INFO_STREAM(goal);

    // action_goal_pub_.publish(goal);
    // ros::Duration(1.0).sleep();

    ros::AsyncSpinner spinner(2);
    spinner.start();

    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader("nav_core", "nav_core::BaseLocalPlanner");
    //
    // auto cls = blp_loader.getDeclaredClasses();
    // for (int i = 0; i < cls.size();i++){
    //     ROS_INFO_STREAM(cls[i]);
    // }
    std::string local_planner = "teb_local_planner/TebLocalPlannerROS";
    tf2_ros::Buffer buffer(ros::Duration(0));
    tf2_ros::TransformListener tf(buffer);
    costmap_2d::Costmap2DROS* controller_costmap_ros_;
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", buffer);
    auto tc_ = blp_loader.createInstance(local_planner);
    tc_->initialize(blp_loader.getName(local_planner), &buffer, controller_costmap_ros_);

    std::vector<geometry_msgs::PoseStamped> path;
    for (int i = 0; i < 100; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = i * 0.1;
        pose.pose.position.y = sin(i * 0.05);
        pose.pose.position.z = 0;
        pose.pose.orientation.w = 1;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        path.push_back(pose);
    }

    ROS_INFO("start planning");

    geometry_msgs::Twist cmd;
    bool if_suc = tc_->setPlan(path);
    if (if_suc){
        while(!tc_->isGoalReached() && ros::ok())
        {
            if (tc_->computeVelocityCommands(cmd))
            {
                cmd_pub.publish(cmd);
            }
            else
            {
                ROS_ERROR("cannot find local path");
            }
            ros::Duration(0.1).sleep();
        }
    }
    else{
        ROS_ERROR("cannot find global path");
    }

    ros::waitForShutdown();
}