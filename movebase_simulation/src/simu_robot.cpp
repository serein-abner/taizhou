#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>

double x = 6, y = 9.71, angle_z = 0;
double rx = 0, ry = 0, rz = 0, rw = 1;
double cx = 0, cy = 0, cangle_z = 0;
double v_x = 0, v_y = 0;
double w_z = 0;
double delta_t = 0.1;
bool if_update = true;
bool if_locate = false;

std::mutex lock;

void callback(const geometry_msgs::Twist::ConstPtr &msg){
    lock.try_lock();
    v_x = msg->linear.x;
    v_y = msg->linear.x;
    w_z = msg->angular.z;
    if_update = true;
    lock.unlock();
    ros::Duration(0.025).sleep();
}

void posecallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    lock.try_lock();
    x = pose->pose.pose.position.x;
    y = pose->pose.pose.position.y;
    rx = pose->pose.pose.orientation.x;
    ry = pose->pose.pose.orientation.y;
    rz = pose->pose.pose.orientation.z;
    rw = pose->pose.pose.orientation.w;
    if_locate = true;
    lock.unlock();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ugv");
    ros::NodeHandle nh;
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &callback);
    ros::Subscriber initial_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &posecallback);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    tf::TransformBroadcaster br;
    while (ros::ok())
    {
        tf::Transform transform;
        tf::Quaternion q;
        //
        if (if_update && !if_locate)
        {
            //
            lock.try_lock();
            x += v_x * delta_t * cos(angle_z + delta_t * w_z);
            y += v_y * delta_t * sin(angle_z + delta_t * w_z);
            angle_z += delta_t * w_z;
            cx = x;
            cy = y;
            cangle_z = angle_z;
            if_update = false;
            lock.unlock();
            //
            transform.setOrigin(tf::Vector3(cx, cy, 0.0));
            q.setRPY(0, 0, angle_z);
            transform.setRotation(q);
        }
        //
        if (if_locate)
        {
            lock.try_lock();
            transform.setOrigin(tf::Vector3(x, y, 0.0));
            q.setX(rx);
            q.setY(ry);
            q.setZ(rz);
            q.setW(rw);
            angle_z = q.getAngle();
            transform.setRotation(q);
            if_locate = false;
            lock.unlock();
        }
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
        ros::Duration(0.08).sleep();
    }
    ros::waitForShutdown();
}