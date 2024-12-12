#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

using namespace std;

pcl::PassThrough<pcl::PointXYZ>::Ptr pass_filter;
ros::Publisher filted_lidarPub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin;
sensor_msgs::PointCloud2 msgout;

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    pcl::fromROSMsg(*msg, *cloudin);
    pass_filter->setInputCloud(cloudin);
    pass_filter->filter(*cloudin);
    //
    pcl::toROSMsg(*cloudin, msgout);
    msgout.header.frame_id = "base_link";
    msgout.header.stamp = msg->header.stamp;
    filted_lidarPub.publish(msgout);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_filter_node");
    ros::NodeHandle nh;

    pass_filter.reset(new pcl::PassThrough<pcl::PointXYZ>);
    cloudin.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter->setFilterFieldName("z");
    pass_filter->setFilterLimits(0, 1.5);
    

    ros::AsyncSpinner spinner(4);
    spinner.start();

    filted_lidarPub= nh.advertise<sensor_msgs::PointCloud2>("obstacle_points", 1);
    ros::Subscriber lidarSub = nh.subscribe<sensor_msgs::PointCloud2>("rslidar_points", 1, callback);

    ros::waitForShutdown();
}