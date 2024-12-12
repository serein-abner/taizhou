#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher localCloudsub;
std::string world_link;

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // Convert ROS point cloud to PCL point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_NaN_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);


  for (size_t i = 0; i < cloud->size(); i++)
  {
    if (isnan(cloud->points[i].x) == false)
    {
      cloud_NaN_filtered->push_back(cloud->points[i]);
    }
  }

  // Create the StatisticalOutlierRemoval object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(cloud_NaN_filtered);

  // Set the parameters for the statistical analysis
  sor.setMeanK(25);            // The number of neighbors to analyze for each point
  sor.setStddevMulThresh(1.0); // The standard deviation multiplier threshold

  // Apply the filter
  sor.filter(*cloud_NaN_filtered);

  sensor_msgs::PointCloud2 localMSG;
  pcl::toROSMsg(*cloud_NaN_filtered, localMSG);
  localMSG.header.frame_id = "base_link";
  localMSG.header.stamp = ros::Time::now();
  localCloudsub.publish(localMSG);
}

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "lidar_filter");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Subscribe to point cloud topic
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/wanji_point", 1, lidar_callback);
  localCloudsub = nh.advertise<sensor_msgs::PointCloud2>("/filted_points", 1);

  //
  ros::waitForShutdown();
  return 0;
}
