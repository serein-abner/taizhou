#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/eigen.h>

double x = 8, y = 8, angle_z = 0;
double cx = 0, cy = 0, cangle_z = 0;
double v_x = 0, v_y = 0;
double w_z = 0;
double delta_t = 0.1;
pcl::PointCloud<pcl::PointXYZ> cloud;
float octreeResolution_arg = 128.0f;
int search_K = 100;
bool got_zval = false;
bool if_update = true;

double PI = 3.1415926;

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

void ground_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    if (got_zval == false)
    {
        pcl::fromROSMsg(*cloud_msg, cloud);
        got_zval = true;
    }
}

Eigen::Quaternionf normalToQuaternion(const Eigen::Vector3f &normal)
{
    // Step 1: Create Axis-Angle Representation
    Eigen::Vector3f axis = normal.normalized(); // Normalize the normal vector
    float angle = acos(axis[2]);                // Angle between normal vector and [0, 0, 1]

    // Step 2: Convert Axis-Angle to Quaternion
    Eigen::Quaternionf quaternion;
    quaternion = Eigen::AngleAxisf(angle / 2.0f, Eigen::Vector3f::UnitX()); // Rotation around X-axis
    quaternion.normalize();                                                 // Ensure quaternion is normalized

    return quaternion;
}

Eigen::Vector3f compute_depth(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                pcl::PointXYZ &point)
{
    std::vector<int> temp_indices;
    std::vector<float> temp_distance;
    if (octree.nearestKSearch(point, search_K, temp_indices, temp_distance))
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices());
        extract.setInputCloud(cloud);
        indices_ptr->indices = temp_indices;
        extract.setIndices(indices_ptr);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*temp_cloud);
        //
        //Step 2: Compute the Covariance Matrix
        Eigen::Matrix3f covariance_matrix;
        pcl::computeCovarianceMatrix(*temp_cloud, covariance_matrix);
        // Step 3: Compute Eigenvalues and Eigenvectors
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_matrix);
        Eigen::Vector3f eigenvalues = solver.eigenvalues();
        Eigen::Matrix3f eigenvectors = solver.eigenvectors();
        // Step 4: Extract Principal Axis (Orientation)
        Eigen::Vector3f principal_axis = eigenvectors.col(0).normalized();
        if (principal_axis(2) < 0)
        {
            principal_axis[2] = -principal_axis[2];
        }
        // Step 5: Convert Principal Axis to Quaternion
        Eigen::Quaternionf orientation;
        orientation.setFromTwoVectors(Eigen::Vector3f::UnitZ(), principal_axis);
        Eigen::Vector3f rpy = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        // ROS_INFO_STREAM(principal_axis.transpose());
        // ROS_INFO_STREAM(rpy.transpose());
        if (std::abs(rpy[0]) > 1)
        {
            if (rpy[0]>0)
            {
                rpy[0] = rpy[0] - PI;
            }
            else
            {
                rpy[0] = rpy[0] + PI;
            }    
        }
        //
        if (std::abs(rpy[1]) > 1)
        {
            if (rpy[1]>0)
            {
                rpy[1] = rpy[1] - PI;
            }
            else
            {
                rpy[1] = rpy[1] + PI;
            }    
        }
        //
        float z_sum = 0.0;
        int temp_count = 0;
        for (size_t j = 0; j < temp_cloud->size(); j++)
        {
            z_sum += temp_cloud->points[j].z;
            temp_count++;
        }
        point.z = z_sum / temp_count;
        //
        return rpy;
    }
    else
    {
        ROS_INFO_STREAM("cannot find point");
        Eigen::Vector3f rpy{0, 0, 0};
        return rpy;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ugv");
    ros::NodeHandle nh;

    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &callback);
    ros::Subscriber ground_cloud_sub = nh.subscribe("/ground_cloud", 1, ground_callback);

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    // wait for ground point ready
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    while (ros::ok())
    {
        if (got_zval == 1)
        {
            *ground_cloud = cloud;
            break;
        }
        else
        {
            ROS_INFO_STREAM("wait for ground points ready");
            ros::Duration(2.0).sleep();
            ros::spinOnce();
        }
    }
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> ground_octree(octreeResolution_arg);
    ground_octree.setInputCloud(ground_cloud);
    ground_octree.addPointsFromInputCloud();

    //
    pcl::PointXYZ point;
    while (ros::ok())
    {
        lock.try_lock();
        if (if_update)
        {
            x += v_x * delta_t * cos(angle_z + delta_t * w_z);
            y += v_y * delta_t * sin(angle_z + delta_t * w_z);
            //
            point.x = x;
            point.y = y;
            point.z = 0;
            //
            angle_z += delta_t * w_z;
            cx = x;
            cy = y;
            cangle_z = angle_z;
        }
        if_update = false;
        lock.unlock();
        //
        Eigen::Vector3f rpy = compute_depth(ground_octree, ground_cloud, point);
        transform.setOrigin(tf::Vector3(cx, cy, point.z));
        Eigen::Matrix3d rotation_matrix;
        // ROS_INFO_STREAM(rpy.transpose());
        rotation_matrix = Eigen::AngleAxisd(-rpy[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(angle_z, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quat(rotation_matrix);
        q.setW(quat.w());
        q.setX(quat.x());
        q.setY(quat.y());
        q.setZ(quat.z());

        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
        ros::Duration(0.08).sleep();
        ros::spinOnce();
    }
    ros::waitForShutdown();
}