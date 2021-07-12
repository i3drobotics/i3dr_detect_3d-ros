#include <ros/ros.h>
#include <iostream>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB PointT;

double grid_size_ = 0.0;
ros::Publisher point_cloud_pub_;

void point_cloud_reduce_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (grid_size_, grid_size_, grid_size_);
    sor.filter (*cloud_filtered);

    // Publish the data
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
    point_cloud_pub_.publish(cloud_filtered_msg);
}

// ROS node to reduce point cloud using VoxelGrid filter
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "reduce_points");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    // get grid_size param
    if (p_nh.hasParam("grid_size"))
    {
        p_nh.getParam("grid_size", grid_size_);
    }
    else
    {
        grid_size_ = 0.05;
    }

    // set ros loop rate (point cloud publish rate)
    ros::Rate loop_rate(1.0);

    // advertise point cloud topic
    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2_reduced", 1);

    // subscribe to input point cloud
    ros::Subscriber point_cloud_sub = nh.subscribe("points2", 1, point_cloud_reduce_callback);

    ros::spin();
}
