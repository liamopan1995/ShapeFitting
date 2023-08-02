#include <iostream>
// This file does the stem segmentation work
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Geometry>
#include "patchworkpp/patchworkpp.hpp"


using PointType = pcl::PointXYZI;
using namespace std;

// function removing points that are to close to sensor root(origin) by its abs distance
pcl::PointCloud<pcl::PointXYZI>::Ptr cropPointCloudBySphere(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                                           const Eigen::Vector3f& center, float radius);
// PatchWorkpp is a class defined in  "patchworkpp/patchworkpp.hpp"
boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

ros::Publisher pub_cloud;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const ros::Time& stamp, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.stamp = stamp;
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}


void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    double time_taken;

    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;

    pcl::fromROSMsg(*cloud_msg, pc_curr);

    //1. add a voxel filter or a outlier removal filter

    // Voxel Filter
    pcl::PointCloud<PointType>::Ptr pc_voxel_filtered(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(pc_curr.makeShared());
    voxel_filter.setLeafSize(0.5, 0.5, 0.5);  
    voxel_filter.filter(*pc_voxel_filtered);



    // // Alternative: REMOVAL BY INTENSITY
    // // Set the intensity threshold for outlier removal
    // float intensityThreshold = 24;  // started from 100, cut to half after each iteration 

    // // Create the condition object
    // pcl::ConditionAnd<pcl::PointXYZI>::Ptr intensityCondition (new pcl::ConditionAnd<pcl::PointXYZI> ());
    // intensityCondition->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new 
    // pcl::FieldComparison<pcl::PointXYZI> ("intensity", pcl::ComparisonOps::GT, intensityThreshold)));

    // // Create the conditional removal filter
    // pcl::ConditionalRemoval<pcl::PointXYZI> cr;
    // // set target point cloud to be processed
    // //cr.setInputCloud(filteredLevel);
    // //cr.setInputCloud(cloud);
    // cr.setInputCloud(pc_curr.makeShared());
    // cr.setCondition(intensityCondition);
    // cr.setKeepOrganized(true); // Set to true if you want to preserve the structure of the input cloud

    // // Create a place holder then apply the intensity threshold condition to remove outliers
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filteredIntensity(new pcl::PointCloud<pcl::PointXYZI>);
    // cr.filter(*filteredIntensity);

    // 2. add a remove by abs distance, which keeps points that have a distance from orgin higher than
    // a specific threshold 

    // Removing the points that are to close to the sensor root
    // Define the sphere parameters
    Eigen::Vector3f center(0.0, 0.0, 0.0);
    float radius = 15.0;

    // Crop the point cloud using the custom sphere cropping filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr croppedCloud = cropPointCloudBySphere(pc_voxel_filtered, center, radius);

    PatchworkppGroundSeg->estimate_ground(*pc_voxel_filtered, pc_ground, pc_non_ground, time_taken);

    ROS_INFO_STREAM("\033[1;32m" << "Input PointCloud: " << pc_curr.size() << " -> Ground: " << pc_ground.size() <<  "/ NonGround: " << pc_non_ground.size()
         << " (running_time: " << time_taken << " sec)" << "\033[0m");

    pub_cloud.publish(cloud2msg(pc_curr, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_ground.publish(cloud2msg(pc_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_non_ground.publish(cloud2msg(pc_non_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
}

int main(int argc, char**argv) {

    ros::init(argc, argv, "Demo");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string cloud_topic;
    pnh.param<string>("cloud_topic", cloud_topic, "/pointcloud");

    cout << "Operating patchwork++..." << endl;
    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&pnh));

    pub_cloud       = pnh.advertise<sensor_msgs::PointCloud2>("cloud", 100, true);
    pub_ground      = pnh.advertise<sensor_msgs::PointCloud2>("ground", 100, true);
    pub_non_ground  = pnh.advertise<sensor_msgs::PointCloud2>("nonground", 100, true);

    ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 100, callbackCloud);
    
    ros::spin();

    return 0;
}


// function removing points that are to close to sensor root(origin) by its abs distance
pcl::PointCloud<pcl::PointXYZI>::Ptr cropPointCloudBySphere(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                                           const Eigen::Vector3f& center, float radius)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZI>);

  for (const pcl::PointXYZI& point : cloud->points)
  {
    Eigen::Vector3f pointVector(point.x, point.y, point.z);
    float distance = (pointVector - center).norm();

    if (distance >= radius)
    {
      croppedCloud->points.push_back(point);
    }
  }

  croppedCloud->width = croppedCloud->points.size();
  croppedCloud->height = 1;
  croppedCloud->is_dense = true;

  return croppedCloud;
}
