#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>



 class cloudSegmentation {
private:
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;

public:
    cloudSegmentation() : nh() {
        //subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/ground_cloud", 1, &cloudSegmentation::cloudHandler, this);
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/stem_cloud", 1, &cloudSegmentation::cloudHandler, this);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        // Implement your code to process the received point cloud data here
    }
};
   