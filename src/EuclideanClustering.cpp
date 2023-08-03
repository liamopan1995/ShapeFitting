//    copied , compiled runable
//

//**   the parameter setting which has the best peformance sofar.

    // ec.setClusterTolerance(0.3); // Set the distance tolerance for clustering    // 0.02, 0.3, 0.2 , 0.5
    // ec.setMinClusterSize(100*1.5);    // Set the minimum cluster size
    // ec.setMaxClusterSize(25000);  // Set the maximum cluster size
    // ec.setInputCloud(cloud);




#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert ROS point cloud message to PCL point cloud
    pcl::fromROSMsg(*msg, *cloud);

    // Apply segmentation algorithm (e.g., EuclideanClusterExtraction)
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;


    // The following setting works well for clustering circles from stems
    
    // ec.setClusterTolerance(0.2); //
    // ec.setMinClusterSize(3);    // 
    // ec.setMaxClusterSize(200);  //   60 was good.
    // Save the above setting for latter 

    // Now cluster pointcloud into seperate trunks first
    // the following setting is for clustering stems from whole cloud

    ec.setClusterTolerance(0.4); //
    ec.setMinClusterSize(150);    // 
    ec.setMaxClusterSize(700);  //   


    


    ec.setInputCloud(cloud);    // 
    // ec.setInputCloud(cloud_filtered);    
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // Place holder pointXYZRGB 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Iterate through each cluster and add the points to the segmented point cloud
    for (const pcl::PointIndices& cluster : cluster_indices) {
        // Generate a random RGB color
        uint8_t r = std::rand() % 256;
        uint8_t g = std::rand() % 256;
        uint8_t b = std::rand() % 256;

        // Pack RGB values into a single float
        uint32_t rgb_value = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

        for (const int index : cluster.indices) {
            // Create a new point with assigned color
            pcl::PointXYZRGB point;
            point.x = cloud->points[index].x;
            point.y = cloud->points[index].y;
            point.z = cloud->points[index].z;
            point.rgb = *reinterpret_cast<float*>(&rgb_value); // Set RGB value
            segmented_cloud->points.push_back(point);
        }
    }

    segmented_cloud->width = segmented_cloud->size();
    segmented_cloud->height = 1;
    segmented_cloud->is_dense = true;

    // Convert the segmented point cloud to a ROS message
    sensor_msgs::PointCloud2 segmented_msg;
    pcl::toROSMsg(*segmented_cloud, segmented_msg);
    segmented_msg.header = msg->header;

    // Publish the segmented point cloud
    pub.publish(segmented_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "euc_cluster_node");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m---->\033[0m Euclidean Clustering Started.");
    // Create a subscriber for the input point cloud
    ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/stem_cloud", 1, pointCloudCallback);

    // Create a publisher for the segmented point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("clustered_cloud_euc", 1);

    ros::spin();

    return 0;
}
