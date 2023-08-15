
// #ifndef CIRCLE_CLUSTER
// #define CIRCLE_CLUSTER


// #include <ros/ros.h>
// #include <signal.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/conditional_removal.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/segmentation/extract_clusters.h>

// typedef pcl::PointXYZI PointType;

// // pcl::PointCloud<PointType>::Ptr nonGroundCloud;
// // std::vector<pcl::PointIndices> cluster_indices_stem;
// // std::vector<pcl::PointIndices> cluster_indices_circle;

// // pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud_stem;      // cloud of points in one stem
// // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud_circle; // cloud of points in one circle
// // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud;  // cloud of points in one whole frame
// // encapsulize it as a class ? 


// // void circleClustering(  )
// // {

// //     // // Clear the point cloud before clustering
// //     // clustered_cloud_frame->clear();
// //     // Zvalue.clear();
// //     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

// //     // Now cluster pointcloud into seperate trunks first

// //     ec.setClusterTolerance(0.1); // aug 4 = 0.3
// //     ec.setMinClusterSize(10);    // aug4 = 20
// //     ec.setMaxClusterSize(1000);  //   60 was good.

// //     ec.setInputCloud(clustered_cloud_stem); //
// //     ec.extract(cluster_indices_circle);

// //     for (const pcl::PointIndices &cluster : cluster_indices_circle)
// //     {
// //         // Generate a random RGB color
// //         uint8_t r = std::rand() % 256;
// //         uint8_t g = std::rand() % 256;
// //         uint8_t b = std::rand() % 256;

// //         // Pack RGB values into a single float
// //         int32_t rgb_value = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

// //         for (const int index : cluster.indices)
// //         {
// //             // Create a new point with assigned color
// //             pcl::PointXYZRGB point;
// //             point.x = clustered_cloud_stem->points[index].x;
// //             point.y = clustered_cloud_stem->points[index].y;
// //             point.z = clustered_cloud_stem->points[index].z;

// //             point.rgb = *reinterpret_cast<float *>(&rgb_value); // Set RGB value
// //             clustered_cloud->points.push_back(point); 
// //         }
// //         //  The clustered_cloud_circle now is containing several clusters of points, each cluster stand for a circle
// //         // clustered_cloud_circle is ready for circle fitting 



// //     }
// // }

// void stemClustering()
// {

//     // Clear the point cloud before clustering
//     clustered_cloud->clear();
//     pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
//     // Now cluster pointcloud into seperate trunks first

//     ec.setClusterTolerance(0.1); // aug 4 = 0.3
//     ec.setMinClusterSize(10);    // aug4 = 20
//     ec.setMaxClusterSize(1000);  //   60 was good.

//     ec.setInputCloud(nonGroundCloud); //
//     ec.extract(cluster_indices_stem);

//     for (const pcl::PointIndices &cluster : cluster_indices_stem)
//     {
        
//         pcl::PointXYZ point;
//         // Clear the point cloud before clustering
//         clustered_cloud_stem->clear();

//         for (const int index : cluster.indices)
//         {
//             // Create a new point without color
            
//             point.x = nonGroundCloud->points[index].x;
//             point.y = nonGroundCloud->points[index].y;
//             point.z = nonGroundCloud->points[index].intensity;

//             clustered_cloud_stem->points.push_back(point);
//         }

//         // here we have a point cloud named : clustered_cloud_stem filled with points belong to one stem
//         /**** Do another clustering on point , to get clusters of circle  in each cluster ****/

//         circleClustering();  // Returns  cluster_indices

//             // for (auto : cluster_indices) 

//             // {    // construct a point cloud with color for visualizing 

//                     // do circle fitting 
//                     // build circle struct 

//                     // push the fitted circle into a vector 
//             // }


//     }

//     //  TODO:

//     // 1. remove outliers  by radius
//     // 2. reconstruct z value

//     cluster_indices_stem.clear();

//     clustered_cloud->width = clustered_cloud->size();
//     clustered_cloud->height = 1;
//     clustered_cloud->is_dense = true;

// }

// void printLineToROSTerminal() {
//     ROS_INFO("This is a line printed to the ROS terminal.");
// }




// #endif