#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include "patchworkpp/patchworkpp.hpp"
// #include "utilities/CircleClsuter.hpp"




typedef pcl::PointXYZI PointType;

// write a class which iherits from the class Patchworkpp
class cloudSegmentation
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    //  this class processes an object instantiated from an class(let's call it Stemworkpp) , which is a subclass of Patchworkpp
    boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubStemCloud;
    ros::Publisher pubTrunksCluster;

    // debugging use only
    ros::Publisher pubDebug;
    ros::Subscriber subLaserCloud;
    // end of debugging use only

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr nonGroundCloud;
    pcl::PointCloud<PointType>::Ptr stemCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud; // Place holder pointXYZRGB.. // TRY : pointcloud XYZRGBA latter
    std::vector<pcl::PointIndices> cluster_indices;         // store the cluster indice

    /********    For Circle clustering        ********/
    std::vector<pcl::PointIndices> cluster_indices_stem;
    std::vector<pcl::PointIndices> cluster_indices_circle;

    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud_stem;      // cloud of points in one stem

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud_circle; // cloud of points in one circle
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud;  // cloud of points in one whole frame
    /********  End of   For Circle clustering   ********/

    PointType nanPoint;
    std_msgs::Header cloudHeader;
    double time_taken;
    // std::vector<float> Zvalue; intentially used for storing z value.

public:
    cloudSegmentation() : nh(), pnh("~")
    {

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &cloudSegmentation::cloudHandler, this);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
        pubStemCloud = nh.advertise<sensor_msgs::PointCloud2>("/stem_cloud", 1);
        pubTrunksCluster = nh.advertise<sensor_msgs::PointCloud2>("clustered_cloud_trunks", 1);
        // for debugging use
        pubDebug = nh.advertise<sensor_msgs::PointCloud2>("/debugging", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
    }

    // pnh("~"){

    // }
    void allocateMemory()
    {

        laserCloudIn.reset(new pcl::PointCloud<PointType>);
        groundCloud.reset(new pcl::PointCloud<PointType>);
        nonGroundCloud.reset(new pcl::PointCloud<PointType>);
        stemCloud.reset(new pcl::PointCloud<PointType>);
        clustered_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        /****  for circle clustering *****/
        clustered_cloud_stem.reset(new pcl::PointCloud<pcl::PointXYZ>);
        // clustered_cloud_circle.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        /*     end of ...*/

        std::string cloud_topic;
        pnh.param<string>("cloud_topic", cloud_topic, "/velodyne_points");
        PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&pnh));
        resetParameters();
    }

    // Init,rest memebers
    void resetParameters()
    {

        laserCloudIn->clear();
        groundCloud->clear();
        nonGroundCloud->clear();
        stemCloud->clear();
        clustered_cloud->clear();
        std::fill(laserCloudIn->points.begin(), laserCloudIn->points.end(), nanPoint);
    }

    ~cloudSegmentation() {}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // 将ROS中的sensor_msgs::PointCloud2ConstPtr类型转换到pcl点云库指针
        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

        // this part is working
    }

    void voxelFilter()
    {
        pcl::PointCloud<PointType>::Ptr pc_voxel_filtered(new pcl::PointCloud<PointType>);
        pcl::VoxelGrid<PointType> voxel_filter;
        voxel_filter.setInputCloud(laserCloudIn);
        voxel_filter.setLeafSize(0.5, 0.5, 0.5);
        voxel_filter.filter(*laserCloudIn);

        // this part is working
    }

    void removePointsInBox()
    {
        pcl::CropBox<PointType> cropFilter;
        cropFilter.setInputCloud(laserCloudIn);
        cropFilter.setMin(Eigen::Vector4f(-5, -5, -5, 1.0)); // Minimum coordinates of the box
        cropFilter.setMax(Eigen::Vector4f(5, 5, 5, 1.0));    // Maximum coordinates of the box

        // Set the filter to remove points inside the box
        cropFilter.setNegative(true);
        pcl::PointCloud<PointType>::Ptr outputCloud(new pcl::PointCloud<PointType>);
        cropFilter.filter(*outputCloud);

        // Update the original input cloud with the filtered points
        *laserCloudIn = *outputCloud;
        // now it is working well
    }
    void removeByIntensity(pcl::PointCloud<PointType>::Ptr &inputCloud, float intensityThreshold = 8)
    {
        // Set the intensity threshold for outlier removal
        // started from 100, cut to half after each iteration 24 was the optimal when input topic is velodyne_points

        // Create the condition object
        pcl::ConditionAnd<PointType>::Ptr intensityCondition(new pcl::ConditionAnd<PointType>());
        intensityCondition->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("intensity", pcl::ComparisonOps::GT, intensityThreshold)));

        // Create the conditional removal filter
        pcl::ConditionalRemoval<PointType> cr;
        cr.setInputCloud(inputCloud);
        // cr.setInputCloud(nonGroundCloud);
        cr.setCondition(intensityCondition);
        cr.setKeepOrganized(true); // Set to true if you want to preserve the structure of the input cloud

        // Create a place holder then apply the intensity threshold condition to remove outliers
        pcl::PointCloud<PointType>::Ptr filteredIntensity(new pcl::PointCloud<PointType>);
        cr.filter(*filteredIntensity);
        // nonGroundCloud = filteredIntensity;
        inputCloud = filteredIntensity;

        // this part works well too
    }

    void projectTo2D()
    {
        // void projectTo2D(pcl::PointCloud<PointType>::Ptr& nonGroundCloud){
        for (auto &point : nonGroundCloud->points)
        {
            point.intensity = point.z;
            point.z = 0.0;
        }
        // this wokks well inside the function
    }

    void removeNan()
    {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*nonGroundCloud, *nonGroundCloud, indices);
    }

    void radiusOutlierRemoval(int minNeigh = 9)   // aug 4. = 9
    { // radius=9 works well
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> filter;
        filter.setInputCloud(nonGroundCloud);
        filter.setRadiusSearch(0.2);  // aug 4. =0.32
        filter.setMinNeighborsInRadius(minNeigh); // Tune this
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredRadiusOutlierRemoval(new pcl::PointCloud<pcl::PointXYZI>);
        filter.filter(*filteredRadiusOutlierRemoval);
        nonGroundCloud = filteredRadiusOutlierRemoval;
    }

    void euclideanClustering()
    {

        // Clear the point cloud before clustering
        clustered_cloud->clear();
        // Zvalue.clear();
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;

        // Now cluster pointcloud into seperate trunks first

        ec.setClusterTolerance(0.1); // aug 4 = 0.3
        ec.setMinClusterSize(10);    // aug4 = 20
        ec.setMaxClusterSize(1000);  //   60 was good.

        ec.setInputCloud(nonGroundCloud); //
        ec.extract(cluster_indices);

        for (const pcl::PointIndices &cluster : cluster_indices)
        {
            // Generate a random RGB color
            uint8_t r = std::rand() % 256;
            uint8_t g = std::rand() % 256;
            uint8_t b = std::rand() % 256;

            // Pack RGB values into a single float
            int32_t rgb_value = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

            for (const int index : cluster.indices)
            {
                // Create a new point with assigned color
                pcl::PointXYZRGB point;
                point.x = nonGroundCloud->points[index].x;
                point.y = nonGroundCloud->points[index].y;
                point.z = nonGroundCloud->points[index].intensity;
                //point.intensity = nonGroundCloud->points[index].intensity;
                // Zvalue.push_back(nonGroundCloud->points[index].z);

                point.rgb = *reinterpret_cast<float *>(&rgb_value); // Set RGB value
                clustered_cloud->points.push_back(point);
            }
        }

        //  TODO:

        // 1. remove outliers  by radius
        // 2. reconstruct z value

        cluster_indices.clear();

        clustered_cloud->width = clustered_cloud->size();
        clustered_cloud->height = 1;
        clustered_cloud->is_dense = true;
    }

        

    void circleClustering( )
    /*    
    this function doesnt perform very well   and runs rather slow , it could be better to consider
    clustering based on the difference of z values
     */
    {

        // // Clear the point cloud before clustering
        // clustered_cloud_frame->clear();
        // Zvalue.clear();
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

        // Now cluster pointcloud into seperate trunks first

        ec.setClusterTolerance(100); // aug 4 = 0.3
        ec.setMinClusterSize(4);    // aug4 = 20
        ec.setMaxClusterSize(1000);  //   60 was good.

        ec.setInputCloud(clustered_cloud_stem); //
        ec.extract(cluster_indices_circle);

        for (const pcl::PointIndices &cluster : cluster_indices_circle)
        {
            // Generate a random RGB color
            uint8_t r = std::rand() % 256;
            uint8_t g = std::rand() % 256;
            uint8_t b = std::rand() % 256;

            // Pack RGB values into a single float
            int32_t rgb_value = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

            for (const int index : cluster.indices)
            {
                // Create a new point with assigned color
                pcl::PointXYZRGB point;
                point.x = clustered_cloud_stem->points[index].x;
                point.y = clustered_cloud_stem->points[index].y;
                point.z = clustered_cloud_stem->points[index].z;

                point.rgb = *reinterpret_cast<float *>(&rgb_value); // Set RGB value
                clustered_cloud->points.push_back(point); 
            }
            //  The clustered_cloud_circle now is containing several clusters of points, each cluster stand for a circle
            // clustered_cloud_circle is ready for circle fitting 



        }
    }

    void stemClustering()
    {

        // Clear the point cloud before clustering
        clustered_cloud->clear();
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        // Now cluster pointcloud into seperate trunks first

        ec.setClusterTolerance(0.1); // aug 4 = 0.3
        ec.setMinClusterSize(10);    // aug4 = 20
        ec.setMaxClusterSize(1000);  //   60 was good.

        ec.setInputCloud(nonGroundCloud); //
        ec.extract(cluster_indices_stem);

        for (const pcl::PointIndices &cluster : cluster_indices_stem)
        {
            
            pcl::PointXYZ point;
            // Clear the point cloud before clustering
            clustered_cloud_stem->clear();

            for (const int index : cluster.indices)
            {
                // Create a new point without color
                
                point.x = nonGroundCloud->points[index].x;
                point.y = nonGroundCloud->points[index].y;
                point.z = nonGroundCloud->points[index].intensity;

                clustered_cloud_stem->points.push_back(point);
            }

            // here we have a point cloud named : clustered_cloud_stem filled with points belong to one stem
            /**** Do another clustering on point , to get clusters of circle  in each cluster ****/

            circleClustering();  // Returns  cluster_indices

                // for (auto : cluster_indices) 

                // {    // construct a point cloud with color for visualizing 

                        // do circle fitting 
                        // build circle struct 

                        // push the fitted circle into a vector 
                // }


        }

        //  TODO:

        // 1. remove outliers  by radius
        // 2. reconstruct z value

        cluster_indices_stem.clear();

        clustered_cloud->width = clustered_cloud->size();
        clustered_cloud->height = 1;
        clustered_cloud->is_dense = true;

    }
    void projectTo3D()
    {
        // void projectTo3D(pcl::PointCloud<PointType>::Ptr& nonGroundCloud){
        for (auto &point : nonGroundCloud->points)
        {
            point.z = point.intensity;
        }
    }

    void stemSeg()
    {
        removeByIntensity(nonGroundCloud);
        projectTo2D();
        removeNan();
        radiusOutlierRemoval();
        //euclideanClustering(); // do stems grouping
        stemClustering();
        projectTo3D();
    }

    void publishCloud()
    {

        sensor_msgs::PointCloud2 laserCloudTemp;

        // segmented ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pubGroundCloud.publish(laserCloudTemp);
        }

        // segmented stem cloud
        if (pubStemCloud.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*nonGroundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pubStemCloud.publish(laserCloudTemp);

        }
        if (pubTrunksCluster.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*clustered_cloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pubTrunksCluster.publish(laserCloudTemp);
        }
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {

        copyPointCloud(laserCloudMsg);

        // Trying with intensity filter and tuninng it started from 9 :
        // We were applying this para=9 to discard points that are less proble
        // to be tree stems at the purely tree trunk segmentation, therefore
        // the para should be less then 9 at here.  started with :7 ,6 ,5
        // removeByIntensity(laserCloudIn,3);
        // Result: removebyintensity doe not have any noticeable improvement on the result
        // and will potentially  slow down the computation. so it is better not to have it here neither.

        // Trying with voxel filter removed
        // voxelFilter();
        // Confirmed:[It is better not to has voxel filter here]

        removePointsInBox();
        PatchworkppGroundSeg->estimate_ground(*laserCloudIn, *groundCloud, *nonGroundCloud, time_taken);
        //printLineToROSTerminal();
        stemSeg();
        publishCloud();
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "cloudseg");

    cloudSegmentation IP;

    ROS_INFO("\033[1;32m---->\033[0m Cloud Segmentation Started.");

    ros::spin();
    return 0;
}
