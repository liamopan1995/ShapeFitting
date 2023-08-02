#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include "patchworkpp/patchworkpp.hpp"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZI PointType;

// write a class which iherits from the class Patchworkpp
class cloudSegmentation {
  private:

    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    
    //  this class processes an object instantiated from an class(let's call it Stemworkpp) , which is a subclass of Patchworkpp
    boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubStemCloud;

    // debugging use only 
    ros::Publisher pubDebug;
    ros::Subscriber subLaserCloud;
    // end of debugging use only

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr nonGroundCloud;
    pcl::PointCloud<PointType>::Ptr stemCloud;

    PointType nanPoint;

    std_msgs::Header cloudHeader;
    double time_taken;

  public:
   cloudSegmentation():
    nh(), pnh("~"){

      subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &cloudSegmentation::cloudHandler, this);
      
      pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
      pubStemCloud = nh.advertise<sensor_msgs::PointCloud2> ("/stem_cloud", 1);
      
      // for debugging use 
      pubDebug = nh.advertise<sensor_msgs::PointCloud2> ("/debugging", 1);

      nanPoint.x = std::numeric_limits<float>::quiet_NaN();
      nanPoint.y = std::numeric_limits<float>::quiet_NaN();
      nanPoint.z = std::numeric_limits<float>::quiet_NaN();
      nanPoint.intensity = -1;

      allocateMemory();
      resetParameters();

    }

    // pnh("~"){

    // }
    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        groundCloud.reset(new pcl::PointCloud<PointType>());
        nonGroundCloud.reset(new pcl::PointCloud<PointType>());
        stemCloud.reset(new pcl::PointCloud<PointType>());
        std::string cloud_topic;
        pnh.param<string>("cloud_topic", cloud_topic, "/velodyne_points");
        PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&pnh));
        
    }

	// Init,rest memebers
    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        nonGroundCloud->clear();
        stemCloud->clear();

        std::fill(laserCloudIn->points.begin(), laserCloudIn->points.end(), nanPoint);
    }

    ~cloudSegmentation(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        // 将ROS中的sensor_msgs::PointCloud2ConstPtr类型转换到pcl点云库指针
        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);


        // this part is working

    }

    void voxelFilter(){
        pcl::PointCloud<PointType>::Ptr pc_voxel_filtered(new pcl::PointCloud<PointType>);
        pcl::VoxelGrid<PointType> voxel_filter;
        voxel_filter.setInputCloud(laserCloudIn);
        voxel_filter.setLeafSize(0.5, 0.5, 0.5);  
        voxel_filter.filter(*laserCloudIn);

        // this part is working
    }

    void removePointsInBox() {
        pcl::CropBox<PointType> cropFilter;
        cropFilter.setInputCloud(laserCloudIn);
        cropFilter.setMin(Eigen::Vector4f(-5, -5, -5, 1.0)); // Minimum coordinates of the box
        cropFilter.setMax(Eigen::Vector4f(5, 5, 5, 1.0)); // Maximum coordinates of the box

        // Set the filter to remove points inside the box
        cropFilter.setNegative(true); 
        pcl::PointCloud<PointType>::Ptr outputCloud(new pcl::PointCloud<PointType>);
        cropFilter.filter(*outputCloud);

        // Update the original input cloud with the filtered points
        *laserCloudIn = *outputCloud;
        // now it is working well
    }
    void removeByIntensity(pcl::PointCloud<PointType>::Ptr& inputCloud,float intensityThreshold=8){
        // Set the intensity threshold for outlier removal
        // started from 100, cut to half after each iteration 24 was the optimal when input topic is velodyne_points
        
        // Create the condition object
        pcl::ConditionAnd<PointType>::Ptr intensityCondition (new pcl::ConditionAnd<PointType> ());
        intensityCondition->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new 
        pcl::FieldComparison<PointType> ("intensity", pcl::ComparisonOps::GT, intensityThreshold)));

        // Create the conditional removal filter
        pcl::ConditionalRemoval<PointType> cr;
        cr.setInputCloud(inputCloud);
        //cr.setInputCloud(nonGroundCloud);
        cr.setCondition(intensityCondition);
        cr.setKeepOrganized(true); // Set to true if you want to preserve the structure of the input cloud

        // Create a place holder then apply the intensity threshold condition to remove outliers
        pcl::PointCloud<PointType>::Ptr filteredIntensity(new pcl::PointCloud<PointType>);
        cr.filter(*filteredIntensity);
        //nonGroundCloud = filteredIntensity;
        inputCloud = filteredIntensity;

        // this part works well too
    }

    void projectTo2D(){
    // void projectTo2D(pcl::PointCloud<PointType>::Ptr& nonGroundCloud){
        for (auto& point : nonGroundCloud->points) {
        point.intensity = point.z;
        point.z = 0.0;
        }
        // this wokks well inside the function
    }

    void removeNan(){
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*nonGroundCloud, *nonGroundCloud, indices);
    }

    void radiusOutlierRemoval(int radius=9){
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> filter;
        filter.setInputCloud(nonGroundCloud);
        filter.setRadiusSearch(0.32); 
        filter.setMinNeighborsInRadius(radius); // Tune this 
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredRadiusOutlierRemoval(new pcl::PointCloud<pcl::PointXYZI>);
        filter.filter(*filteredRadiusOutlierRemoval);
        nonGroundCloud = filteredRadiusOutlierRemoval;
    }
    void projectTo3D(){
    //void projectTo3D(pcl::PointCloud<PointType>::Ptr& nonGroundCloud){
        for (auto& point : nonGroundCloud->points) {
        point.z = point.intensity;
        }
    }

    void stemSeg(){
        removeByIntensity(nonGroundCloud);
        projectTo2D();
        removeNan();
        radiusOutlierRemoval();
        projectTo3D();
    }

    void publishCloud(){

        sensor_msgs::PointCloud2 laserCloudTemp;

        // segmented ground cloud 
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pubGroundCloud.publish(laserCloudTemp);
        }

        // segmented stem cloud 
        if (pubStemCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*nonGroundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = cloudHeader.frame_id;
            pubStemCloud.publish(laserCloudTemp);
        }

    }
    
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

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
        PatchworkppGroundSeg->estimate_ground(*laserCloudIn,*groundCloud,*nonGroundCloud,time_taken); 
        stemSeg();
        publishCloud();

    }
    

    

};

int main(int argc, char** argv){

    ros::init(argc, argv, "cloudseg");
    
    cloudSegmentation IP;

    ROS_INFO("\033[1;32m---->\033[0m Cloud Segmentation Started.");

    ros::spin();
    return 0;
}

