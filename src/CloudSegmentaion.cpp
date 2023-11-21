#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include "patchworkpp/patchworkpp.hpp"
#include "circle_fitting_integrate/CircleFit_Integrate.hpp"
#include "common/utilities.h"

#include "icp/icp2d.h"
#include "pose_graph/PoseGraphBuilder.hpp"
#include <g2o/types/slam2d/types_slam2d.h>
// #include "utilities/CircleClsuter.hpp"

// TODO : 0. Organize the file into hpp, cpp. the current file is too messy,  and add comments to each function about its usage
//        1. Make a node which has circleclustering function (basing on stemCluster() and circleCluster() in followings) and can
//           display the clustered result on rivz.
//        2. So you can subsscribe to the point cloud of stem segmentaion which is  generated by other method
//           and also do clustering on the resulted point cloud of them , this is necessary for later evaluation 
//        3. Write and library of circle fitting, or first data structe development , then try capture some points of circle and 
//           save them to local , later load them on python , and to evaluate those fitting algorithem implemented in python, since
//           it is more convinient to visualize the data on matplotlib. 
//        4. Write a utility.h which contains the Parameter Settings and type define and self defined data structures as well.
//       
//        Augst 10th

// Inline function to generate a random RGB color value

// After testing , put the otherweres e.g. utilitys
inline int32_t generateRandomRGB() {
    uint8_t r = std::rand() % 256;
    uint8_t g = std::rand() % 256;
    uint8_t b = std::rand() % 256;
    return (static_cast<int32_t>(r) << 16 | static_cast<int32_t>(g) << 8 | static_cast<int32_t>(b));
}


typedef pcl::PointXYZI PointType;

//  Set viewmode to 1, to view clusters of points, which are considered as prior circles
//  Set viewmode to 2, to view clusters stems, each stem are made of fitted circles
const int viewmode = 0;
const int MIN_STEM_NUM =3;

int pose_i = 1;
int failure_num = 0;
// write a class which iherits from the class Patchworkpp
class cloudSegmentation
{
private:

    Single_scan single_scan;
    vector<Single_scan> scans; //  corresponds to single_scan.txt
    PoseGraphBuilder poseGraphBuilder;
/************************************** 20 Nov ************************************************/
    Icp2d icp_2d;
    std::vector<MovementData> odometry;
    std::vector<MovementData> translation_pose2pose;
    std::vector<Vec6d> global_map;
    std::vector<Vec6d> local_map;


/************************************** End 20 No v************************************************/

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    //  this class processes an object instantiated from an class(let's call it Stemworkpp) , which is a subclass of Patchworkpp
    boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubStemCloud;
    ros::Publisher pubTrunksCluster;
    ros::Publisher pubPath;
    ros::Publisher pubLandmarks;

    // debugging use only
    ros::Publisher pubDebug;
    ros::Subscriber subLaserCloud;
    // end of debugging use only

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr nonGroundCloud;
    pcl::PointCloud<PointType>::Ptr stemCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud; // Place holder pointXYZRGB.. // TRY : pointcloud XYZRGBA latter
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_circle_cloud;
    std::vector<pcl::PointIndices> cluster_indices;         // store the cluster indice

    /********    For Circle clustering        ********/

    std::vector<pcl::PointIndices> cluster_indices_stem;
    std::vector<pcl::PointIndices> cluster_indices_circle;
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr clustered_cloud_stem;      // cloud of points in one stem

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud_circle; // cloud of points in one circle
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud;  // cloud of points in one whole frame

    /********  End of   For Circle clustering   ********/

    PointType nanPoint;
    std_msgs::Header cloudHeader;
    double time_taken;
    // std::vector<float> Zvalue; intentially used for storing z value.

public:
    /************** 20 Nov ************/
    Vec3d path_accumulated =Vec3d::Zero();
    Mat3d Rotation_accumulated = Mat3d::Identity();
    /************** End 20 Nov************/

    void saveToFiles() const {
        /// Saves tree infomations ( x, y ,r ) as one row to local
        /// In each txt file , it contains several rows ,each represent a single tree (stem)
        /// So later visulization tools can  be applied on these files to see 
        /// how well the fitting was processed, and can provide straight ituition for scan matching  process
        const char* homeDir = getenv("HOME");
        if (!homeDir) {
            std::cerr << "Error: HOME environment variable not set." << std::endl;
            return;
        }
        int scanIdx = 0;
        for (const auto& scan : scans) {
            // Construct the desired path with unique filename
            std::string fullPath = std::string(homeDir) + "/catkin_ws_aug/src/shapefitting/scans/"
                                + "single_scan_" + std::to_string(scanIdx++) + ".txt";

            std::ofstream fout(fullPath);
            if (!fout) {
                std::cerr << "Error: Unable to open file for writing: " << fullPath << std::endl;
                continue; // move to the next file
            }
            //  modified at oct.
            int stemIdx = 0;
            for (const auto& tree_info : scan.tree_infos_) {
                fout<< std::setprecision(18)<< scan.time_stamp_ << " " 
                    << std::setprecision(9)
                    << tree_info.x_ << " " 
                    << tree_info.y_ << " " 
                    << tree_info.r_ << " " 
                    << stemIdx++          << std::endl;
            }

        }
    }
        
    cloudSegmentation() : nh(), pnh("~")
    {

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &cloudSegmentation::cloudHandler, this);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
        pubStemCloud = nh.advertise<sensor_msgs::PointCloud2>("/stem_cloud", 1);
        pubTrunksCluster = nh.advertise<sensor_msgs::PointCloud2>("clustered_cloud_trunks", 1);
        pubPath = nh.advertise<nav_msgs::Path>("path_topic", 1000);
        pubLandmarks= nh.advertise<visualization_msgs::MarkerArray>("landmarks", 1000);


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
        cylinder_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        single_circle_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        /****  for circle clustering *****/
        clustered_cloud_stem.reset(new pcl::PointCloud<pcl::PointXYZLNormal>);
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
        cylinder_cloud->clear();
        single_circle_cloud->clear();
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
        cluster_indices.clear();
        ec.extract(cluster_indices);

        for (const pcl::PointIndices &cluster : cluster_indices)
        {
            // Generate a random RGB color
            int32_t rgb_value = generateRandomRGB();
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


        clustered_cloud->width = clustered_cloud->size();
        clustered_cloud->height = 1;
        clustered_cloud->is_dense = true;
    }

        

    void circleClustering( )
    /*    
    This function is used to further cluster a single stem group into groups , where points in each of those groups likely 
    form a circle.
     */
    {

        pcl::EuclideanClusterExtraction<pcl::PointXYZLNormal> ec;

        // Now cluster pointcloud into seperate circles according to z value

        ec.setClusterTolerance(0.1); 
        ec.setMinClusterSize(4);     
        ec.setMaxClusterSize(1000);  

        ec.setInputCloud(clustered_cloud_stem); //
        cluster_indices_circle.clear();
        ec.extract(cluster_indices_circle);
        // //Generate a random RGB color
        int32_t rgb_value = generateRandomRGB();

        //int num_circle =0;
        Stem stem_candidate;
        cylinder_cloud->clear();
        for (const pcl::PointIndices &cluster : cluster_indices_circle)
        {   
            if(viewmode==1){
            /*
            Start of cylinder clusters (made of points) visualizer
            */
                //Generate a random RGB color
                int32_t rgb_value = generateRandomRGB();

                for (const int index : cluster.indices)
                {
                    // Create a new point with assigned color
                    pcl::PointXYZRGB point;
                    point.x = clustered_cloud_stem->points[index].normal_x;
                    point.y = clustered_cloud_stem->points[index].normal_y;
                    // point.x = 0;
                    // point.y = 0;
                    point.z = clustered_cloud_stem->points[index].z;

                    point.rgb = *reinterpret_cast<float *>(&rgb_value); // Set RGB value
                    clustered_cloud->points.push_back(point); 
                    // Here we can store the points belonging to the same circle into our data structure: circle
                }
            }

            /*
            End of cylinder clusters (made of points) visualizer
            */

            
            // Following is the Original way of adding previously unkown size of points into a matrix

            // Dynamic-sized matrix for points on which a circle is to be fitted 
            // Eigen::Matrix<reals, Eigen::Dynamic, 3> points_to_fit; 
            
            // // Writting all points into a matrix , for esitmating the fitted circle 
            // for (const int index : cluster.indices) {
            //     points_to_fit.conservativeResize(points_to_fit.rows() + 1, Eigen::NoChange);  // Add a row
            //     points_to_fit.row(points_to_fit.rows() - 1) << clustered_cloud_stem->points[index].normal_x, clustered_cloud_stem->points[index].normal_y, clustered_cloud_stem->points[index].z;
            // }

            Eigen::Matrix<reals, Eigen::Dynamic, 3> points_to_fit(cluster.indices.size(), 3);

            for (size_t i = 0; i < cluster.indices.size(); ++i) {
                int index = cluster.indices[i];
                points_to_fit.row(i) << clustered_cloud_stem->points[index].normal_x,
                                        clustered_cloud_stem->points[index].normal_y,
                                        clustered_cloud_stem->points[index].z;
            }




            // Fit a circle on this points
            Circle circle = CircleFitting_3D(points_to_fit);

            
            if(viewmode==2){
            /*
            Start of cylinder (made of a set of vertically aligned circles) visualizer
            */
                // See if the result is a valid circle
                
                if (circle.s <= Circle::MSE_MAX && circle.r <0.4) {


                    // is there a way to add a matrix of points to point cloud without for loop ?
                    // read this https://www.jianshu.com/p/5b0a81855a47


                    // Here we further store the circle into a data structure : stem
                    //clustered_cloud->points.push_back(circle.visualizer())
                    //  The clustered_cloud_circle now is containing several clusters of points, each cluster stand for a circle
                    // clustered_cloud_circle is ready for circle fitting 

                    single_circle_cloud->clear();
                    // Create a point cloud of that fitted circle 
                    for (int i = 0; i < 360; i++){
                        double angle = i * Pi / 180.0;
                        double point_x = circle.Px + circle.r * std::cos(angle);
                        double point_y = circle.Py + circle.r * std::sin(angle);

                        // Create a new point with assigned color ( each stem will have one exact color in each scan)
                        pcl::PointXYZRGB point;
                        point.x = point_x;
                        point.y = point_y;
                        point.z = circle.Pz;
                        point.rgb = *reinterpret_cast<float *>(&rgb_value); // Set RGB value
                        single_circle_cloud->points.push_back(point);//

                    }
                    cylinder_cloud->insert(cylinder_cloud->end(), single_circle_cloud->begin(), single_circle_cloud->end());
                    stem_candidate.pushCircle(std::move(circle));
                    // TODO: update the variance of radius, mse, and center(center may not helpful , when taking tilt trees into consideration )
                    // it is critical to be clear with, wether we want is to be able to recognize more trees or, 
                    // to recognize more reliable but meanwhiles less trees,
                    // if we use ICP, which of the two decisions benifits us more?

                }


                // Insert the points from single_circle_cloud into clustered_cloud for display
                if(stem_candidate.num_circle > 6){
                    clustered_cloud->insert(clustered_cloud->end(), cylinder_cloud->begin(), cylinder_cloud->end());            
                }
                
            }
            
            /*
            End of cylinder (made of circles)  visualizer
            */


           if(viewmode==0){
            // No visulizer
                if (circle.s <= Circle::MSE_MAX && circle.r <0.4) {
                    stem_candidate.pushCircle(std::move(circle));
                }
            }


        }

        if(stem_candidate.num_circle > 6)//6
            {
            /*
            Using derivation/variance of radii , and min_num of circles in cluester as threshold/criterien to valid stems, 
            To Do: add radii criterion 
            Could averagee/variance of mse also be benificial ?
            */  
            stem_candidate.get_averages();
            stem_candidate.get_standard_deviation();
            //stem_candidate.print(); // print stem infos
            single_scan.tree_infos_.push_back(stem_candidate.Get_tree_info());
            // single_scan.tree_infos_ stores the info of stems in the current scan. 


            ///  create a vector which holds the info of stems inside a gloabl map,
            ///  implement two functions  that create pointcloud type to vector and vise versa
            ///   
            }
    
    }



    void stemClustering()
    /*    
    This function is used to cluster points into groups, where points each of them likely forms a stem, from the whole pointcloud
     which is recognized as stems.
    TODO: add a tag here , so it is allowed to control whetehr the colored pointcloud should be generated and published for validation.
     */
    {

        // Clear the point cloud before clustering
        clustered_cloud->clear();
        single_scan.tree_infos_.clear();
        single_scan.time_stamp_ = cloudHeader.stamp.toSec();
        // cylinder_cloud->clear();
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        // Now cluster pointcloud into seperate trunks first

        ec.setClusterTolerance(0.1); // aug 4 = 0.3
        ec.setMinClusterSize(10);    // aug4 = 20
        ec.setMaxClusterSize(1000);  //   60 was good.

        ec.setInputCloud(nonGroundCloud); //
        cluster_indices_stem.clear();
        ec.extract(cluster_indices_stem);

        for (const pcl::PointIndices &cluster : cluster_indices_stem)
        {
            
            pcl::PointXYZLNormal point;
            // Clear the point cloud before clustering
            clustered_cloud_stem->clear();

            for (const int index : cluster.indices)
            {
                // Create a new point without color
                point.normal_x = nonGroundCloud->points[index].x;
                point.normal_y = nonGroundCloud->points[index].y;
                // point.h = 0;
                // point.s = 0;
                point.x = 0;
                point.y = 0;
                point.z = nonGroundCloud->points[index].intensity;
                clustered_cloud_stem->points.push_back(point);
            }

            // here we have a point cloud named : clustered_cloud_stem filled with points belong to one stem
            /**** Do another clustering on point , to get clusters of circle  in each cluster ****/

            circleClustering();  // Returns  cluster_indices
            //  Here we further store the stems into our structe  : frame or scan
        }

        clustered_cloud->width = clustered_cloud->size();
        clustered_cloud->height = 1;
        clustered_cloud->is_dense = true;

        if(single_scan.tree_infos_.size() > MIN_STEM_NUM ){
            //std::cout<< "\n single_scan.tree_infos\n"<<single_scan.tree_infos_.size();
            scans.push_back(single_scan);// optimizing with std::move is possible here
            
            std::cout<< "\n scans 's size\n"<<scans.size()<<std::endl;
            //std::cout<< "\n last single scan 's size\n"<<scans.back().tree_infos_.size()<<std::endl;
            // illegal visiting of back() will cause the funtion  to be out  of work

            /*********************** 20 Nov **************************/

            double timestamp = single_scan.time_stamp_;
            std::vector<Eigen::Vector3d> scan_data = readXYFromSingleScan(single_scan);
            std::vector<Eigen::Vector3d> scan_data_with_radius = readXYRFromSingleScan(single_scan);

            if(!icp_2d.isSourceSet()) {
                icp_2d.SetSource(scan_data);
                return;
            }
            icp_2d.SetTarget(scan_data);
            // if pose estimation failed:
            if (!icp_2d.pose_estimation_3d3d()) {
                std::cout<<"*********************icp matching failed!**************** "<< failure_num++<<std::endl;
                return;
            }
            Mat3d R = icp_2d.Get_Odometry().R_;
            Vec3d t = icp_2d.Get_Odometry().p_;
            if(!t.hasNaN()&& !R.hasNaN()) {
                // because the swaped use of target and source in the code : bfnn
                // the t and R is the transformation for aligning the older scan to the newst scan.
                path_accumulated -= t;
                Rotation_accumulated = R.inverse() *  Rotation_accumulated;
                // Save the translation between consective poses
                translation_pose2pose.push_back(MovementData(timestamp, R, 
                Vec3d::Zero(), t));
                // Save the translation from the current pose to the origin
                odometry.push_back(MovementData(timestamp, Rotation_accumulated, 
                Vec3d::Zero(), path_accumulated));
                
                // Maintain the global-, local- stem map:
                int idx = 0;
                for(Eigen::Vector3d  point: scan_data_with_radius) {
                    Eigen::Vector3d frame_in_global = Rotation_accumulated * point + path_accumulated;

                    Vec6d frame_in_global_time_idx_clusteridx;
                    frame_in_global_time_idx_clusteridx<<frame_in_global(0),
                    frame_in_global(1),
                    frame_in_global(2),
                    timestamp,
                    idx,
                    0.;
                    global_map.push_back(frame_in_global_time_idx_clusteridx);

                    // Updated at 17 Nov
                    Vec6d frame_in_local_time_idx_clusteridx;
                    frame_in_local_time_idx_clusteridx<<point(0),
                    point(1),
                    point(2),
                    timestamp,
                    idx,
                    0.;
                    local_map.push_back(frame_in_local_time_idx_clusteridx);

                    idx++;
                }
                icp_2d.SetSource(scan_data);//oct 18
            } else {
                std::cout<<"*********************icp matching failed!**************** "<< failure_num++<<std::endl;
            }
        }       
    }


            /************************* End 20 Nov ************************/
        

        // After a sincle_scan is generated: ( which has to have the infos like a single_scan.txt does):

        // 1. render( reconstruct the data structe to fit with the _____.cpp)
        // 2. run scan to scan ICP 
        // 3. maitain a global stem map, local stem map, odometry and pose2posetranslation data structures.
        // 4. if condition to run graph optimization is met, construct the pose graph and solve

    /*************************  20 Nov ************************/
    void runPoseGraphOptimization(){
        //std::cout << "runPoseGraphOptimization has been called"<<endl;
        if(pose_i % 3==0 || pose_i==407){
        // Run in every 10 scan
            const char* homeDir = getenv("HOME");
            std::string fullPath = std::string(homeDir) + "/catkin_ws_aug/src/shapefitting/g2o_result/"
                                + "single_scan_" + std::to_string(pose_i) + ".g2o";
            std::cout<< "In iter: "<<pose_i<<endl;
            //Reusing it by  just reseting all vertices and edges at here 
            poseGraphBuilder.clear_edges_vertices();

            //With/out robust kernel
            double kernelwidth_SE2 = 5.0;   // ori. 1
            double kernelwidth_SE2XY = 5.0; // ori. 5
            bool use_kernel = true;
            bool save_g2o = false;
            poseGraphBuilder.build_optimize_Graph(global_map, local_map, odometry, translation_pose2pose,use_kernel,kernelwidth_SE2,kernelwidth_SE2XY);

            // Save a .g2o file to local for inspection
            if(pose_i % 3==0 || pose_i==407&&save_g2o==true){
            // Run in every 50 scan
                poseGraphBuilder.saveGraph(fullPath);
                std::cout << "Pose graph optimization completed." << std::endl;
            }
            
        }
        ++pose_i;

        
        // Visualization of the trajectory and landmarks
        nav_msgs::Path path;
        visualization_msgs::MarkerArray marker_array;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "world";  // Adjust the frame_id as per your setup

        for (auto it = poseGraphBuilder.optimizer_.vertices().begin(); it != poseGraphBuilder.optimizer_.vertices().end(); ++it) {
            g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);


            if (auto pose_vertex = dynamic_cast<g2o::VertexSE2*>(v)) {
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header = path.header;

                // Extracting the pose from the vertex
                pose_stamped.pose.position.x = pose_vertex->estimate().translation().x();
                pose_stamped.pose.position.y = pose_vertex->estimate().translation().y();
                pose_stamped.pose.position.z = 0; // For 2D, z is typically 0

                tf::Quaternion q;
                q.setRPY(0, 0, pose_vertex->estimate().rotation().angle());
                tf::quaternionTFToMsg(q, pose_stamped.pose.orientation);

                path.poses.push_back(pose_stamped);
            }

            if (auto landmark_vertex = dynamic_cast<g2o::VertexPointXY*>(v)) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "world";  // or your relevant frame
                marker.header.stamp = ros::Time::now();
                marker.ns = "landmarks";
                marker.id = landmark_vertex->id(); // Unique ID for the landmark
                marker.type = visualization_msgs::Marker::CYLINDER; // Marker's shape
                marker.action = visualization_msgs::Marker::ADD;
                
                marker.pose.position.x = landmark_vertex->estimate()[0];
                marker.pose.position.y = landmark_vertex->estimate()[1];
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0; // Identity quaternion

                marker.scale.x = 0.5; // Size of the marker
                marker.scale.y = 0.5;
                marker.scale.z = 2;

                marker.color.a = 1.0; // alpha
                marker.color.r = 1.0; // Red color
                marker.color.g = 0.0;
                marker.color.b = 0.0;

                marker_array.markers.push_back(marker);
            }
        }

        pubPath.publish(path);
        pubLandmarks.publish(marker_array);

    }
    /*************************  End 20 Nov ************************/
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
        runPoseGraphOptimization();
        //projectTo3D();
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
        // process the message only after 3 messages have been received
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

    IP.saveToFiles();
    return 0;
}
