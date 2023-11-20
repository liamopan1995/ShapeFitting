#ifndef UTILITIE
#define UTILITIE
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "sophus/se3.h"
#include "sophus/se2.h"
#include <vector>
// #include "sensor_msgs/LaserScan.h"  //  error !
using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Vec2d = Eigen::Vector2d; 
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Mat2d = Eigen::Matrix2d;
using Mat3d = Eigen::Matrix3d;
using Mat32d = Eigen::Matrix<double, 3, 2>;
// using Mat2d = Eigen::Matrix2d;
using Mat3f = Eigen::Matrix3f;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using SE3 = Sophus::SE3;
using SE2 = Sophus::SE2;
using SO2 = Sophus::SO2;
using SO3 = Sophus::SO3;
using IndexVec = std::vector<int>;

// nov 13
// using Scan2d = sensor_msgs::LaserScan; // error! 



inline Vec3d ToVec3d(const PointType& pt) { return pt.getVector3fMap().cast<double>(); }
inline Vec3d ToVec3d(const pcl::PointXYZRGB& pt) { return pt.getVector3fMap().cast<double>(); }

inline Vec3f ToVec3f(const PointType& pt) { return pt.getVector3fMap(); }
inline Vec3f ToVec3f(const pcl::PointXYZRGB& pt) { return pt.getVector3fMap(); }

template <typename S>
inline PointType ToPointType(const Eigen::Matrix<S, 3, 1>& pt) {
    PointType p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    return p;
}

// Function to convert a pcl::PointCloud to a std::vector of Eigen::Vector3d
template <typename PointT>
std::vector<Eigen::Vector3d> pointCloudToVector(const typename pcl::PointCloud<PointT>&cloud) {
    std::vector<Eigen::Vector3d> vec;
    for (const auto& point : cloud.points) {
        vec.emplace_back(static_cast<double>(point.x), 
                         static_cast<double>(point.y), 
                         static_cast<double>(point.z));
    }
    return vec;
}

// Function to convert a std::vector of Eigen::Vector3d to a pcl::PointCloud
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr vectorToPointCloud_2D(const std::vector<Eigen::Vector3d> &vec) {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    for (const auto& point : vec) {
        PointT p;
        p.x = point(0);
        p.y = point(1);
        p.z = 0;
        cloud->points.push_back(p);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    return cloud;
}

// Function to convert a std::vector of Eigen::Vector3d to a pcl::PointCloud
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr vectorToPointCloud(const std::vector<Eigen::Vector3d> &vec) {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    for (const auto& point : vec) {
        PointT p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);
        cloud->points.push_back(p);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    return cloud;
}



struct MovementData {
    double timestamp_;
    Mat3d R_= Mat3d::Identity();
    Eigen::Vector3d v_;
    Eigen::Vector3d p_; //x y z

    MovementData() 
        : timestamp_(0.0), v_(Eigen::Vector3d::Zero()), p_(Eigen::Vector3d::Zero()) {} 
    
    MovementData(double timestamp, 
            const Mat3d &rotation_matrix = Mat3d::Identity(),
            const Eigen::Vector3d &velocity = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d &position = Eigen::Vector3d::Zero())
        : timestamp_(timestamp),  v_(velocity), p_(position),R_(rotation_matrix) {}

    MovementData(double timestamp, 
            const Mat2d &rotation_matrix ,
            const Eigen::Vector3d &velocity,
            const Eigen::Vector2d &position )
            : timestamp_(timestamp), v_(velocity),p_(Eigen::Vector3d::Zero()),R_(Mat3d::Identity()){
 
            R_.block<2, 2>(0, 0) = rotation_matrix;
            p_ << position.x(), position.y(), 0.0;
        }
};



#endif