
#include "sophus/so3.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iomanip>
#include <fstream>
#include <sensor_msgs/NavSatFix.h>
#include "utilities/toUTM.hpp"
using namespace Eigen;


// Define a struct to store IMU data
struct IMUData {
    double timestamp_;
    Sophus::SO3 R_= Sophus::SO3(Eigen::Matrix3d::Identity());
    Eigen::Vector3d v_;
    Eigen::Vector3d p_;

    IMUData() 
        : timestamp_(0.0), v_(Eigen::Vector3d::Zero()), p_(Eigen::Vector3d::Zero()) {} 
    
    IMUData(double timestamp, 
            const Sophus::SO3 &rotation_matrix = Sophus::SO3(Eigen::Matrix3d::Identity()),
            const Eigen::Vector3d &velocity = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d &position = Eigen::Vector3d::Zero())
        : timestamp_(timestamp),  v_(velocity), p_(position),R_(rotation_matrix) {}
};

class EKF {

    public:
    ///  state: x = [sx,sy,vx,vy,sphi]
    using MotionNoise = Eigen::Matrix<float, 5, 5>;  // 运动噪声类型
    using UTMNoise = Eigen::Matrix<float, 2, 2>;  // observation 噪声类型
    using Error_cov = Eigen::Matrix<float, 5, 5>;//error_covariance
    using Mat5f = Eigen::Matrix<float, 5, 5>;  // 运动噪声类型
    using Mat2f = Eigen::Matrix<float, 2, 2>; 
    using Observation = Eigen::Matrix<float,2,5>;
    using Gain = Eigen::Matrix<float,5,2>;

    //eigen::Matrix  do not save F

    void Predict(const sensor_msgs::Imu::ConstPtr &msg_in);
    void Update(const sensor_msgs::NavSatFix::ConstPtr& msg_in);

    IMUData Get_IMU () const{ 
    return IMUData(current_time_, 
                   Sophus::SO3(Eigen::Matrix3d::Identity()), 
                   Eigen::Vector3d(vx_, vy_, 0), 
                   Eigen::Vector3d(sx_, sy_, 0));
}

    private:

    MotionNoise Q_ = MotionNoise::Zero();
    UTMNoise V_ = UTMNoise::Zero();
    Error_cov P_ = Error_cov::Identity() * 1e-4;
    double current_time_ = 0.0;  // 当前时间
    /// Flags
    bool first_imu = true;
    bool initilized = false; //  not use it for now
    bool first_utm = true;
    Eigen::Vector2f utm_origin = Eigen::Vector2f::Zero();
    /// states:
    float sx_ =0;
    float sy_ =0;
    float vx_ =0;
    float vy_ =0;
    float sphi_ =0;
};

void EKF::Predict(const sensor_msgs::Imu::ConstPtr &msg_in) {

    
    if(first_imu == true) {
        current_time_ = msg_in->header.stamp.toSec();
        first_imu = false;
        return;
    }
    //if(first_utm) return;

    double dt = msg_in->header.stamp.toSec() - current_time_;
    current_time_ = msg_in->header.stamp.toSec();

    Eigen::Vector3d angular_velocity(msg_in->angular_velocity.x,
                                     msg_in->angular_velocity.y,
                                     msg_in->angular_velocity.z);

    Eigen::Vector3d linear_acceleration(msg_in->linear_acceleration.x,
                                       msg_in->linear_acceleration.y,
                                       msg_in->linear_acceleration.z);

    // Sophus::SO3 rot = Sophus::SO3 (Eigen::Quaterniond(msg_in->orientation.w,msg_in->orientation.x,msg_in->orientation.y,msg_in->orientation.z));

    // Eigen::Vector3d angular_velocity_world = rot*angular_velocity;
    // Eigen::Vector3d linear_velocity_world = rot*linear_acceleration;

    // float wphi = angular_velocity_world(2);
    // float a = linear_velocity_world(0);

    float wphi = msg_in->angular_velocity.z;
    float a = msg_in->linear_acceleration.x;

    //std::cout<<"linear acceleration a :"<<a <<"\n";

    if (dt > 0 && dt < 0.1) {

        sx_ += dt* vx_;
        sy_ += dt* vy_;
        vx_ += dt* a* cos(sphi_);
        vy_ += dt* a* sin(sphi_);
        sphi_ += dt * wphi;
    }


    ///  update the error covariancce P_
    Mat5f F = Mat5f::Zero();
    // 0 0 1 | 0            0  | 
    // 0 0 0 | 1            0  | 
    // 0 0 0 | 0 -a*sin(sphi)  |
    // 0 0 0 | 0  a*cos(sphi)  |
    // 0 0 0 | 0            0  | 
    F.template block<2, 2>(0, 2) = Mat2f::Identity() ;
    F(2,4) = - a*sin(sphi_);
    F(3,4) =   a*cos(sphi_);
    //Q_.diagonal() << 0.1, 0.1, 0.01,0.01,0.05;
    Q_.diagonal() <<  1e-2,  1e-2,  1e-3, 1e-3, 1e-1;
    P_ = F * P_ * F.transpose() + Q_;

    //std::cout<<"\n"<< P_ <<"\n\n";
}

void EKF::Update(const sensor_msgs::NavSatFix::ConstPtr& msg_in){

    if (msg_in->header.stamp.toSec() < current_time_) return;
    current_time_=msg_in->header.stamp.toSec();

    double  latitude = msg_in->latitude;
    double  longitude = msg_in->longitude;
    double  altitude = msg_in->altitude;
    double UTMNorthing,UTMEasting;

    // Process the GPS data as required


    char UTMZone[4];
    LLtoUTM(latitude, longitude, UTMNorthing, UTMEasting, UTMZone);

    /// 去掉原点
    if(first_utm == true){
        utm_origin[0] = UTMNorthing;
        utm_origin[1] = UTMEasting;
        first_utm = false; 
        return;
    }

    Eigen::Vector2f observation =  Eigen::Vector2f(UTMNorthing,UTMEasting) -utm_origin;




    std::cout<<"UTM\n"<<std::endl;
    std::cout<<observation[0]<<std::endl;
    std::cout<<observation[1]<<std::endl;


    Observation H = Observation::Zero();
    H.template block<2,2>(0,0) = Mat2f::Identity();
    V_.diagonal()<< 0, 0;
    //V_.diagonal()<< 0, 0;
    Gain K = P_ * H.transpose() * (H * P_ * H.transpose() + V_ ).inverse();

    MatrixXf estimate(5,1);
    estimate << sx_ , sy_ , vx_, vy_, sphi_;
    std::cout<< "before update"<< vx_<<", "<<vy_<<std::endl;
    std::cout<< "before update"<< sx_<<", "<<sy_<<std::endl;
    Eigen::Vector2f innovation = observation - estimate.template block<2,1>(0,0);
    std::cout<< "innovation"<< innovation<<", "<<std::endl;
    std::cout<< "kalman Gain"<< K<<", "<<std::endl;
    estimate += K*innovation;

    sx_  = estimate(0,0);
    sy_  = estimate(1,0);
    vx_  = estimate(2,0);
    vy_  = estimate(3,0);
    sphi_= estimate(4,0);
    P_ = ( Mat5f::Identity()- K * H) * P_;
    std::cout<< "after update"<< vx_<<", "<<vy_<<std::endl;
    std::cout<< "after update"<< sx_<<", "<<sy_<<std::endl;
}

std::vector<IMUData> imu_data; // Store IMU data
EKF  ekf;
ros::Publisher path_pub; // Publisher for the path









void imuCallback(const sensor_msgs::Imu::ConstPtr &msg_in) {

    ekf.Predict(msg_in);
    imu_data.push_back(ekf.Get_IMU());
}

void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg_in) {
    ekf.Update(msg_in);
}

int main(int argc, char **argv) {




    ros::init(argc, argv, "imu_integal");
    ros::NodeHandle nh;
    ros::Subscriber subGPS = nh.subscribe("/lernsmart/isobus_read/NavSatFix_in", 10000, GPSCallback);
    ros::Subscriber sub = nh.subscribe("/imu/data", 10000, imuCallback);

    
    // Initialize the path publisher
    path_pub = nh.advertise<nav_msgs::Path>("imu_path", 1000);

    ros::spin();

    // Save IMU data to a text file

        // Get user's home directory
    const char* homeDir = getenv("HOME");
    if (!homeDir) {
        std::cerr << "Error: HOME environment variable not set." << std::endl;
        return 1;
    }

    // Construct the desired path
    std::string fullPath = std::string(homeDir) + "/slam_in_autonomous_driving/data/ch3/velodyne_bag_txt/state_imu_2d.txt";

    // Open the file for writing
    std::ofstream fout(fullPath);
    if (!fout) {
        std::cerr << "Error: Unable to open file for writing." << std::endl;
        return 1;
    }


    for (const IMUData& imu_point : imu_data) {
        auto save_vec3 = [](std::ofstream& fout, const Eigen::Vector3d& v) {
            fout << v[0] << " " << v[1] << " " << v[2] << " ";
        };
        auto save_quat = [](std::ofstream& fout, const Sophus::SO3& R) {
            Eigen::Quaterniond q = R.unit_quaternion();
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << imu_point.timestamp_ << " " << std::setprecision(9);
        save_vec3(fout, imu_point.p_);
        save_quat(fout, imu_point.R_);
        save_vec3(fout, imu_point.v_);
        fout << std::endl;
    }
    std::cout<<"done"<<std::endl;

    return 0;
}

