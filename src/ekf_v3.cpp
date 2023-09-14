
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


/// Define a struct to store IMU data
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
    ///  State: x = [sx,sy,v_,sphi]
    ///  Control input: u = [angular velocity , linear acceleration]
    ///  Observation y = [sx,sy]
    using MotionNoise = Eigen::Matrix<float, 2, 2>;  // 运动噪声类型
    using UTMNoise = Eigen::Matrix<float, 2, 2>;  // observation 噪声类型
    using Error_cov = Eigen::Matrix<float, 4, 4>;//error_covariance
    using Mat4f = Eigen::Matrix<float, 4, 4>;  
    using Mat2f = Eigen::Matrix<float, 2, 2>; 
    using Observation = Eigen::Matrix<float,2,4>;
    using Gain = Eigen::Matrix<float,4,2>;


    /// Methods
    void Predict(const sensor_msgs::Imu::ConstPtr &msg_in);
    void Update(const sensor_msgs::NavSatFix::ConstPtr& msg_in);
    /// Helper methods
    IMUData Get_IMU () const{ 
    return IMUData(current_time_, 
                   Sophus::SO3(Eigen::Matrix3d::Identity()), 
                   Eigen::Vector3d(v_ *cos(sphi_), v_ *sin(sphi_), 0), 
                   Eigen::Vector3d(sx_, sy_, 0));
}


private:
    MotionNoise Q_ = MotionNoise::Zero();
    UTMNoise V_ = UTMNoise::Zero();
    Error_cov P_ = Error_cov::Identity() * 1e-2;
    double current_time_ = 0.0;  
    /// Flags
    bool first_imu = true;
    bool first_utm = true;
    bool initilized_imu = false; //  
    Eigen::Vector2f utm_origin = Eigen::Vector2f::Zero();
    /// States:
    float sx_ =0;
    float sy_ =0;
    float v_ =0;
    float sphi_ =0;

    /// IMU initilization
    float accumu_acce =0.0, accumu_angu =0.0;
    float bias_acc = 0.0,  bias_ang = 0.0;
    int  record_time = 0;
};

void EKF::Predict(const sensor_msgs::Imu::ConstPtr &msg_in) {


    /// Initialize imu , estimate the imu's bias
    if(!initilized_imu) {
        
        accumu_acce +=msg_in->linear_acceleration.x;
        accumu_angu +=msg_in->angular_velocity.z;
        record_time++;
        std::cout<<"Initializing IMU... "<<record_time<<"\n";
        /// Init the time stamp
        if(first_imu == true) {
        current_time_ = msg_in->header.stamp.toSec();
        first_imu = false;
        return;
        }
        if(msg_in->header.stamp.toSec() - current_time_ > 2){ // was >3
            bias_acc = accumu_acce/record_time;
            bias_ang = accumu_angu/record_time;
            current_time_ = msg_in->header.stamp.toSec();
            initilized_imu = true;
            std::cout<<"\n\nIMU initilized:\n"<<"bias_acc: "<< bias_acc<<"      bias_ang: "<<bias_ang<<std::endl;
        }
        return;
    }


    /// Check data validity
    if(msg_in->header.stamp.toSec() < current_time_) return;

    /// Get dt and update current_time_
    double dt = msg_in->header.stamp.toSec() - current_time_;
    current_time_ = msg_in->header.stamp.toSec();
    /// Check dt's validity , not predict when dt is larger than 0.1s
    if(dt >0.1) return;
    Eigen::Vector3d angular_velocity(msg_in->angular_velocity.x,
                                     msg_in->angular_velocity.y,
                                     msg_in->angular_velocity.z - bias_ang);

    Eigen::Vector3d linear_acceleration(msg_in->linear_acceleration.x - bias_acc,
                                       msg_in->linear_acceleration.y,
                                       msg_in->linear_acceleration.z );
    /// When the orientation of IMU has to be condisred:

    // Sophus::SO3 rot = Sophus::SO3 (Eigen::Quaterniond(msg_in->orientation.w,msg_in->orientation.x,msg_in->orientation.y,msg_in->orientation.z));
    // Eigen::Vector3d angular_velocity_world = rot*angular_velocity;
    // Eigen::Vector3d linear_velocity_world = rot*linear_acceleration;
    // float wphi = angular_velocity_world(2);
    // float a = linear_velocity_world(0);

    /// Ohterwise :
    float wphi = angular_velocity[2];
    float a = linear_acceleration[0]; 

    /// Update the states
    sx_ += dt* v_*cos(sphi_);
    sy_ += dt* v_*sin(sphi_);
    sphi_ += dt * wphi;
    v_ += dt *a;

    /// Update the error covariancce P_
    Mat4f F = Mat4f::Identity() ;
    F(0,2) = - a*sin(sphi_)*dt;
    F(0,3) =     cos(sphi_)*dt;
    F(1,2) =   a*cos(sphi_)*dt;
    F(1,3) =     sin(sphi_)*dt;

    Q_.diagonal() <<  1e-2,  1e-3 ;
    Eigen::Matrix<float, 4, 2> G_k = Eigen::Matrix<float, 4, 2>::Zero();
    G_k.template block<2,2>(2,0) = Eigen::Matrix<float, 2, 2>::Identity() * dt;
    Error_cov Q_k = G_k * Q_ * G_k.transpose();
    P_ = F * P_ * F.transpose() + Q_k;
}

void EKF::Update(const sensor_msgs::NavSatFix::ConstPtr& msg_in){
    /// Check data validity
    if (msg_in->header.stamp.toSec() < current_time_) {
        std::cout<<"/n******Msg header time stamp:  "<<msg_in->header.stamp.toSec() <<"/n"
        <<"while the current time of the system is : "<< current_time_<<"*****"<<std::endl;
        return;
        }
    
    if( initilized_imu ) current_time_=msg_in->header.stamp.toSec();
    /// Process the GPS data as required
    double  latitude = msg_in->latitude;
    double  longitude = msg_in->longitude;
    double  altitude = msg_in->altitude;
    double UTMNorthing,UTMEasting;
    char UTMZone[4];
    LLtoUTM(latitude, longitude, UTMNorthing, UTMEasting, UTMZone);

    /// Substract the first utm coordinates to centralize the coordinates
    if(first_utm == true){
        utm_origin[0] = UTMEasting;
        utm_origin[1] = UTMNorthing;
        first_utm = false; 
        return;
    }
    Eigen::Vector2f observation =  Eigen::Vector2f(UTMEasting, UTMNorthing) -utm_origin;

    std::cout<<"UTM\n"<<std::endl;
    std::cout<<observation[0]<<std::endl;
    std::cout<<observation[1]<<std::endl;

    
    if(!initilized_imu) return;
    /// Update( Correct) the prediction and Update the error covariance matrix P_
    Observation H = Observation::Zero();
    H.template block<2,2>(0,0) = Mat2f::Identity();
    V_.diagonal()<< 0.3, 0.3;
    //V_.diagonal()<< 0, 0;
    Gain K = P_ * H.transpose() * (H * P_ * H.transpose() + V_ ).inverse();

    MatrixXf estimate(4,1);
    estimate << sx_ , sy_ , sphi_, v_;
    std::cout<< "before update v_"<< v_<<std::endl;
    std::cout<< "before update sx_ , sy_"<< sx_<<", "<<sy_<<std::endl;
    Eigen::Vector2f innovation = observation - estimate.template block<2,1>(0,0);
    std::cout<< "innovation\n"<< innovation<<", "<<std::endl;
    std::cout<< "kalman Gain\n"<< K<<", "<<std::endl;
    estimate += K*innovation;

    sx_  = estimate(0,0);
    sy_  = estimate(1,0);
    sphi_  = estimate(2,0);
    v_  = estimate(3,0);
    P_ = ( Mat4f::Identity()- K * H) * P_;
    std::cout<< "after update v_ "<< v_<<std::endl;
    std::cout<< "after update sx_ , sy_ "<< sx_<<", "<<sy_<<std::endl;
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
    //imu_data.push_back(ekf.Get_IMU());
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
    /// Modify the location where the file will be saved as needed
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

