
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
#include <cmath>
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
    ///  State: x = [sx,sy,sphi,v_]
    ///  Control input: u = [angular velocity , linear acceleration]
    ///  Observation y = [sx,sy]
    using Process_cov = Eigen::Matrix<float, 2, 2>; // control covariance 
    using Observation_cov = Eigen::Matrix<float, 3, 3>;    // observation covariance 
    using State_cov = Eigen::Matrix<float, 4, 4>;   // error_covariance
    using Mat4f = Eigen::Matrix<float, 4, 4>;       // state covariance 
    using Mat3f = Eigen::Matrix<float, 3, 3>; 
    using Mat2f = Eigen::Matrix<float, 2, 2>;       // F ( df/dx)
    using Measurement = Eigen::Matrix<float,3,4>;   // H
    using Kalman_Gain = Eigen::Matrix<float,4,3>;          // K

public:

    std::vector< Eigen::Matrix<double, 1,5>> UTM;
    std::vector<Eigen::Vector3d> control_input;
    Process_cov Q_ = Process_cov::Zero();// Process Covariance
    Observation_cov R_ = Observation_cov::Zero();// Measurement Covariance
    State_cov P_ = State_cov::Identity() * 1e-3;// State Covariance

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

    double initial_time_ = 0.0;
    double current_time_ = 0.0;  
    double dt_ =0.0;
    double last_utm_time_ = 0.0;
    double utm_time_diff_ = 0.0;
    /// Flags
    bool first_imu = true;
    bool first_utm = true;
    bool initilized_imu = false; 
    
    /// States:
    float sx_ =0;
    float sy_ =0;
    float v_ =0;
    float sphi_ =0;

    /// IMU initilization
    float accumu_acce =0.0, accumu_angu =0.0;// what?
    float bias_acc = 0.0,  bias_ang = 0.0; // bias of IMU measurements
    int  record_time = 0;

    /// UTM 
    Eigen::Vector2f utm_origin = Eigen::Vector2f::Zero();
    Eigen::Vector2f utm_last = Eigen::Vector2f::Zero();
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
        initial_time_ = msg_in->header.stamp.toSec();
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

    /// Get dt_ and update current_time_
    dt_ = msg_in->header.stamp.toSec() - current_time_;
    current_time_ = msg_in->header.stamp.toSec();
    /// Check dt_'s validity , not predict when dt_ is larger than 0.1s
    if(dt_ >0.1) return;
    Eigen::Vector3d angular_velocity(msg_in->angular_velocity.x,
                                     msg_in->angular_velocity.y,
                                     msg_in->angular_velocity.z - bias_ang);

    Eigen::Vector3d linear_acceleration(msg_in->linear_acceleration.x - bias_acc,
                                       msg_in->linear_acceleration.y,
                                       msg_in->linear_acceleration.z );


    // Sophus::SO3 rot = Sophus::SO3 (Eigen::Quaterniond(msg_in->orientation.w,msg_in->orientation.x,msg_in->orientation.y,msg_in->orientation.z));
    // Eigen::Vector3d angular_velocity_world = rot*angular_velocity;
    // Eigen::Vector3d linear_velocity_world = rot*linear_acceleration;
    // float wphi = angular_velocity_world(2);
    // float a = linear_velocity_world(0);

    /// Ohterwise :
    float wphi = angular_velocity[2];
    float a = linear_acceleration[0]; 
    control_input.push_back(Eigen::Vector3d(msg_in->header.stamp.toSec()-initial_time_,a,wphi));
    /// Update the states
    sx_ += dt_* v_*cos(sphi_);
    sy_ += dt_* v_*sin(sphi_);
    // sx_ += dt_* v_*cos(sphi_);// oct. 6th
    // sy_ += dt_* v_*sin(sphi_);
    sphi_ += dt_ * wphi;
    v_ += dt_ *a;

    /// Update the state covariancce P_
    Mat4f F = Mat4f::Identity() ;
    F(0,2) = - a*sin(sphi_)*dt_;
    F(0,3) =     cos(sphi_)*dt_;
    F(1,2) =   a*cos(sphi_)*dt_;
    F(1,3) =     sin(sphi_)*dt_;
    //Q_.diagonal() <<  1e-2,  1e-3 ;
    Eigen::Matrix<float, 4, 2> G_k = Eigen::Matrix<float, 4, 2>::Zero();
    G_k.template block<2,2>(2,0) = Eigen::Matrix<float, 2, 2>::Identity() * dt_;
    State_cov Q_k = G_k * Q_ * G_k.transpose();
    P_ = F * P_ * F.transpose() + Q_k;
}

void EKF::Update(const sensor_msgs::NavSatFix::ConstPtr& msg_in){
    /// Check data validity
    if (msg_in->header.stamp.toSec() < current_time_) {
        std::cout<<"/n******Msg header time stamp:  "<<msg_in->header.stamp.toSec() <<"/n"
        <<"while the current time of the system is : "<< current_time_<<"*****"<<std::endl;
        return;
        }
    
    if( initilized_imu ) current_time_ = msg_in->header.stamp.toSec();
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
        utm_last[0] = UTMEasting;
        utm_last[1] = UTMNorthing;
        last_utm_time_ =  msg_in->header.stamp.toSec();
        first_utm = false; 
        return;
    }
    ///  I think here lacks a condition to check if the newst utm_time stamp is near to the time stamp
    ///  of the lastest IMU message
    utm_time_diff_ = msg_in->header.stamp.toSec() - last_utm_time_;
    last_utm_time_ = msg_in->header.stamp.toSec();

    Eigen::Vector2f observation_2d =  Eigen::Vector2f(UTMEasting, UTMNorthing) -utm_origin;
    float measurement_v = (observation_2d - utm_last).norm() / utm_time_diff_;
    float measurement_theta = atan2(observation_2d[0]-utm_last[0], observation_2d[1]-utm_last[1]);

    utm_last = observation_2d;
    Eigen::Vector3f observation_3d = Eigen::Vector3f(observation_2d[0],observation_2d[1],measurement_v);

    
    Eigen::Matrix<double, 1,5> UTM_row;
    UTM_row << msg_in->header.stamp.toSec()-initial_time_,observation_3d[0],observation_3d[1],observation_3d[2], measurement_theta;
    UTM.push_back(UTM_row);
    std::cout<<"UTM\n"<<observation_3d[0]<<"\t"<<observation_3d[1]<<"\t"<<observation_3d[2]<<std::endl;
    
    if(!initilized_imu) return;
    /// Update( Correct) the prediction and Update the error covariance matrix P_
    Measurement H = Measurement::Zero();
    H.template block<2,2>(0,0) = Mat2f::Identity();
    H(2,3) = 1;

    //R_.diagonal()<< 0.3, 0.3 ,0.5;

    Kalman_Gain K = P_ * H.transpose() * (H * P_ * H.transpose() + R_ ).inverse();

    MatrixXf estimate(4,1);
    estimate << sx_ , sy_ ,sphi_,v_;
    std::cout<< "before update v_"<< v_<< "\nbefore update sx_ , sy_"<< sx_<<", "<<sy_<<std::endl;

    // Measurement function : H = [ [1,0,0,0] , [0,1,0,0] ]
    // Revision : derive a velo observation from utm's travel in last time step ie.:
    // (UTM current - UTM last) / dt_

    Eigen::Vector3f innovation = observation_3d - Eigen::Vector3f (estimate(0,0),estimate(1,0),estimate(3,0));
    //std::cout<< "innovation\n"<< innovation<<", \nkalman Gain\n"<< K<<", "<<std::endl;
    estimate += K*innovation;

    sx_  = estimate(0,0);
    sy_  = estimate(1,0);
    sphi_  = estimate(2,0);
    v_  = estimate(3,0);
    P_ = ( Mat4f::Identity()- K * H) * P_;
    std::cout<< "after update v_ "<< v_<< "\nafter update sx_ , sy_ "<< sx_<<", "<<sy_<<std::endl;
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
    using Process_cov = Eigen::Matrix<float, 2, 2>; // control covariance 
    using Observation_cov = Eigen::Matrix<float, 3, 3>;    // observation covariance 
    using State_cov = Eigen::Matrix<float, 4, 4>;   // error_covariance

    Eigen::Vector2f process_covariance(0.01, 0.02); // Vector with diagonal values
    Eigen::Vector3f observation_covariance(3, 3, 0); // Vector with diagonal values
    ekf.Q_ = process_covariance.asDiagonal();
    // ekf.R_ = Observation_cov::Zero();
    ekf.R_ = observation_covariance.asDiagonal();
    ekf.P_ = State_cov::Identity() * 1e-3;

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

    // Save UTM to /slam_in_autonomous_driving/data/ch3/velodyne_bag_txt/utm.txt
    std::ofstream utm_file( std::string(homeDir) +"/slam_in_autonomous_driving/data/ch3/velodyne_bag_txt/utm_measurement.txt");
    if (utm_file.is_open()) {
        for (const auto &row : ekf.UTM) {
            for (int i = 0; i < row.size(); i++) {
                utm_file << row(i);
                if (i < row.size() - 1) utm_file << " "; // Separate the columns by spaces
            }
            utm_file << "\n";
        }
        utm_file.close();
    } else {
        std::cerr << "Error: Unable to open UTM file for writing." << std::endl;
    }

    // Save control_input to /slam_in_autonomous_driving/data/ch3/velodyne_bag_txt/control_inputs.txt
    std::ofstream control_input_file( std::string(homeDir) +"/slam_in_autonomous_driving/data/ch3/velodyne_bag_txt/control_inputs.txt");
    if (control_input_file.is_open()) {
        for (const auto &vector : ekf.control_input) {
            control_input_file << vector(0) << " " << vector(1) << " " << vector(2) << "\n";
        }
        control_input_file.close();
    } else {
        std::cerr << "Error: Unable to open control_input file for writing." << std::endl;
    }

    std::cout<<"done"<<std::endl;

    return 0;
}

