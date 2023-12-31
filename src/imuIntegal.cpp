
#include "sophus/so3.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iomanip>
#include <fstream>

using namespace Eigen;

Sophus::SO3 rot = Sophus::SO3(Eigen::Matrix3d::Identity());
Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);

Eigen::Vector3d bg = Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d ba = Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d grav = Eigen::Vector3d(0, 0, -9.8);

bool first_imu = true;
double last_timeStamp;

ros::Publisher imu_pub;
ros::Publisher path_pub; // Publisher for the path
nav_msgs::Path path_msg; // Path message

Eigen::Vector3d accumulated_linear_acceleration = Eigen::Vector3d::Zero();
Eigen::Vector3d accumulated_angular_velocity = Eigen::Vector3d::Zero();
int stationary_data_count = 0;
const double stationary_duration = 1.0; // in seconds
bool init_flag = false;
bool first_time_flag = false;
double firstTimeStamp = 0.;
// Define a struct to store IMU data
struct IMUData {
    double timestamp_;
    Sophus::SO3 R_;
    Eigen::Vector3d v_;
    Eigen::Vector3d p_;
};

std::vector<IMUData> imu_data; // Store IMU data






void imuCallback(const sensor_msgs::Imu::ConstPtr &msg_in) {

    // double heading_degrees = 2*90; // for Fendt
    double heading_degrees = 2*90; // for Gasse   it is 180, if result not align to this, this is due to bag corruption
    double heading_radians = heading_degrees * M_PI / 180.0;

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(heading_radians, Eigen::Vector3d::UnitZ());
    // std::cout<<rotation_matrix<<std::endl;
    Sophus::SO3 Tsolid2dash = Sophus::SO3(rotation_matrix);


    // Extract necessary IMU data from the received message
    double timeStamp = msg_in->header.stamp.toSec();

    if(!first_time_flag){
        firstTimeStamp = timeStamp; // timestamp of the first imu
        first_time_flag = true;
    }
    
    // Check if the current time is within the stationary period
    if (timeStamp  - firstTimeStamp <= 2.0) { // Assuming 2 second stationary period
        Eigen::Vector3d angular_velocity(msg_in->angular_velocity.x,
                                         msg_in->angular_velocity.y,
                                         msg_in->angular_velocity.z);

        Eigen::Vector3d linear_acceleration(msg_in->linear_acceleration.x,
                                           msg_in->linear_acceleration.y,
                                           msg_in->linear_acceleration.z);

        // Accumulate linear acceleration and angular velocity data
        accumulated_linear_acceleration += (linear_acceleration + grav); 
        accumulated_angular_velocity += angular_velocity;
        stationary_data_count++;
    }

    // Initial condition: Set the previous timestamp and skip processing for the first message
    if (first_imu) {
        last_timeStamp = timeStamp;
        first_imu = false;
        return;
    }
    // after stand still for 2 seconds, calculate the bias .
    if (timeStamp  - first_time_flag > 2.0 && !init_flag) {
        // Calculate the average linear acceleration and angular velocity
        ba = accumulated_linear_acceleration / stationary_data_count;
        bg = accumulated_angular_velocity / stationary_data_count;

    // Output the estimated bias values
    std::cout << "stationary_data_count: " << stationary_data_count << std::endl;
    std::cout << "Estimated ba (accelerometer bias): " << ba.transpose() << std::endl;
    std::cout << "Estimated bg (gyroscope bias): " << bg.transpose() << std::endl;
    init_flag = true;
    }


    double dt = timeStamp - last_timeStamp;
    last_timeStamp = timeStamp;

    Eigen::Vector3d angular_velocity(msg_in->angular_velocity.x,
                                     msg_in->angular_velocity.y,
                                     msg_in->angular_velocity.z);

    Eigen::Vector3d linear_acceleration(msg_in->linear_acceleration.x,
                                       msg_in->linear_acceleration.y,
                                       msg_in->linear_acceleration.z);

    if (dt > 0 && dt < 0.1) {
        // Update the position
        pos += vel * dt + 0.5 * grav * dt * dt + 0.5 * (rot * (linear_acceleration - ba)) * dt * dt;
        // Update the velocity
        vel += (rot * (linear_acceleration - ba) + grav) * dt; // 
        // Update the rotation
        Sophus::SO3 dR = Sophus::SO3::exp((angular_velocity - bg) * dt); // 
        rot = rot * dR;


        // geometry_msgs::PoseStamped pose;
        // pose.header.stamp = ros::Time::now();
        // pose.header.frame_id = "world"; // You can change this frame ID if needed
        // pose.pose.position.x = pos[0];
        // pose.pose.position.y = pos[1];
        // pose.pose.position.z = pos[2];
        // pose.pose.orientation.x = rot.unit_quaternion().x();
        // pose.pose.orientation.y = rot.unit_quaternion().y();
        // pose.pose.orientation.z = rot.unit_quaternion().z();
        // pose.pose.orientation.w = rot.unit_quaternion().w();

        // // Add the pose to the path
        // path_msg.header.stamp = ros::Time::now();
        // path_msg.header.frame_id = "world"; // You can change this frame ID if needed
        // path_msg.poses.push_back(pose);

        // Publish the path
        //path_pub.publish(path_msg);



        // Store IMU data
        IMUData imu_data_point;
        imu_data_point.timestamp_ = timeStamp;
        //  transform the points  from solid frame to utm frame
        imu_data_point.R_ = Tsolid2dash*rot;
        imu_data_point.v_ = Tsolid2dash*vel;
        imu_data_point.p_ = Tsolid2dash*pos;
        // imu_data_point.R_ = rot;
        // imu_data_point.v_ = vel;
        // imu_data_point.p_ = pos;
        imu_data.push_back(imu_data_point);
    }
}

int main(int argc, char **argv) {




    ros::init(argc, argv, "imu_integal");
    ros::NodeHandle nh;

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
    std::string fullPath = std::string(homeDir) + "/slam_in_autonomous_driving/data/ch3/velodyne_bag_txt/state_imu.txt";

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

