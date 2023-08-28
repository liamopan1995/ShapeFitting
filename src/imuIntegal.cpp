#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "sophus/so3.h"

// Eigen::Vector3d pos = Eigen::Vector3d(0,0,0);
Sophus::SO3     rot = Sophus::SO3(Eigen::Matrix3d::Identity());
// Eigen::Vector3d vel = Eigen::Vector3d(0,0,0);
Eigen::Vector3d bg = Eigen::Vector3d(0,0,0);
Eigen::Vector3d ba = Eigen::Vector3d(0,0,0);
Eigen::Vector3d grav = Eigen::Vector3d(0,0,-9.8);

// double last_timeStamp;
// bool first_imu = true;

// ros::Publisher imu_pub;

// void imuCallback(const sensor_msgs::Imu::ConstPtr &msg_in)
// {
//   // Extract necessary IMU data from the received message
//   double timeStamp = msg_in->header.stamp.toSec();

//   // Initial condition: Set the previous timestamp and skip processing for the first message
//   if (first_imu)
//   {
//       last_timeStamp = timeStamp;
//       first_imu = false;
//       return;
//   }

//   double dt = timeStamp - last_timeStamp;
//   last_timeStamp = timeStamp;

//   Eigen::Vector3d angular_velocity(msg_in->angular_velocity.x,
//                                     msg_in->angular_velocity.y,
//                                     msg_in->angular_velocity.z);

//   Eigen::Vector3d linear_acceleration(msg_in->linear_acceleration.x,
//                                       msg_in->linear_acceleration.y,
//                                       msg_in->linear_acceleration.z);


//   // Update the rotation
//   Sophus::SO3 dR = Sophus::SO3::exp((angular_velocity - bg) * dt); // offset bg,noise is neglected for now
//   rot = rot * dR;

//   // Update the velocity
//   vel += (rot * (linear_acceleration - ba) + grav) * dt; // offset ba , ...

//   // Update the position
//   pos += vel * dt;

//   // Print the current position, velocity, and rotation
//   std::cout << "Position: " << pos.transpose() << std::endl;
//   std::cout << "Velocity: " << vel.transpose() << std::endl;
//   std::cout << "Rotation:\n" << rot.matrix() << std::endl;

//   // encapsule and publishing the resulted in type of  Imu msg
//   sensor_msgs::Imu imu_msg;
//   imu_msg.header.stamp = ros::Time::now();
//   imu_msg.header.frame_id = "imu_frame";
//   imu_msg.orientation.x = rot.unit_quaternion().x();
//   imu_msg.orientation.y = rot.unit_quaternion().y();
//   imu_msg.orientation.z = rot.unit_quaternion().z();
//   imu_msg.orientation.w = rot.unit_quaternion().w();
//   imu_msg.linear_acceleration.x = linear_acceleration.x();
//   imu_msg.linear_acceleration.y = linear_acceleration.y();
//   imu_msg.linear_acceleration.z = linear_acceleration.z();
//   imu_msg.angular_velocity.x = angular_velocity.x();
//   imu_msg.angular_velocity.y = angular_velocity.y();
//   imu_msg.angular_velocity.z = angular_velocity.z();
//   // Publish the new IMU message

//   imu_pub.publish(imu_msg);

// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "imu_integal");// node init
//   ros::NodeHandle nh;// init node handeler , it is then used to manage sub and publisers.

//   ros::Subscriber sub = nh.subscribe("/imu/data", 10000, imuCallback);
//   imu_pub = nh.advertise<sensor_msgs::Imu>("processed_imu", 1000);
//   ros::spin();

//   return 0;
// }



#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
bool first_imu = true;
ros::Publisher imu_pub;
double last_timeStamp;

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    // Extract necessary IMU data from the received message
    double timeStamp = msg_in->header.stamp.toSec();

    // Initial condition: Set the previous timestamp and skip processing for the first message
    if (first_imu)
    {   
      last_timeStamp = timeStamp;
        first_imu = false;
        
        return;
    }

    double dt = timeStamp - last_timeStamp;
    last_timeStamp = timeStamp;

    Eigen::Vector3d angular_velocity(msg_in->angular_velocity.x,
                                      msg_in->angular_velocity.y,
                                      msg_in->angular_velocity.z);

    Eigen::Vector3d linear_acceleration(msg_in->linear_acceleration.x,
                                        msg_in->linear_acceleration.y,
                                        msg_in->linear_acceleration.z);



  // Update the rotation
  Sophus::SO3 dR = Sophus::SO3::exp((angular_velocity - bg) * dt); // offset bg,noise is neglected for now
  rot = rot * dR;

  // Update the velocity
  vel += (rot * (linear_acceleration - ba) + grav) * dt; // offset ba , ...

  // Update the position
  pos += vel * dt;

  // // Print the current position, velocity, and rotation
  std::cout << "Position: " << pos.transpose() << std::endl;
  std::cout << "Velocity: " << vel.transpose() << std::endl;
  // std::cout << "Rotation:\n" << rot.matrix() << std::endl;

  // encapsule and publishing the resulted in type of  Imu msg
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "imu_frame";
  imu_msg.orientation.x = rot.unit_quaternion().x();
  imu_msg.orientation.y = rot.unit_quaternion().y();
  imu_msg.orientation.z = rot.unit_quaternion().z();
  imu_msg.orientation.w = rot.unit_quaternion().w();
  imu_msg.linear_acceleration.x = linear_acceleration.x();
  imu_msg.linear_acceleration.y = linear_acceleration.y();
  imu_msg.linear_acceleration.z = linear_acceleration.z();
  imu_msg.angular_velocity.x = angular_velocity.x();
  imu_msg.angular_velocity.y = angular_velocity.y();
  imu_msg.angular_velocity.z = angular_velocity.z();
  // Publish the new IMU message

  imu_pub.publish(imu_msg);



    // Extract the rotation quaternion from the provided transform information
    // Convert the provided quaternion to an Eigen quaternion
    Eigen::Quaterniond eigen_rot(
        0.31322694681719887,
        -0.052656498228382254,
        0.015847392653084057,
        0.9480849292800719
    );

    // Transform the position and velocity to world frame
    Eigen::Vector3d pos_world = eigen_rot * pos;
    Eigen::Vector3d vel_world = eigen_rot * vel;
    //Sophus::SO3     rot_world = Sophus::SO3::exp(eigen_rot.toRotationMatrix()) * rot;

    // Print the current position and velocity in world frame
    std::cout << "Position in world:  " << pos_world.transpose() << std::endl;
    std::cout << "Velocity in world: " << vel_world.transpose() << std::endl;
    //std::cout << "Rotation in world:\n" << rot_world.matrix() << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_integal");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/imu/data", 10000, imuCallback);
    imu_pub = nh.advertise<sensor_msgs::Imu>("processed_imu", 1000);

    ros::spin();

    return 0;
}
