#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "utilities/toUTM.hpp"
#include <nav_msgs/Path.h>
#include "sophus/so3.h"

Eigen::Vector3d pos = Eigen::Vector3d(0,0,0);
Sophus::SO3     rot = Sophus::SO3(Eigen::Matrix3d::Identity());
Eigen::Vector3d vel = Eigen::Vector3d(0,0,0);
Eigen::Vector3d bg = Eigen::Vector3d(0,0,0);
Eigen::Vector3d ba = Eigen::Vector3d(0,0,0);
Eigen::Vector3d grav = Eigen::Vector3d(0,0,-9.8);
double last_timeStamp;
bool first_imu=true;
bool first_utm = true;
bool flag_aligned = false;

double first_UTMNorthing=0;
double  first_UTMEasting=0;
double UTMNorthing=0;
double UTMEasting=0;
double latitude=0;
double longitude=0;
double altitude=0;
ros::Time lastDebugPrintTime;

ros::Publisher path_publisher_gps;
ros::Publisher path_publisher_imu;


void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg_in)
{
    
    latitude = msg_in->latitude;
    longitude = msg_in->longitude;
    altitude = msg_in->altitude;


    // Process the GPS data as required

    char UTMZone[4];
    LLtoUTM(latitude, longitude, UTMNorthing, UTMEasting, UTMZone);

    if(first_utm == true){
    first_UTMNorthing = UTMNorthing;
    first_UTMEasting = UTMEasting;
    first_utm = false; 
    return;
    }

    // for debugging use :
    std::cout << "\nGPS Data: Lat=" << latitude << ", Lon=" << longitude << ", Alt=" << altitude << std::endl;
    // UTMNorthing -= (5.424e+06); //normalize
    // UTMEasting -= (463800);
    std::cout <<" \nUTMEasting(x): "<<UTMEasting -first_UTMEasting<<" \nUTMNorthing(y): "<< UTMNorthing -first_UTMNorthing<<std::endl;




    // TODO 
    // visualize the path

    // Create a nav_msgs::Path message to store the GPS data as a path
    nav_msgs::Path path_msg;
    path_msg.header = msg_in->header; // Set the header of the Path message

    // this line is for testing purpose
    path_msg.header.frame_id = "velodyne";   


    // Create a geometry_msgs::PoseStamped message to store each pose (GPS data point)
    geometry_msgs::PoseStamped pose_stamped;  
    pose_stamped.header = path_msg.header; // Set the header of the PoseStamped message
    pose_stamped.pose.position.x = UTMEasting ; // Use UTMEasting as the x-coordinate   
    pose_stamped.pose.position.y = UTMNorthing ; // Use UTMNorthing as the y-coordinate
    pose_stamped.pose.position.z = altitude; // Use altitude as the z-coordinate
    // Add the pose_stamped message to the path_msg
    path_msg.poses.push_back(pose_stamped);
    // Publish the path_msg

    path_publisher_gps.publish(path_msg);

}



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

  if ( !first_utm && flag_aligned)
  {
      pos(0)+=first_UTMEasting;
      pos(1)+=first_UTMNorthing;
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

  // Update the rotation,velocity,position
  Sophus::SO3 dR = Sophus::SO3::exp((angular_velocity - bg) * dt); // offset bg,noise is neglected for now
  rot = rot * dR;
  vel += (rot * (linear_acceleration - ba) + grav) * dt; // offset ba , ...
  pos += vel * dt;

  // Debuging, Print the current position, velocity, and rotation
  std::cout << "\nPosition: " << pos.transpose() << std::endl;
  std::cout << "Velocity: " << vel.transpose() << std::endl;
  std::cout << "Rotation:\n" << rot.matrix() << std::endl<< std::endl;

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

  path_publisher_imu.publish(imu_msg);



}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_imu_path");// node init
  ros::NodeHandle nh;// init node handeler , it is then used to manage sub and publisers.

  ros::Subscriber subGPS = nh.subscribe("/lernsmart/isobus_read/NavSatFix_in", 10000, GPSCallback);
  //ros::Subscriber sub = nh.subscribe("/imu/data", 10000, imuCallback);

  path_publisher_gps = nh.advertise<nav_msgs::Path>("gps_path", 20);
  //path_publisher_imu = nh.advertise<sensor_msgs::Imu>("imu_path", 1000);

  ros::spin();
  return 0;
}
