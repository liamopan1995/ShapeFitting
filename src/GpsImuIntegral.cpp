#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "utilities/toUTM.hpp"
#include <nav_msgs/Path.h>


ros::Publisher path_publisher;
void GPUCallback(const sensor_msgs::NavSatFix::ConstPtr& msg_in)
{
    
    double latitude = msg_in->latitude;
    double longitude = msg_in->longitude;
    double altitude = msg_in->altitude;


    // Process the GPS data as required
  
    double UTMNorthing, UTMEasting = 0;
    char UTMZone[4];
    LLtoUTM(latitude, longitude, UTMNorthing, UTMEasting, UTMZone);

    // for debugging use :
    std::cout << "GPS Data: Lat=" << latitude << ", Lon=" << longitude << ", Alt=" << altitude << std::endl;
    UTMNorthing -= (5.424e+06); //normalize
    UTMEasting -= (463800);
    std::cout <<" UTMNorthing: "<< UTMNorthing <<" UTMEasting: "<<UTMEasting <<std::endl;



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

    path_publisher.publish(path_msg);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "GPSpath");// node init
  ros::NodeHandle nh;// init node handeler , it is then used to manage sub and publisers.

  ros::Subscriber subGPS = nh.subscribe("/lernsmart/isobus_read/NavSatFix_in", 10000, GPUCallback);
  path_publisher = nh.advertise<nav_msgs::Path>("gps_path", 20);

  ros::spin();
  return 0;
}
