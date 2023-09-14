#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "utilities/toUTM.hpp"

#include <nav_msgs/Path.h>
#include "sophus/so3.h"
#include <iomanip>
#include <fstream>

#include "ESKF/eskf.hpp"
#include "ESKF/static_imu_init.h"
#include "utm_convert.h"
// #include "ESKF/common/imu.h"
Eigen::Vector3d pos = Eigen::Vector3d(0,0,0);
Sophus::SO3     rot = Sophus::SO3(Eigen::Matrix3d::Identity());
Eigen::Vector3d vel = Eigen::Vector3d(0,0,0);
Eigen::Vector3d bg = Eigen::Vector3d(0,0,0);
Eigen::Vector3d ba = Eigen::Vector3d(0,0,0);
Eigen::Vector3d grav = Eigen::Vector3d(0,0,-9.8);

Eigen::Vector3d accumulated_linear_acceleration = Eigen::Vector3d::Zero();
Eigen::Vector3d accumulated_angular_velocity = Eigen::Vector3d::Zero();

bool imu_inited = false, gnss_inited = false,first_utm = true;
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
nav_msgs::Path path_msg;

// Define a struct to store IMU data
struct UTMData {
    double timestamp_;
    Eigen::Vector3d p_;
};



std::vector<UTMData> utm_data; // Store IMU data

int stationary_data_count = 0;
double firstTimeStamp = 0.;

sad::ESKFD eskf;
sad::StaticIMUInit imu_init;  // 使用默认配置


void GPSCallback( const sensor_msgs::NavSatFix::ConstPtr& msg_in)
{
    double cur_time = msg_in->header.stamp.toSec();
    latitude = msg_in->latitude;
    longitude = msg_in->longitude;
    altitude = msg_in->altitude;


    // Process the GPS data as required


    char UTMZone[4];
    LLtoUTM(latitude, longitude, UTMNorthing, UTMEasting, UTMZone);

    /// 去掉原点
    if(first_utm == true){
        first_UTMNorthing = UTMNorthing;
        first_UTMEasting = UTMEasting;
        first_utm = false; 
        return;
    }




    sad::GNSS gnss_convert ( msg_in );
    gnss_convert.utm_pose_.translation() = Vec3d(UTMEasting - first_UTMEasting, UTMNorthing - first_UTMNorthing, 0);
    gnss_convert.utm_valid_ = true;

    eskf.ObserveGps(gnss_convert);
    auto state = eskf.GetNominalState();

    std::cout<<"UTM\n"<<std::endl;
    std::cout<<state.p_<<std::endl;
    // // for debugging use :
    // std::cout << "\nGPS Data: Lat=" << latitude << ", Lon=" << longitude << ", Alt=" << altitude << std::endl;

    // std::cout <<" \nUTMEasting(x): "<<UTMEasting -first_UTMEasting<<" \nUTMNorthing(y): "<< UTMNorthing -first_UTMNorthing<<std::endl;

    //eskf.ObserveGps()



}



void imuCallback(const sensor_msgs::Imu::ConstPtr &msg_in)
{         sad::IMU imu(msg_in);
              /// IMU 处理函数
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }

          /// 需要IMU初始化
          if (!imu_inited) {
              // 读取初始零偏，设置ESKF
              sad::ESKFD::Options options;
              // 噪声由初始化器估计
              options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
              options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
              eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
              imu_inited = true;
              return;
          }

          if (!gnss_inited) {
              /// 等待有效的RTK数据
              return;
          }

    eskf.Predict(imu);
    auto state = eskf.GetNominalState();
    std::cout<<"GPS\n"<<std::endl;
    std::cout<<state.p_<<std::endl;
}



int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "gps_imu_path");// node init
    ros::NodeHandle nh;// init node handeler , it is then used to manage sub and publisers.

    ros::Subscriber subGPS = nh.subscribe("/lernsmart/isobus_read/NavSatFix_in", 10000, GPSCallback);
    
    ros::Subscriber sub = nh.subscribe("/imu/data", 10000, imuCallback);

    path_publisher_gps = nh.advertise<nav_msgs::Path>("gps_path", 20);
    //path_publisher_imu = nh.advertise<sensor_msgs::Imu>("imu_path", 1000);

    ros::spin();
  
    // Save IMU data to a text file

        // Get user's home directory
    const char* homeDir = getenv("HOME");
    if (!homeDir) {
        std::cerr << "Error: HOME environment variable not set." << std::endl;
        return 1;
    }

    // Construct the desired path
    std::string fullPath = std::string(homeDir) + "/slam_in_autonomous_driving/data/ch3/velodyne_bag_txt/state_utm.txt";

    // Open the file for writing
    std::ofstream fout(fullPath);
    if (!fout) {
        std::cerr << "Error: Unable to open file for writing." << std::endl;
        return 1;
    }
    for (const UTMData& utm_point : utm_data) {
        fout << std::setprecision(18) << utm_point.timestamp_ << " " << std::setprecision(9)
             << utm_point.p_[0] << " " << utm_point.p_[1] << " " <<0 << " " <<1 << " " <<0 << " " <<0 << " " <<0<<std::endl;
    }



    return 0;
}
