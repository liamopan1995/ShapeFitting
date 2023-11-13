option(BUILD_JSK_PKGS "Enable building of required components of jsk_recognition_msgs and jsk_rviz_plugins" ON)

if(BUILD_JSK_PKGS)
    add_subdirectory(include/jsk_recognition_msgs)
#     add_subdirectory(include/jsk_rviz_plugins) #TODO: allow building of rviz plugins as well
endif()

cmake_minimum_required(VERSION 3.0.2)
project(patchworkpp)

add_compile_options(-std=c++17)
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14) # was initially 14
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        roslaunch
        cv_bridge
        pcl_conversions
        pcl_ros
        geometry_msgs
        laser_geometry
        sensor_msgs
        message_generation
        jsk_recognition_msgs
)

find_package(OpenCV REQUIRED)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(OpenMP)
find_package(Sophus REQUIRED) 
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        include
)

#  For Cmake is able to find files under include folder
include_directories( ${PROJECT_SOURCE_DIR}/include )

#add_library( CircleClustering ${PROJECT_NAME}/include/utilities/CircleCluster.hpp )

# start of modification
## Declare a C++ library
add_library(${PROJECT_NAME} src/patchworkpp.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} patchworkpp_generate_messages_cpp)
# end of modification

add_executable(offline_kitti src/offline_kitti.cpp)
target_link_libraries(offline_kitti ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(offline_kitti patchworkpp_generate_messages_cpp)

add_executable(demo src/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(demo patchworkpp_generate_messages_cpp)

add_executable(video src/video.cpp)
target_link_libraries(video ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(video patchworkpp_generate_messages_cpp)

add_executable(groundStemSeg src/groundStemSeg.cpp)
target_link_libraries(groundStemSeg ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(groundStemSeg patchworkpp_generate_messages_cpp)

#added following 3 loc.
add_executable(CloudSegmentaion src/CloudSegmentaion.cpp )

target_link_libraries(CloudSegmentaion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(CloudSegmentaion patchworkpp_generate_messages_cpp  )


#testing tranfer gps to coordinates 
add_executable(GpsImuIntegral src/GpsImuIntegral.cpp)
target_link_libraries(GpsImuIntegral ${Sophus_LIBRARIES} ${catkin_LIBRARIES} )


add_executable(ImuIntegral src/imuIntegal.cpp)
target_link_libraries(ImuIntegral ${Sophus_LIBRARIES} ${catkin_LIBRARIES} )


add_executable(ImuIntegral_2d src/imu_2d.cpp)
target_link_libraries(ImuIntegral_2d ${Sophus_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(ekf_2d src/ekf.cpp)
target_link_libraries(ekf_2d ${Sophus_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(ekf_2d_v2 src/ekf_v2.cpp)
target_link_libraries(ekf_2d_v2 ${Sophus_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(ekf_2d_v3 src/ekf_v3.cpp)
target_link_libraries(ekf_2d_v3 ${Sophus_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(ekf_v3_new_io src/ekf_v3_new_io.cpp)
target_link_libraries(ekf_v3_new_io ${Sophus_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(ekf_v3_new_iov2 src/ekf_v3_new_iov2.cpp)
target_link_libraries(ekf_v3_new_iov2 ${Sophus_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(ekf_2d_v4 src/ekf_v4.cpp)
target_link_libraries(ekf_2d_v4 ${Sophus_LIBRARIES} ${catkin_LIBRARIES} )

# add_executable(new_GPS_IMU_EKF src/new_GPS_IMU_EKF.cpp)
# target_link_libraries(new_GPS_IMU_EKF ${Sophus_LIBRARIES} ${catkin_LIBRARIES} )
# ==== Install ====

# start of modification

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING 
#   PATTERN "*.h"
#   PATTERN "*.hpp"            #  by adding this  line  and it works ,    
#   PATTERN ".svn" EXCLUDE
# )
# end of modification
install(TARGETS demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#  adeded  following 4 LOC.
install(TARGETS CloudSegmentaion
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# End of adding
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)