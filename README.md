# Contour-based SLAM for a Robust Odometry for Forestry Vehicles

A modified version of Patchworkpp with additional code for Stem Segmentation

## Introduction

Recognizing tree contours within densely managed forests through LiDAR(Light Detection
and Ranging) and futher identifying stem locations are feasible. Hence employing resulting
stem-location maps as references for localization is achievable.

This study introduces a novel Simultaneous Localization and Mapping (SLAM) algorithm
that relies on identifying stem points based on the spatial distribution properties of laser
points. The stem model is constructed as a series of cylinders, and stem location is
estimated using this model, at the post end, G2O is applied for post-end optimization.

The performance of this algorithm is assessed by comparing its results with those obtained
through the GNSS and one state-of-the-art SLAM algorithm for the same trajectory.
The ndings demonstrate that utilizing stem-location features for simultaneous localization
and mapping presents a robust solution for SLAM in densely managed forests.

The ground segmentation is done with Patchworkpp library which can be found at [Patchwork++ ROS](https://github.com/original_repository_link). Please refer to the original repository for more information about Patchworkpp.

## Features

- Ground segmentation: Utilize Patchworkpp's ground segmentation functionality to identify and extract ground points from a point cloud.
- Stem segmentation: Build upon Patchworkpp to include code that can extract tree stems or trunks from the point cloud data.

## Getting Started

To get started with this project, follow these steps:

1. Clone the repository:

place it under /catkin_ws/src
  <pre>
cd ~/catkin_ws/src
git clone ...
  </pre>

2. Install the required dependencies. 
PCL, ROS, Eigen，Sophus, G2O
For Sophus, this specific version might be needed [download here](https://drive.google.com/file/d/1t2P_DoQ9s0Q0I5ukexdbTpXXRIuBt62x/view?usp=drive_link)

For G2O, this specific version might be needed [download here](https://drive.google.com/file/d/1Q-nvxnD4tugpC4kOWcjvvKKamVvYKRw4/view?usp=drive_link)

4. Build the project:
  <pre>
cd ~/catkin_ws
catkin_make
  </pre>

5. Run the application:
  <pre>
cd ~/catkin_ws
source devel/setup.bash
roslaunch patchworkpp CloudSegmentation.launch
rosbag play ...  ( pointcloud topic has to be /velodyne_points for present)
  </pre>

## Operation Demonstration 
[Video](https://drive.google.com/file/d/1N1WgAAsGCnm57edv6hc4fqz94HvVkzQY/view?usp=sharing)

## License

 no license has been specified yet. However, it is subjected to the license in [patchworkpp ros](https://github.com/original_repository_link).

## Acknowledgments

This project is built upon the work of the original Patchworkpp library. Thanks to the contributors of Patchworkpp for their valuable contributions.





