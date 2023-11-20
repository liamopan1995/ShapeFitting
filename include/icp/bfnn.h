///  brutal force NN searching method implementation
///  this approach suit for search in 2d space when the number of point is significantly small
///  as our case each scan contains in average 20 trees, this method is considerable
///  on the other hand is this approcah less commonly used in other cases, it has a time complexity of
///  O(n^2)

///  implement a more advanced matching  algorithm such as search in grid , kd tree or oct tree.
///  this more advanced method shall not take vector as input , instead it should take point cloud
///  which is supposed to be generated basing on the normal vector ( or even the radius) from that 
///  fitted stem on the corresponding cluster -- 19th semptember.
///  Continue research in post end  optimization
#ifndef SLAM_IN_AUTO_DRIVING_BFNN_H
#define SLAM_IN_AUTO_DRIVING_BFNN_H

#include "common/utilities.h"
#include <vector>


// #include <thread>

// namespace sad {

// /**
//  * Brute-force Nearest Neighbour
//  * @param cloud 点云
//  * @param point 待查找点
//  * @return 找到的最近点索引
//  */
// int bfnn_point(std::vector<Vec3f> cloud, const Vec3f& point);
int bfnn_point(std::vector<Vec3d> cloud, const Vec3d& point);
// /**
//  * Brute-force Nearest Neighbour, k近邻
//  * @param cloud 点云
//  * @param point 待查找点
//  * @param k 近邻数
//  * @return 找到的最近点索引
//  */
// std::vector<int> bfnn_point_k(CloudPtr cloud, const Vec3f& point, int k = 5);

// /**
//  * 对点云进行BF最近邻
//  * @param cloud1  目标点云
//  * @param cloud2  被查找点云
//  * @param matches 两个点云内的匹配关系
//  * @return
//  */
// void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches);

// /**
//  * 对点云进行BF最近邻 多线程版本
//  * @param cloud1
//  * @param cloud2
//  * @param matches
//  */
// void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches);
// void bfnn_cloud_mt( std::vector<Vec3f> cloud1,std::vector<Vec3f> cloud2, std::vector<std::pair<size_t, size_t>>& matches);
void bfnn_cloud_mt( std::vector<Vec3d> cloud1,std::vector<Vec3d> cloud2, std::vector<std::pair<size_t, size_t>>& matches);
// /**
//  * 对点云进行BF最近邻 多线程版本，k近邻
//  * @param cloud1  
//  * @param cloud2
//  * @param matches
//  */
// void bfnn_cloud_mt_k(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches, int k = 5);
// }  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_BFNN_H
