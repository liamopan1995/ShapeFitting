/*
Brutal Force Nearest Neighbour Search
*/  
#include "bfnn.h"
#include <execution>


// find one point's nearest point in the given cloud
int bfnn_point(std::vector<Vec3d> cloud, const Vec3d& point){

    return std::min_element (cloud.begin(),cloud.end(),
    [&point](const Vec3d pt1, const Vec3d pt2) -> bool {
        return(pt1 - point).squaredNorm() < (pt2 - point).squaredNorm();
    }
    ) - cloud.begin();
}


/// revised version , not used  for now
bool bfnn_point_reversion(std::vector<Vec3f> cloud, const Vec3f& point, std::vector<int> nn){

    nn.push_back( std::min_element (cloud.begin(),cloud.end(),
    [&point](const Vec3f pt1, const Vec3f pt2) -> bool {
        return(pt1 - point).squaredNorm() < (pt2 - point).squaredNorm();
    }
    ) - cloud.begin());
    return 1;
}




// A implementation that  finds the corresponding idxes in two scans by brutal force method. 
// Requires compiler enviroemtn to be set as C++ 17

// void bfnn_cloud_mt( std::vector<Vec3d> cloud1,std::vector<Vec3d> cloud2, std::vector<std::pair<size_t, size_t>>& matches){
// /// for each pt in cloud 2, find  its match in cloud 1
//     // generate index
//     //std::cout<<"\n bfnn_cloud_mt is called\n";
//     std::vector<size_t>index(cloud2.size());
//     std::for_each(index.begin(),index.end() ,[idx=0] (size_t& i) mutable {i= idx++;});

//     matches.resize(index.size());
//     std::for_each (std::execution::par_unseq, index.begin(),index.end(), [&](auto idx){
//         matches[idx].second =idx;
//         matches[idx].first = bfnn_point(cloud1,cloud2[idx]);// find cloud2[idx]'s cp's idx in cloud1 
//     });
// }

// Modified to work with C++14
void bfnn_cloud_mt(std::vector<Vec3d> cloud1, std::vector<Vec3d> cloud2, std::vector<std::pair<size_t, size_t>>& matches){
    matches.resize(cloud2.size());

    for(size_t idx = 0; idx < cloud2.size(); ++idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, cloud2[idx]); // find cloud2[idx]'s cp's idx in cloud1
    }
}