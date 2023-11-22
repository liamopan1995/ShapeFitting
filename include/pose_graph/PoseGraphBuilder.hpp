
#ifndef POSE_GRAPH_BUILDER_H
#define POSE_GRAPH_BUILDER_H

#include <string>
#include <vector>
#include <unordered_set>
#include <Eigen/Dense>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include "common/utilities.h" // Custom header for utility structures like Vec6d
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <armadillo>
#include <limits>
class PoseGraphBuilder {
public:
    g2o::SparseOptimizer optimizer_;
    std::vector<Vec6d> clusteredData_;
    //int last_pose_id_;
    //g2o::VertexSE2* last_se2_;
    PoseGraphBuilder();

    void build_optimize_Graph(const std::vector<Vec6d>& globalMap, 
                    std::vector<Vec6d> localMap, 
                    const std::vector<MovementData>& odometry,
                    const std::vector<MovementData>& translationPose2Pose,
                    bool  use_kernel=false,
                    double kernelwidth_SE2=1,
                    double kernelwidth_SE2XY=1);
    void saveGraph();
    void saveGraph(std::string location);
    void clear_edges_vertices();

private:
    
    void initializeOptimizer();
    void clusterData(const std::vector<Vec6d>& globalMap);
    void addVerticesAndEdges(   
                             std::vector<Vec6d> localMap, 
                             const std::vector<MovementData>& odometry,
                             const std::vector<MovementData>& translationPose2Pose);
    void addVerticesAndEdges_kernelized(
                             std::vector<Vec6d> localMap, 
                             const std::vector<MovementData>& odometry,
                             const std::vector<MovementData>& translationPose2Pose,
                             double kernelwidth_SE2,
                             double kernelwidth_SE2XY);
    // infomation matrice
    Mat3d information_edge_se2_;
    Mat2d information_edge_xy_;


    // Additional private members and methods...
    // Todo  iterate over vertices and publish them in ros message types.
};

#endif // POSE_GRAPH_BUILDER_H
