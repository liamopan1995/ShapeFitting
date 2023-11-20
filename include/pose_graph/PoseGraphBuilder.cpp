
#include "PoseGraphBuilder.hpp"
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

PoseGraphBuilder::PoseGraphBuilder() {
    initializeOptimizer();
    information_edge_se2_ = Mat3d::Identity();
    information_edge_xy_ = Mat2d::Identity();
    // information_edge_se2_ << 100.0, 0.0, 0.0,
    //                              0.0, 100.0, 0.0,
    //                              0.0, 0.0, 1000.0;
    // information_edge_xy_ << 100.0, 0.0,
    //                             0.0, 100.0;
}
void PoseGraphBuilder::clear_edges_vertices(){
    optimizer_.clear();
    std::cout<<"all vertices and edges are deleted, ready for new problem"<<std::endl;
}
void PoseGraphBuilder::initializeOptimizer() {
    optimizer_.setVerbose(true);
    // Choose the linear solver - CHOLMOD ( this one is especially fast)
    auto linearSolver = std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();

    // Create the block solver
    auto blockSolver = std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));

    // Choose the algorithm - Gauss-Newton
    auto algorithm = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));

    optimizer_.setAlgorithm(algorithm);
}


void PoseGraphBuilder::build_optimize_Graph(const std::vector<Vec6d>& globalMap, 
                                  std::vector<Vec6d> localMap, 
                                  const std::vector<MovementData>& odometry,
                                  const std::vector<MovementData>& translationPose2Pose) {
    clusterData(globalMap);
    addVerticesAndEdges( localMap, odometry, translationPose2Pose);
    optimizer_.initializeOptimization();
    optimizer_.optimize(30); // You might want to make the number of iterations configurable
}

void PoseGraphBuilder::saveGraph() {

    optimizer_.save("../result_g2o/result_2d.g2o");
}
void PoseGraphBuilder::saveGraph(std::string filename) {

    optimizer_.save(filename.c_str());
}

void PoseGraphBuilder::clusterData(const std::vector<Vec6d>& globalMap) {
        // Assuming global_map is a std::vector<Vec6d>
    clusteredData_.clear();

    arma::mat data;
    for (const auto& point : globalMap) {
        // Include only x, y, r
        data.insert_cols(data.n_cols, arma::vec({point(0,0), point(1,0), point(2,0)}));
    }

    mlpack::dbscan::DBSCAN<> dbscan(0.5, 5); // Example values for eps and min_samples Nov ori: (0.5 5)
    // (0.5 5)
    // (0.5 3)
    // (0.7, 5) good
    // try 3
    arma::Row<size_t> assignments;
    dbscan.Cluster(data, assignments);

    // std::vector<Vec6d> clusteredData;
    for (size_t i = 0; i < globalMap.size(); ++i) {
        Vec6d objectWithCluster = globalMap[i];
        size_t clusterId = assignments[i];

        // Append the cluster ID to the object's existing data
        // Assuming the 5th index of Vec6d is free to store the cluster ID
        if (clusterId == std::numeric_limits<size_t>::max()) {
        // If the point is considered noise, set the cluster ID to -1
            objectWithCluster(5, 0) = -1;
        } else {
        // Otherwise, use the actual cluster ID
            objectWithCluster(5, 0) = static_cast<double>(clusterId);
        }

        clusteredData_.push_back(objectWithCluster);
    }
}

void PoseGraphBuilder::addVerticesAndEdges(
                                           std::vector<Vec6d> localMap, 
                                           const std::vector<MovementData>& odometry,
                                           const std::vector<MovementData>& translationPose2Pose) {
                                            
    // Update global_map so it now has global cluster idex

    // globalMap = clusteredData;

    //  Update local_map so it now has global cluster idex
    if(clusteredData_.size() ==localMap.size()) {
        std::cout<<"global map and local map are of same size" <<std::endl;
        for(int i=0;i<localMap.size();++i) {
            localMap[i](5) = clusteredData_[i](5);
            // cout<<global_map[i].transpose()<<"     global"<<endl;
            // cout<<local_map[i].transpose()<<"      local"<<endl<<endl;
            // if (i>100) return 0;
        }
    }
    //return 0;


    //std::vector<Vec6d> global_map_test = localMap;

//  Averaging the features in global_map(clusteredData) by clusters  ( find centriod of each cluster, 
//  which will be used as initial guess for stem centers in the global map)

    // First, calculate the centroids of each cluster  in clusteredData_
    std::unordered_map<size_t, std::pair<Vec6d, size_t>> clusterSums;
    for (const auto& point : clusteredData_) {
        size_t clusterId = static_cast<size_t>(point(5, 0));
        if (clusterId != std::numeric_limits<size_t>::max()) {
            clusterSums[clusterId].first(0, 0) += point(0, 0); // Sum x
            clusterSums[clusterId].first(1, 0) += point(1, 0); // Sum y
            clusterSums[clusterId].second += 1; // Count
        }
    }
    std::unordered_map<size_t, Vec6d> centroids;
    for (const auto& pair : clusterSums) {
        size_t clusterId = pair.first;
        Vec6d sum = pair.second.first;
        size_t count = pair.second.second;
        centroids[clusterId](0, 0) = sum(0, 0) / count; // Average x
        centroids[clusterId](1, 0) = sum(1, 0) / count; // Average y
    }

    // Now replace x and y of each point with the centroid of its cluster
    for (auto& point : clusteredData_) {
        size_t clusterId = static_cast<size_t>(point(5, 0));
        if (clusterId != std::numeric_limits<size_t>::max()) {
            point(0, 0) = centroids[clusterId](0, 0); // Centroid x
            point(1, 0) = centroids[clusterId](1, 0); // Centroid y
        }
    }

    std::unordered_set<size_t> processedClusters;
    int vertex_cnt = 0;
    for (size_t i = 0; i < clusteredData_.size(); ++i) {
        size_t clusterId = clusteredData_[i](5,0);

        if (clusterId == -1 || clusterId == std::numeric_limits<size_t>::max() || processedClusters.count(clusterId) > 0) {
            // Skip if it's noise or already processed
            continue;
        } else {
            // Process this cluster
            g2o::VertexPointXY* vertex = new g2o::VertexPointXY();
            vertex->setId(clusterId);
            vertex->setEstimate(Eigen::Vector2d(clusteredData_[i](0), clusteredData_[i](1))); // Set the x, y position
            optimizer_.addVertex(vertex);
            vertex_cnt++;
            processedClusters.insert(clusterId); // Mark this clusterId as processed
        }
    }
    //cout<< "vertex_cnt in the end :"<< vertex_cnt<<endl;// Notice: Vertex idx is started from 0 !

    int iter_local_map = 0;

    // in order to differ from the idices occupied by VertexXY in above.
    for ( size_t i = 0; i < odometry.size() ; ++i) {
        // int vertex_se2_id = i + vertex_cnt + 1; // this is the vertex ID we going to start with
        int vertex_se2_id = i + vertex_cnt + 1   ; //  additionally +49 if want to let it be the same as the result in python
        auto pose = odometry[i];
        double timestamp = pose.timestamp_;
        Eigen::Vector3d euler_angles = pose.R_.eulerAngles(2, 1, 0);
        double x = pose.p_(0);
        double y = pose.p_(1);
        double yaw = euler_angles[0] ;

        // added a vertex , 
        //cout<< i + vertex_cnt + 1 <<"  !"<<endl;  // this is the vertex ID we going to use here
        
        g2o::VertexSE2* v = new g2o::VertexSE2();
        v->setId(vertex_se2_id);
        v->setEstimate(g2o::SE2(x, y, yaw));
        //cout<< "new vertex ,  id: "<< vertex_se2_id <<"added\n";
        // Fix the first vertex
        if(vertex_se2_id == vertex_cnt + 1    ) { //+49
            v->setFixed(true);
            optimizer_.addVertex(v); 
            //cout<< "first node is set fix, first node id: "<< vertex_se2_id <<"\n";
            
        } else {
            optimizer_.addVertex(v);
            // set a edege EDGE_SE2 to the previous  node : i.e : vertex_se2_id-1
            int to_idx = vertex_se2_id;
            int from_idx = to_idx-1; 
            //cout<< " from_idx and to_idx : "<<from_idx<<"   " << to_idx <<endl;
            // since first tranlation in the vec: translation_pose2pose is the move from id:0 to id:1
            auto translation = translationPose2Pose[i-1];
            Eigen::Vector3d euler_angles = translation.R_.eulerAngles(2, 1, 0);
            double x = translation.p_(0);
            double y = translation.p_(1);
            double yaw = euler_angles[0];
   
            //cout<< "x ,y ,yaw : "<< x<<" "<< y<< " "<<yaw<<endl;
            g2o::EdgeSE2* e = new g2o::EdgeSE2();
            // Set the connecting vertices (nodes)
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(from_idx)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(to_idx)));

            // Set the measurement (relative pose)
            g2o::SE2 relative_pose(x, y, yaw);
            e->setMeasurement(relative_pose);
            e->setInformation(information_edge_se2_);// defined at the beginning of main()

            // Add the edge to the optimizer
            optimizer_.addEdge(e);
            //cout<< "edge se2 has been added in between from: "<<from_idx<<" to " << to_idx <<endl;
            // return 0;
        }
        

        // Iterate over the data and print the 6th element where the 4th element matches the specific value
        while(true){
            // cout<< "localMap[iter_local_map]: " << localMap[iter_local_map]<<endl;
            // cout<< "localMap[iter_local_map](3) : " << localMap[iter_local_map](3)<<endl;
            if (timestamp == localMap[iter_local_map](3)) {
                int vertex_xy_id = localMap[iter_local_map](5);
    
                if (vertex_xy_id == -1){
                    //cout<<" 6th Element equal -1: skipping it" << endl;
                    // Do nothing since it is a outlier
                } else {
                    double x =localMap[iter_local_map](0);
                    double y =localMap[iter_local_map](1);
                    // std::cout << "time stamp  " <<std::setprecision(18)<< localMap[iter_global_map](3)<<
                    // " 6th Element (global idx,i.e vertexXY's ID): " << vertex_xy_id << "  x "<<
                    // x<<"  y "<< y << std::endl;

                    // BUILD THE EDGESE2PointXY :
                    g2o::EdgeSE2PointXY* e = new g2o::EdgeSE2PointXY();
                    // Set the connecting vertices (nodes)
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(vertex_se2_id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(vertex_xy_id)));
                    //Eigen::Vector2d position(x, y); // Assuming x and y are defined earlier as the positions
                    e->setMeasurement(Eigen::Vector2d(x, y));
                    e->setInformation(information_edge_xy_); // Define information_edge_xy appropriately
                    optimizer_.addEdge(e);
                    //cout << "Edge XY has been added in between from: " << vertex_se2_id << " to " << vertex_xy_id << endl;

                }
                ++iter_local_map;
            } else {break;}
            
        }
        // if(iter_local_map>49) return 0;
    }

}

