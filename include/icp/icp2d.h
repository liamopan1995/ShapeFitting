/*
Class: ICP 2d  Implementation 
*/
#ifndef ICP_2D_CLASS
#define ICP_2D_CLASS
#include "common/utilities.h"
//#include "kdtree.h"
#include <numeric>
#include "bfnn.h"
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h> 


class Icp2d {
   public:
    double fitness_;  // fitness= intersection/union

    struct Options {
        ///int max_iteration_ = 20;                // 最大迭代次数
        double max_nn_distance_ = 1.0;          // 点到点最近邻查找时阈值
        int min_effective_pts_ = 10;            // 最近邻点数阈值
        ///double eps_ = 1e-2;                     // 收敛判定条件
        bool use_initial_translation_ = false;  // 是否使用初始位姿中的平移估计
    };

    Icp2d() {}
    Icp2d(Options options) : options_(options) {}

    /// 设置目标的Scan
    void SetTarget(std::vector<Vec3d>  target) {
        target_ = target;
        //BuildTargetKdTree();

        // 计算点云中心
        target_center_ = std::accumulate(target_.begin(), target_.end(), Vec3d::Zero().eval(),
                                         [](const Vec3d& c, const Vec3d& pt) -> Vec3d { return c + pt; }) /
                         target_.size();
    }

    /// 设置被配准的Scan
    void SetSource(std::vector<Vec3d> source) {
        source_ = source;
        source_center_ = std::accumulate(source_.begin(), source_.end(), Vec3d::Zero().eval(),
                                         [](const Vec3d& c, const Vec3d& pt) -> Vec3d { return c + pt; }) /
                         source_.size();
    }

    void SetGroundTruth(const SE2& gt_pose) {
        gt_pose_ = gt_pose;
        gt_set_ = true;
    }

    int check_matches(std::vector<std::pair<size_t, size_t>>& matches);

    bool pose_estimation_3d3d(
        // Mat3d& R, Vec3d& t
    );
    bool isSourceSet()const {
        return !source_.empty();
    } 



    MovementData Get_Odometry()const{
        return MovementData(0, R_,Vec3d(0, 0, 0), t_);
    }

   private:
    // 建立目标点云的Kdtree
    //void BuildTargetKdTree();

    //std::shared_ptr<sad::KdTree> kdtree_ = nullptr;  // 第5章的kd树

    std::vector<Vec3d>  target_ ;
    std::vector<Vec3d>  source_ ;

    Vec3d target_center_ = Vec3d::Zero();
    Vec3d source_center_ = Vec3d::Zero();

    bool gt_set_ = false;  // 真值是否设置
    SE2 gt_pose_;// the ground  truth: translation & rotaion  that aligns the target to the source

    Options options_;
    Mat3d R_ = Mat3d::Zero();
    Vec3d t_ = Vec3d::Zero();




    // /*  followings are for PCL  ICP  




    // */
};






//  ICP exported by PCL
class pcl_icp {
public:
    pcl_icp() {
        // Configuration of ICP parameters can be done here
        // e.g., setting max iterations, transformation epsilon, etc.
        icp.setMaximumIterations(50);
        // icp.setRANSACOutlierRejectionThreshold(0.05);
        //icp.setMaxCorrespondenceDistance(0.5);
        icp.setEuclideanFitnessEpsilon(0.1);
    }

    void align(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr target
               ) {
        icp.setInputSource(source);
        icp.setInputTarget(target);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
    }

    bool hasConverged() const {
        return icp.hasConverged();
    }

    Eigen::Matrix4f getFinalTransformation() {
        return icp.getFinalTransformation();
    }


    // pcl::PointCloud<pcl::PointXYZ>::Ptr getAlignedTargetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     if (icp.hasConverged()) {
    //         // Applying the final transformation to the target cloud
    //         pcl::transformPointCloud(*target, *aligned_cloud, icp.getFinalTransformation());
    //     }
    //     return aligned_cloud;
    // }

private:
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
};


#endif  // ICP_2D_CLASS