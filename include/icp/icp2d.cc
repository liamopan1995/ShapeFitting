#include "icp2d.h"


int Icp2d::check_matches(std::vector<std::pair<size_t, size_t>>& matches){
    int i= 0;
    const double MAX_DISTANCE_SQUARED =    6.5;// Value exceeds this threshold will be abandoned.
                                               // otherwise it always choose the pair which has minimal dist.
                                           // 2.5 was great//2.5; we increase the threshold
                                           // to handel the case between two two scans to be matched,
                                           // there are scans discarded for insufficient match number
                                           // it is obvious that to this case the distance between same
                                           // object in two scans of diff time points will be larger
                                           //  it was 2.5 before tunning process at 21 Nov.
                                           //  after 21 Nov. MAX_DISTANCE_SQUARED is set to 6.5,
                                           //  this has significantly mediated the problem
                                           //  of frequently occuring matching failure while it was set to 2.5
                                           //  which was good in unit test but performs bad during online running 

    for(int j=0;j<matches.size();j++){
        double dis = (target_[matches[j].first] - source_[matches[j].second]).squaredNorm();
        if(dis < MAX_DISTANCE_SQUARED){
            matches[i] = matches[j];
            i++;
        } 
    }
    matches = std::vector<std::pair<size_t, size_t>>(matches.begin(),matches.begin()+i);
    // Todo:
    // compute the mean and sigma of distances ,then apply the operation in above one more time
    // this might be a effective way to reject outliers while having a bigger threshold (MAX_DISTANCE_SQUARED)
    // 2. Try the gaussian approach
    return i;
}

bool Icp2d::pose_estimation_3d3d() {
    const int MIN_MATCHED_PAIR = 3;  // 2 is the minimal number of equations to determine 3 unkonw factors
    std::vector<std::pair<size_t, size_t>> matches;
    bfnn_cloud_mt(target_, source_, matches);
    // bfnn_cloud_mt(source_, target_, matches);
    // for every point in target , find a corespond in source
    assert(matches.size() != 0);

    double N_match = check_matches(matches);
    double N_union = (target_.size() + source_.size()) ;
    fitness_ = N_match /  ( N_union - N_match);

    //std::cout<<"N_match: "<<N_match<<"\nN_union: "<<N_union<<"\nfitness_ "<<fitness_<<std::endl;
    if( N_match < MIN_MATCHED_PAIR) {
       
        return  false;
        }

    for (auto pair : matches) {
        // LOG(INFO) << "idx: " << pair.first << " : " << target_[pair.first].transpose();
        // LOG(INFO) << "pa_idx: " << pair.second << " : " << source_[pair.second].transpose();
        // LOG(INFO) << "dis: " << (target_[pair.first] - source_[pair.second]).squaredNorm();
    }

    std::vector<Vec3d> target_c(N_match), source_c(N_match); // holder for centralized points
    Vec3d p1, p2; // center of mass
    for (int i = 0; i < N_match; i++) {
        p1 += target_[matches[i].first].cast<double>();
        p2 += source_[matches[i].second].cast<double>();
    }
    p1 /= N_match;
    p2 /= N_match;
    for (int i = 0; i < N_match; i++) {
        target_c[i] = target_[matches[i].first].cast<double>() - p1;
        source_c[i] = source_[matches[i].second].cast<double>() - p2;
    }

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N_match; i++) {
        W += Eigen::Vector3d(target_c[i][0], target_c[i][1], target_c[i][2]) *
             Eigen::Vector3d(source_c[i][0], source_c[i][1], source_c[i][2]).transpose();
    }
    // LOG(INFO) << "W=" << W;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0) {
        // LOG(INFO) << "determinant v*u <0";
        for (int x = 0; x < 3; ++x) {
            U(x, 2) *= -1;
        }
    }

    // LOG(INFO) << "U=" << U;
    // LOG(INFO) << "V=" << V;

    R_ = U * (V.transpose());
    t_ = Eigen::Vector3d(p1[0], p1[1], p1[2]) - R_ * Eigen::Vector3d(p2[0], p2[1], p2[2]);
    return true;
}
