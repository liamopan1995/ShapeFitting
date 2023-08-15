#include <iostream>
#include <vector>
#include "CircleFitByLeastSquare.cpp"
#include <Eigen/Dense>
#include <cmath> 

void fit_plane( Eigen::Matrix<float, Eigen::Dynamic, 3>& P, Eigen::Vector3f& P_mean, Eigen::Vector3f& normal, float& d) {
    /*
    This function takes points and find its centriod, and the normal vector of the plane that most points are close to 
    not sure what if the use of d..
    After running fit_plane , P actually refer to the points been centerized, 
    */

    P_mean = P.colwise().mean(); // Calculate mean along columns

    // simply save P_centered in P instead create a new P_centered object
    // Eigen::Matrix<float, Eigen::Dynamic, 3> P_centered = P.rowwise() - P_mean.transpose();
    // Eigen::JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, 3>> svd(P_centered, Eigen::ComputeFullU | Eigen::ComputeFullV);

    P = P.rowwise() - P_mean.transpose();
    Eigen::JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, 3>> svd(P, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f V = svd.matrixV();

    normal = V.col(2); // Third column of V
    d = -P_mean.dot(normal); // d = -<p,n>

}


/*
The following rodrigues_rot function implementation is not accelerated by vectorization
*/


Eigen::MatrixXf rodrigues_rot_original(const Eigen::MatrixXf& P, const Eigen::Vector3f& n0, const Eigen::Vector3f& n1)
/*
    This function takes and then rotates the points in 3d from the plane define by n0 , 
    into the X-Y plane ,it returns a matrix of shape (n,3)
*/ 
{

    Eigen::MatrixXf P_rot = Eigen::MatrixXf::Zero(P.rows(), 3);

    Eigen::Vector3f n0_norm = -1 * n0.normalized();
    // n1 = [0,0,1] 
    
    // Get the axis around which the rotation is be applied
    Eigen::Vector3f k = n0_norm.cross(n1);
    k.normalize();
    std::cout<<"\ndot product:\n"<<n0_norm.dot(n1)<<std::endl;
    // Get the radian for which the rotation is applied 
    float theta = std::acos(n0_norm[2]); //  has same effort of applying dot product with [0,0,1]


    // Compute the points coords after rotation.
    for (int i = 0; i < P.rows(); ++i) {
        /*
        seeking a way to write it in vectorizd form
        */
        Eigen::Vector3f P_row = P.row(i); // Convert row to a 3D vector
        P_rot.row(i)  = P_row * std::cos(theta) + k.cross(P_row) * std::sin(theta) + k * k.dot(P_row) * (1 - std::cos(theta));
    }


    std::cout<<"\nn0:\n"<<n0_norm<<"\nn1:\n"<<n1<<"\nk:\n"<<k<<"\ntheta:\n"<< theta <<"\n P_rot:\n "<<P_rot<<std::endl;

    return P_rot;
}




Eigen::MatrixXf rodrigues_rot_vec(const Eigen::MatrixXf& P, const Eigen::Vector3f& n0, const Eigen::Vector3f& n1) 
/*
    This function takes and then rotates the points in 3d from the plane define by n0 , 
    into the X-Y plane ,it returns a matrix of shape (n,3)
*/
{
    Eigen::MatrixXf P_rot = Eigen::MatrixXf::Zero(P.rows(), 3);

    Eigen::Vector3f n0_norm = -1 * n0.normalized();
    Eigen::Vector3f k = n0_norm.cross(n1);
    k.normalize();


    float theta = std::acos(n0_norm[2]);
    Eigen::Matrix3f rot_matrix;

    rot_matrix = Eigen::AngleAxisf(theta, k);
    Eigen::MatrixXf P_rot2 = Eigen::MatrixXf::Zero(P.rows(), 3);
    P_rot2 = P * rot_matrix.transpose();

    std::cout<<"\n P_rot2:\n"<<P_rot2<<std::endl;


    Eigen::Matrix3f I = Eigen::Matrix3f::Identity(); // Identity matrix
    Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
    K <<  0, -k(2), k(1),
          k(2), 0, -k(0),
         -k(1), k(0), 0;

    Eigen::Matrix3f R = I + std::sin(theta) * K + (1 - std::cos(theta)) * K * K;

    P_rot = P * R.transpose();

    return P_rot;
}


Eigen::MatrixXf rodrigues_rot_lib(const Eigen::MatrixXf& P, const Eigen::Vector3f& n0, const Eigen::Vector3f& n1) 
/*
    This function takes and then rotates the points in 3d from the plane define by n0 , 
    into the X-Y plane ,it returns a matrix of shape (n,3)
*/
{
    Eigen::MatrixXf P_rot = Eigen::MatrixXf::Zero(P.rows(), 3);
    Eigen::Matrix3f rot_matrix;

    Eigen::Vector3f n0_norm = -1 * n0.normalized();
    Eigen::Vector3f k = n0_norm.cross(n1);
    k.normalize();

    float theta = std::acos(n0_norm[2]);
    rot_matrix = Eigen::AngleAxisf(theta, k);

    P_rot = P * rot_matrix.transpose();
    return P_rot;
}



int main() {
    std::vector<double> x = {1.0, 2.0, 3.0};
    std::vector<double> y = {2.0, 4.0, 6.0};
    std::vector<double> w = {1.0, 0.5, 2.0};

    auto result = CircleFitByLeastSquare(x, y, w);
    // std::cout << "xc: " << std::get<0>(result) << ", yc: " << std::get<1>(result) << ", r: " << std::get<2>(result) << std::endl;



/*
write the cloud points into a matrix

*/
    Eigen::Matrix<double, Eigen::Dynamic, 3> pointsMatrix; // Dynamic-sized matrix for points

    // Simulate adding points dynamically
    for (int i = 0; i < 5; ++i) {
        double x = i * 1.0;
        double y = i * 2.0;
        double z = i * 3.0;
        pointsMatrix.conservativeResize(pointsMatrix.rows() + 1, Eigen::NoChange);
        pointsMatrix.row(pointsMatrix.rows() - 1) << x, y, z;
    }

    // Printing the points matrix
    // std::cout << "Points Matrix:" << std::endl;
    // std::cout << pointsMatrix << std::endl;


/*
    Manually construct a P , P is sampled from python DiskFitting

*/  
    // Points in 3D :P  n=10   radian=" 1 " ;

    // Define the values of x, y, and z
    float x_values[] = {1.01, 0.94, 0.79, 0.48, 0.15, -0.25, -0.46, -0.71, -1.13, -0.96};
    float y_values[] = {0.04, 0.43, 0.65, 0.76, 1.04, 0.87, 1.13, 0.87, 0.43, -0.12};
    float z_values[] = {0.52, 0.78, 0.64, 0.67, 0.36, -0.01, -0.24, -0.31, -0.60, -0.78};

    Eigen::Matrix<float, Eigen::Dynamic, 3> P; // Dynamic-sized matrix for points

    for (int i = 0; i < 10; ++i) {
        P.conservativeResize(P.rows() + 1, Eigen::NoChange);  // Add a row
        // P(P.rows() - 1, 0) = x_values[i];  // Set x values in the first column
        // P(P.rows() - 1, 1) = y_values[i];  // Set y values in the second column
        // P(P.rows() - 1, 2) = z_values[i];  // Set z values in the third column
        P.row(P.rows() - 1) << x_values[i], y_values[i], z_values[i];
    }

    // Printing the matrix P
    std::cout << "Matrix P:\n" << P << std::endl;


    Eigen::Vector3f P_mean = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();
    float d = 0.0;
    fit_plane(P,P_mean,normal,d);
    P_mean = P.colwise().mean().transpose();
    // std::cout << "P_mean:\n " << P_mean.transpose() << std::endl;
    // std::cout << "Normal: " << normal.transpose() << std::endl;
    // std::cout << "d: " << d << std::endl;
    std::cout << "\nMatrix P_centered:\n" << P << std::endl;    

    Eigen::Vector3f n1(0.0, 0.0, 1);
    //Eigen::MatrixXf P_rot = rodrigues_rot_original(P, normal, n1);
    //Eigen::MatrixXf P_rot = rodrigues_rot_vec(P, normal, n1);
    Eigen::MatrixXf P_rot = rodrigues_rot_lib(P, normal, n1);

    std::cout << "\nP_rot:\n" << P_rot << std::endl;


    // Eigen::Matrix<float, Eigen::Dynamic, 3> P_centered = P.rowwise() - P_mean.transpose();



    // std::cout << "P_centered:\n " << P_centered << std::endl;

    // Eigen::JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, 3>> svd(P_centered, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Eigen::Matrix3f V = svd.matrixV();
    // std::cout << "V:\n " << V << std::endl;
    // normal = V.col(2); // Third column of V

    // std::cout << "normal:\n " << normal.transpose() << std::endl;

    // d = -P_mean.dot(normal); // d = -<p,n>

    // std::cout << "d:\n " << d << std::endl;


    return 0;


}








