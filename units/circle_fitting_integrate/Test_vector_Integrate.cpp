#include <iostream>
// #include <vector>
#include <Eigen/Dense>
// #include <cmath> 

#include"CircleFit_Integrate.hpp"



// void fit_plane( Eigen::Matrix<reals, Eigen::Dynamic, 3>& P, Eigen::Vector3f& P_mean, Eigen::Vector3f& normal, reals& d) {
//     /*
//     This function takes points and find its centriod, and the normal vector of the plane that most points are close to 
//     not sure what if the use of d..
//     After running fit_plane , P actually refer to the points been centerized, 
//     */

//     P_mean = P.colwise().mean(); // Calculate mean along columns
//     // std::cout<< "p_mean in fit_plane inside:"<< P_mean<<std::endl;  
//     // simply save P_centered in P instead create a new P_centered object
//     // Eigen::Matrix<reals, Eigen::Dynamic, 3> P_centered = P.rowwise() - P_mean.transpose();
//     // Eigen::JacobiSVD<Eigen::Matrix<reals, Eigen::Dynamic, 3>> svd(P_centered, Eigen::ComputeFullU | Eigen::ComputeFullV);

//     P = P.rowwise() - P_mean.transpose();
//     Eigen::JacobiSVD<Eigen::Matrix<reals, Eigen::Dynamic, 3>> svd(P, Eigen::ComputeFullU | Eigen::ComputeFullV);
//     Eigen::Matrix3f V = svd.matrixV();

//     normal = -1 * V.col(2); // Third column of V
//     d = -P_mean.dot(normal); // d = -<p,n>

// }


// /*
// The following rodrigues_rot function implementation is not accelerated by vectorization
// */


// Eigen::MatrixXf rodrigues_rot_original(const Eigen::MatrixXf& P, const Eigen::Vector3f& n0, const Eigen::Vector3f& n1)
// /*
//     This function takes and then rotates the points in 3d from the plane define by n0 , 
//     into the X-Y plane ,it returns a matrix of shape (n,3)
// */ 
// {

//     Eigen::MatrixXf P_rot = Eigen::MatrixXf::Zero(P.rows(), 3);

//     Eigen::Vector3f n0_norm =  n0.normalized();
//     Eigen::Vector3f n1_norm = n1.normalized();
    
//     // Get the axis around which the rotation is be applied
//     Eigen::Vector3f k = n0_norm.cross(n1_norm);
//     k.normalize();
//     // std::cout<<"\ndot product:\n"<<n0_norm.dot(n1_norm)<<std::endl;
//     // Get the radian for which the rotation is applied 
//     reals theta = std::acos(n0_norm.dot(n1_norm)); //  has same effort of applying dot product with [0,0,1]


//     // Compute the points coords after rotation.
//     for (int i = 0; i < P.rows(); ++i) {
//         /*
//         seeking a way to write it in vectorizd form
//         */
//         Eigen::Vector3f P_row = P.row(i); // Convert row to a 3D vector
//         P_rot.row(i)  = P_row * std::cos(theta) + k.cross(P_row) * std::sin(theta) + k * k.dot(P_row) * (1 - std::cos(theta));
//     }


//     // std::cout<<"\nn0:\n"<<n0_norm<<"\nn1:\n"<<n1_norm<<"\nk:\n"<<k<<"\ntheta:\n"<< theta <<"\n P_rot:\n "<<P_rot<<std::endl;

//     return P_rot;
// }




// Eigen::MatrixXf rodrigues_rot_vec(const Eigen::MatrixXf& P, const Eigen::Vector3f& n0, const Eigen::Vector3f& n1) 
// /*
//     This function takes and then rotates the points in 3d from the plane define by n0 , 
//     into the X-Y plane ,it returns a matrix of shape (n,3)
// */
// {
//     Eigen::MatrixXf P_rot = Eigen::MatrixXf::Zero(P.rows(), 3);
//     Eigen::Vector3f n0_norm =  n0.normalized(); // -1 *
//     Eigen::Vector3f n1_norm = n1.normalized();
//     Eigen::Vector3f k = n0_norm.cross(n1_norm);
//     k.normalize();


//     reals theta = std::acos( n0_norm.dot(n1_norm));
//     Eigen::Matrix3f rot_matrix;

//     rot_matrix = Eigen::AngleAxisf(theta, k);
//     Eigen::MatrixXf P_rot2 = Eigen::MatrixXf::Zero(P.rows(), 3);
//     P_rot2 = P * rot_matrix.transpose();

//     // std::cout<<"\n P_rot2:\n"<<P_rot2<<std::endl;


//     Eigen::Matrix3f I = Eigen::Matrix3f::Identity(); // Identity matrix
//     Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
//     K <<  0, -k(2), k(1),
//           k(2), 0, -k(0),
//          -k(1), k(0), 0;

//     Eigen::Matrix3f R = I + std::sin(theta) * K + (1 - std::cos(theta)) * K * K;

//     P_rot = P * R.transpose();

//     return P_rot;
// }


// Eigen::MatrixXf rodrigues_rot_lib(const Eigen::MatrixXf& P, const Eigen::Vector3f& n0, const Eigen::Vector3f& n1) 
// /*
//     This function takes and then rotates the points in 3d from the plane define by n0 , 
//     into the X-Y plane ,it returns a matrix of shape (n,3)
// */
// {
//     Eigen::MatrixXf P_rot = Eigen::MatrixXf::Zero(P.rows(), 3);
//     Eigen::Matrix3f rot_matrix;

//     Eigen::Vector3f n0_norm =  n0.normalized();
//     Eigen::Vector3f n1_norm = n1.normalized();
//     Eigen::Vector3f k = n0_norm.cross(n1_norm);
//     k.normalize();

//     reals theta = std::acos(n0_norm.dot(n1_norm));
//     rot_matrix = Eigen::AngleAxisf(theta, k);
//     // std::cout<< "rot matrix \n"<<rot_matrix<<std::endl;
//     P_rot = P * rot_matrix.transpose();
//     return P_rot;
// }


// Circle CircleFitting_3D( Eigen::Matrix<reals, Eigen::Dynamic, 3> P)
// /*  
//     A wrapper function, which does follows:

//         1. Take a set of points <x,y,z>  : P 
//         2. Compute its mean : P_mean and norml: normal
//         3. Use one of the the rodrigues_rot function to rotate the points in 3D onto the x-y plane Pxy
//         4. Apply a circle fitting function in 2d , which results at first the fitted circle on x-y plane
//         5. Rerotate the circle back to 3D, and updates its coeffients accordingly.(rodrigues_rot is used again for reversion)

//     This function still can be improved by modifying the ways of passing arguments, by pointers, or references
//     TODO: .. 
// */
// {   
//     // Place holders
//     Circle circle;
//     Eigen::Vector3f P_mean = Eigen::Vector3f::Zero();
//     Eigen::Vector3f normal = Eigen::Vector3f::Zero();
//     Eigen::MatrixXf Pxyz  = Eigen::MatrixXf::Zero(1, 3);
//     Eigen::Vector3f n1(0.0, 0.0, 1);// the normal to xy plane
//     reals d = 0.0;

//     Eigen::Matrix<reals, Eigen::Dynamic, 3> P_ori = P; // original P is modified by Centralization, so save one for computing MSE later
    
//     fit_plane(P,P_mean,normal,d);
    
//     // std::cout<< "P is:"<< P<<endl;
//     Eigen::MatrixXf Pxy = rodrigues_rot_lib(P, normal, n1); 

//     // std::cout<< "Pxy is:"<< Pxy <<endl;

//     //circle = CircleFitByLeastSquare_vectorized (P_rot);
//     circle = CircleFitByHyper_vectorized (Pxy);
//     // std::cout<<" circle.Px, circle.Py:  "<< circle.Px << ","<<circle.Py<<std::endl;
//     //std::cout<< "circle in 2d  is:"<<endl;
//     // circle.print();

//     //std::cout<< "2  . Pxy is:"<< Pxy <<endl;
//     circle.computeMSE_2d(Pxy);   //  MSE 2d  and MSE 3d for well distributed dataset has no much  difference , it is to be experienced if it
//     // is still so for outlier datasets. , after make this clear , we can decide which MSE measure we will use. 

//     // //Remap the circle center into 3d space
//     Pxyz  = rodrigues_rot_original(Eigen::Matrix<reals, 1, 3> (circle.Px, circle.Py, 0) ,n1,  normal);
//     // Check and adjust normal vecotr's orientation , so it always point up towards the sky
//     if(normal(2) < 0) { normal *= -1;}

//     circle.Px = Pxyz(0) + P_mean(0);
//     circle.Py = Pxyz(1) + P_mean(1);
//     circle.Pz = Pxyz(2) + P_mean(2);

//     // write the normal into circle.
//     circle.normal =  normal;


//     // circle.computeMSE(P);

//     circle.computeMSE_3d(P_ori);  

//     return circle;
// }
    


int main() {

/*
    Testing of function with vector as input
*/
    // std::vector<reals> x = {1.0, 2.0, 3.0};
    // std::vector<reals> y = {2.0, 4.0, 6.0};
    // std::vector<reals> w = {1.0, 0.5, 2.0};

    // auto result = CircleFitByLeastSquare(x, y, w);
    // std::cout << "xc: " << std::get<0>(result) << ", yc: " << std::get<1>(result) << ", r: " << std::get<2>(result) << std::endl;



/*
    Code snippet :  Writing the cloud points into a matrix 

*/
    // Eigen::Matrix<reals, Eigen::Dynamic, 3> pointsMatrix; // Dynamic-sized matrix for points

    // // Simulate adding points dynamically
    // for (int i = 0; i < 5; ++i) {
    //     reals x = i * 1.0;
    //     reals y = i * 2.0;
    //     reals z = i * 3.0;
    //     pointsMatrix.conservativeResize(pointsMatrix.rows() + 1, Eigen::NoChange);
    //     pointsMatrix.row(pointsMatrix.rows() - 1) << x, y, z;
    // }

    // Printing the points matrix
    // std::cout << "Points Matrix:" << std::endl;
    // std::cout << pointsMatrix << std::endl;








/*
    Testing of single units:  ( Passed )
                            1.rodrigues_rot_original, 
                            2.rodrigues_rot_vec,
                            3.rodrigues_rot_lib,
                            4.CircleFitByLeastSquare_vectorized
                            5.CircleFitByHyper_vectorized
*/

    // Printing the matrix P
    // std::cout << "Matrix P:\n" << P << std::endl;


    // Eigen::Vector3f P_mean = Eigen::Vector3f::Zero();
    // Eigen::Vector3f normal = Eigen::Vector3f::Zero();
    // reals d = 0.0;
    // fit_plane(P,P_mean,normal,d);
    // P_mean = P.colwise().mean().transpose();
    // std::cout << "\n P_mean:\n" << P_mean << std::endl;  
    // // std::cout << "P_mean:\n " << P_mean.transpose() << std::endl;
    // // std::cout << "Normal: " << normal.transpose() << std::endl;
    // // std::cout << "d: " << d << std::endl;
    // std::cout << "\nMatrix P_centered:\n" << P << std::endl;    

    // Eigen::Vector3f n1(0.0, 0.0, 1);
    // //Eigen::MatrixXf P_rot = rodrigues_rot_original(P, normal, n1);
    // //Eigen::MatrixXf P_rot = rodrigues_rot_vec(P, normal, n1);
    // Eigen::MatrixXf P_rot = rodrigues_rot_lib(P, normal, n1);

    // std::cout << "\nP_rot:\n" << P_rot << std::endl;


    // Eigen::Matrix<reals, Eigen::Dynamic, 3> P_centered = P.rowwise() - P_mean.transpose();



    // std::cout << "P_centered:\n " << P_centered << std::endl;

    // Eigen::JacobiSVD<Eigen::Matrix<reals, Eigen::Dynamic, 3>> svd(P_centered, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Eigen::Matrix3f V = svd.matrixV();
    // std::cout << "V:\n " << V << std::endl;
    // normal = V.col(2); // Third column of V

    // std::cout << "normal:\n " << normal.transpose() << std::endl;

    // d = -P_mean.dot(normal); // d = -<p,n>

    // std::cout << "d:\n " << d << std::endl;

    // Circle circle = CircleFitByLeastSquare_vectorized (P_rot);
    // cout << "\n  LS  fit:  center (" 
    //       << circle.Px <<","<< circle.Py <<")  radius "
    //       << circle.r << "  sigma  modify this later" <<std::endl;

    // Circle circle2 = CircleFitByHyper_vectorized (P_rot);
    //     cout << "\n  Hyper  fit:  center (" 
    //       << circle2.Px <<","<< circle2.Py <<circle2.Pz<<")  radius "
    //       << circle2.r << "  sigma  modify this later" <<std::endl;





/*  
    After testing of each unit was succed , encapsulize them into one single wrapepr function called CircleFitting_3D.
    Test of the wrapper function : CircleFitting_3D 
*/



/*  
    1. Testcase
    Manually construct a P using the snippet above, P is sampled from python DiskFitting

*/  
    // Points in 3D :P  n=10   radian=" 1 " ;

    // Define the values of x, y, and z
    reals x_values[] = {1.01, 0.94, 0.79, 0.48, 0.15, -0.25, -0.46, -0.71, -1.13, -0.96};
    reals y_values[] = {0.04, 0.43, 0.65, 0.76, 1.04, 0.87, 1.13, 0.87, 0.43, -0.12};
    reals z_values[] = {0.52, 0.78, 0.64, 0.67, 0.36, -0.01, -0.24, -0.31, -0.60, -0.78};

    Eigen::Matrix<reals, Eigen::Dynamic, 3> P; // Dynamic-sized matrix for points

    for (int i = 0; i < 10; ++i) {
        P.conservativeResize(P.rows() + 1, Eigen::NoChange);  // Add a row

        P.row(P.rows() - 1) << x_values[i], y_values[i], z_values[i];
    }



/*  
    2. Test case
    Manually construct a P_test_2 using the snippet above, P is sampled from python DiskFitting

*/  
    // Points in 3D :P  n=20   radian=" 0.5 " ;

    //  Ground Truth=" [-1.57088976  4.98581211  3.42316407] 3 ";

    reals test_npx_3[]  {1.13, 1.21, 1.06, 0.83, 0.59, 0.95, 0.59, 0.54, 0.23, 0.09, -0.11, -0.21, -0.27, -0.65, -0.84, -0.99, -1.22, -1.38, -1.84, -2.15};
    reals test_npy_3[]  {4.81, 5.28, 5.35, 5.68, 5.78, 6.36, 6.37, 6.51, 6.87, 6.86, 7.08, 7.44, 7.71, 7.83, 7.95, 7.97, 8.07, 8.09, 8.01, 8.05};
    reals test_npz_3[]  {5.02, 5.10, 5.27, 5.21, 5.07, 5.01, 5.07, 5.06, 4.91, 4.72, 4.84, 4.60, 4.63, 4.28, 4.28, 4.24, 4.15, 3.82, 3.71, 3.49};
    

    Eigen::Matrix<reals, Eigen::Dynamic, 3> P_test_2; // Dynamic-sized matrix for points

    for (int i = 0; i < 20; ++i) {
        P_test_2.conservativeResize(P_test_2.rows() + 1, Eigen::NoChange);  // Add a row

        P_test_2.row(P_test_2.rows() - 1) << test_npx_3[i], test_npy_3[i], test_npz_3[i];
    }

/*  
    3. Test case  (on outlier )
    Manually construct a P_test_2 using the snippet above, P is sampled from python DiskFitting

*/  
    

    reals test_npx_4[]  {10.50, 10.65, 9.77, 11.58, 9.53, 9.54, 10.24, 8.28, 8.99, 9.09, 11.47, 10.07, 9.46, 8.85, 9.40, 9.40, 9.99, 10.82, 10.21, 8.67, 10.74, 9.88, 8.52, 9.54, 10.34, 10.32, 9.32, 11.03, 9.16, 10.33, 9.52, 8.89, 10.81, 9.93, 10.36, 10.36, 9.96, 7.38, 10.09, 10.09, 9.78, 11.48, 9.19, 10.92, 9.47, 10.10, 9.30, 9.61, 10.30, 10.01};
    reals test_npy_4[]  {9.86, 11.52, 9.77, 10.77, 10.54, 9.53, 8.09, 9.44, 10.31, 8.59, 9.77, 8.58, 10.11, 10.38, 9.71, 11.85, 8.94, 8.78, 8.04, 10.20, 10.17, 9.70, 9.28, 11.06, 8.24, 9.61, 10.61, 10.93, 9.69, 10.98, 9.81, 8.80, 11.36, 11.00, 9.35, 11.54, 11.56, 10.82, 9.70, 8.01, 10.36, 9.48, 9.50, 10.33, 10.51, 10.97, 9.67, 8.54, 10.26, 9.77};
    reals test_npz_4[]  {0.61, -0.24, 1.03, 1.57, 0.00, 0.76, -2.25, 1.12, 0.59, -0.06, 0.53, 0.61, 0.11, 0.36, 1.66, -2.01, 0.97, 0.84, 0.80, -0.68, 1.53, -1.22, 0.25, -0.07, -0.44, 0.85, -0.87, -0.09, -1.16, -0.28, -0.75, 0.80, -0.86, -0.02, 1.21, -2.31, 2.94, -0.82, -1.75, -0.63, -0.93, -0.53, -0.06, 0.94, 0.52, -0.38, 2.05, 1.09, -0.50, -0.47};

    

    Eigen::Matrix<reals, Eigen::Dynamic, 3> P_test_3; // Dynamic-sized matrix for points

    for (int i = 0; i < 50 ; ++i) {
        P_test_3.conservativeResize(P_test_3.rows() + 1, Eigen::NoChange);  // Add a row

        P_test_3.row(P_test_3.rows() - 1) << test_npx_4[i], test_npy_4[i], test_npz_4[i];
    }

/*  
    4. Test case  (on outlier )
    Manually construct a P_test_2 using the snippet above, P is sampled from python DiskFitting

*/  


    reals test_npx_5[]  {2.56, 2.18, 1.75, 2.23, 1.93, 1.80, 1.88, 1.51, 1.84, 1.66, 2.09, 1.88, 1.72, 2.06, 2.07, 1.62, 2.32, 2.02, 1.36, 1.51, 1.32, 2.14, 1.77, 1.46, 1.61, 1.46, 1.31, 1.17, 1.57, 1.50, 1.76, 1.07, 1.16, 1.35, 1.65, 0.58, 0.84, 0.80, 0.82, 0.14, 0.65, 0.94, 0.25, 0.47, 0.37, 0.68, 0.36, 0.20, 0.22, 0.20, 0.42, 0.03, 0.30, 0.50, 0.19, 0.31, 0.44, 0.54, 1.00, 0.69, 0.10, 0.71, 0.93, 0.95, 0.93, 0.60, 1.20, 0.82, 0.64, 1.29, 1.10, 1.30, 0.70, 1.13, 1.55, 1.25, 1.67, 1.41, 1.38, 1.42, 1.77, 2.01, 1.88, 2.12, 2.10, 2.14, 1.26, 2.33, 1.67, 2.23, 2.20, 1.42, 1.73, 2.00, 1.57, 1.78, 1.83, 1.95, 1.54, 1.95, 1.93, 2.29, 1.65, 2.61, 2.16, 1.64, 2.06, 1.95, 2.10, 1.91, 2.33, 1.94, 1.30, 1.74, 1.49, 1.67, 1.55, 2.59, 1.59, 1.20, 1.26, 1.59, 1.67, 1.34, 1.53, 1.25, 1.78, 1.27, 1.76, 1.14, 1.22, 0.80, 0.79, 0.88, 0.76, 0.97, 0.96, 0.80, 0.43, 0.54, 0.59, 0.78, 0.64, 0.15, 0.19, 0.37, 0.01, 0.01, 0.07, 0.14, 0.01, 0.11, 0.10, 0.46, 0.42, 0.62, 1.05, 0.73, 0.94, 0.38, 0.33, 1.26, 0.70, 0.76, 0.29, 0.59, 1.00, 1.36, 1.17, 0.88, 1.64, 1.88, 1.07, 1.19, 1.01, 1.40, 1.41, 1.56, 1.64, 2.11, 1.89, 1.02, 1.86, 2.07, 1.50, 1.51, 1.58, 1.77, 1.76, 2.06, 2.02, 2.49, 1.65, 1.66, 2.51, 2.55, 1.66, 1.89, 1.89, 2.47};
    reals test_npy_5[]  {0.32, 0.57, 1.45, 1.29, 1.52, 2.28, 1.89, 2.30, 2.93, 2.36, 2.85, 2.97, 3.06, 2.82, 3.59, 4.25, 2.97, 3.80, 3.82, 4.03, 3.54, 3.93, 4.30, 3.95, 4.68, 4.34, 4.38, 4.45, 4.83, 4.59, 4.49, 4.01, 4.81, 4.62, 4.96, 4.85, 5.17, 4.55, 4.67, 4.60, 4.72, 4.51, 4.63, 5.36, 4.90, 5.67, 4.40, 5.16, 4.36, 5.19, 5.49, 5.17, 5.51, 5.05, 4.65, 4.69, 4.68, 5.14, 4.55, 5.08, 4.76, 4.71, 4.86, 5.02, 4.93, 4.93, 4.80, 4.57, 4.54, 4.35, 4.70, 4.74, 4.13, 4.74, 3.88, 4.31, 4.27, 3.99, 3.88, 3.13, 3.82, 3.64, 3.64, 3.13, 3.13, 3.30, 3.47, 3.48, 2.95, 3.34, 2.62, 2.67, 2.42, 2.30, 2.09, 1.50, 1.77, 1.77, 1.41, 0.70, -0.34, -1.57, -1.85, -1.21, -2.39, -2.10, -2.32, -2.32, -2.51, -3.44, -3.16, -2.85, -3.47, -3.06, -3.39, -3.36, -3.36, -3.71, -4.21, -3.92, -4.21, -3.95, -4.28, -4.03, -3.87, -4.36, -4.01, -4.64, -3.94, -4.72, -5.10, -4.52, -5.15, -4.78, -4.68, -5.00, -5.19, -4.23, -4.55, -4.67, -5.20, -4.59, -4.95, -5.07, -5.28, -5.14, -4.82, -5.01, -5.19, -4.79, -5.25, -4.57, -4.99, -4.64, -5.08, -5.15, -5.18, -5.19, -5.63, -5.43, -4.71, -5.07, -4.46, -4.72, -4.56, -4.74, -5.01, -4.49, -4.45, -5.03, -4.26, -4.45, -4.48, -4.31, -4.65, -4.03, -3.75, -4.14, -3.84, -4.25, -4.25, -4.15, -3.40, -4.05, -3.70, -3.50, -2.96, -3.14, -2.78, -2.50, -2.38, -2.47, -2.81, -2.15, -2.12, -1.75, -1.79, -1.25, -0.88, 0.33};
    reals test_npz_5[]  {0.46, 0.67, -0.60, -0.10, -0.89, 0.94, 0.78, -0.07, 1.18, -0.26, -1.47, -1.03, -0.91, 0.76, -0.37, -1.24, 0.19, -0.62, 0.98, -1.53, -0.20, 1.56, 1.23, 0.58, -0.32, 0.40, 0.14, 0.49, -1.22, -0.73, 0.56, -0.74, -0.35, 0.23, 0.41, -0.26, 0.18, -0.95, -1.52, 0.96, 0.75, 1.17, 1.82, 0.23, 1.59, 0.24, -1.29, -0.95, -0.06, 0.02, -0.26, 0.13, -1.53, -0.15, 1.17, 1.26, 0.93, -0.58, 0.14, -1.67, -0.73, 1.18, -0.59, 0.39, -1.17, 0.43, 0.00, 1.11, -0.06, -1.06, -0.30, 1.08, -0.18, 0.49, 0.48, -1.07, -0.75, -1.26, 0.46, -0.24, 0.05, -1.98, -1.19, 0.51, -1.05, -0.72, 1.64, 0.66, -0.27, -0.27, -0.27, -0.68, -0.05, -0.45, 0.14, -0.01, 1.92, -1.20, 0.09, 0.52, 1.20, -0.88, -0.63, -0.54, -1.67, -0.26, 0.36, -0.50, -1.07, -0.84, -1.11, -1.28, 0.33, -1.77, 0.10, 0.19, 0.25, 0.82, 0.04, -0.40, -0.34, -0.67, -0.19, 0.27, -2.30, -0.33, -0.37, 0.42, 0.46, -1.46, 1.76, -0.09, -1.72, 0.19, 0.89, -0.11, 0.17, 0.37, 0.58, -0.86, -1.06, 0.87, 0.00, -0.17, -0.42, 1.46, 1.17, -0.25, 0.31, 1.12, -0.23, 1.09, -0.23, 0.02, 1.68, -1.84, 0.16, 0.20, -0.95, 0.51, -1.79, -0.47, -0.36, -0.24, 0.08, -0.14, -1.78, 0.53, -1.16, 0.33, -0.34, 1.27, 0.10, 0.87, 0.08, 0.44, -0.97, 0.11, 0.12, 0.23, 1.52, 0.38, 0.06, 0.04, -0.34, 0.40, 1.08, -0.13, 0.58, -1.42, -0.63, -0.50, -0.31, 0.75, -0.75, 1.09, 0.46, -0.09, 0.38, 0.51};

    Eigen::Matrix<reals, Eigen::Dynamic, 3> P_test_4; // Dynamic-sized matrix for points

    for (int i = 0; i < 200 ; ++i) {
        P_test_4.conservativeResize(P_test_4.rows() + 1, Eigen::NoChange);  // Add a row

        P_test_4.row(P_test_4.rows() - 1) << test_npx_5[i], test_npy_5[i], test_npz_5[i];
    }

    
//    *************************    Test   *************************

    std::cout<< "\n *******   *******   ******* Notes:   *******  *******  *******: \n"<<std::endl;
    std::cout<< "\n *******   *******   A Convention for convenience : normal should always point up towards sky (backwards to the earth)  *******  *******: \n\n\n\n"<<std::endl;
    std::cout<< "\n *******   *******   ******* Following is test 1   *******  *******  *******: \n"<<std::endl;

    Circle circle = CircleFitting_3D(P);

    circle.print();
    
    std::cout<< "\n *******   *******   *******   Following is test 2   *******   *******   *******: \n"<<std::endl;
    Circle circle_test_2 = CircleFitting_3D(P_test_2);

    circle_test_2.print();


    std::cout<< "\n *******   *******   *******   Following is test 3  ( test on outlier dataset)   *******   *******   *******: \n"<<std::endl;
    Circle circle_test_3 = CircleFitting_3D(P_test_3);

    circle_test_3.print();

    std::cout<< "\n *******   *******   *******   Following is test 4  ( test on outlier dataset)   *******   *******   *******: \n"<<std::endl;
    Circle circle_test_4 = CircleFitting_3D(P_test_4);

    circle_test_4.print();
    return 0;
    //  seems mse_3d  is more sensitive towards outliers , but it could be caused by the difference shift used in regular data and outlier data
    //  thus for now , we only could assume mse_3d is slightly better. 


}








