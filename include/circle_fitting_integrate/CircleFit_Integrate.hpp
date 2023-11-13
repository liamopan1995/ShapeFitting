#ifndef CIRCLE_H
#define CIRCLE_H

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <vector>
using namespace std;

typedef float reals; 


const reals One=1.0,Two=2.0,Three=3.0,Four=4.0,Five=5.0,Six=6.0,Ten=10.0;
//const reals One=1.0L,Two=2.0L,Three=3.0L,Four=4.0L,Five=5.0L,Six=6.0L,Ten=10.0L;
const reals Pi=3.141592653589793238462643383L;
const reals REAL_MAX=numeric_limits<reals>::max();
const reals REAL_MIN=numeric_limits<reals>::min();
const reals REAL_EPSILON=numeric_limits<reals>::epsilon();


/*
Struct definition
*/
struct Tree_info {
    /// For the maintenance of fitted tree object:
    /// Usage:
    ///     vector<Single_scan> scans;
    ///     Single_scan single_scan(time_stamp); // Initializing with time_stamp
    ///     single_scan.tree_infos_.push_back(Stem.Get_tree_info());
    ///     scans.push_back(single_scan)
    ///   
    reals x_;
    reals y_;
    reals r_;
    ///  add other fileds when necessary , e.g: normal vector for 3d mathing,  but we first focus on 2d scan mathicng.
    ///  don forget to modify the function: Get_tree_info as the field of Tree_info has been updated..
    Tree_info() 
        : x_(0.0), y_(0.0), r_(0.0) {} 
    
    Tree_info(reals x, reals y, reals r )
        : x_(x), y_(y), r_(r) {} 
};

struct Single_scan {
    /// For the maintenance of fitted tree object:
    /// Usage:
    ///     vector<Single_scan> scans;
    ///     Single_scan single_scan(time_stamp); // Initializing with time_stamp
    ///     single_scan.tree_infos_.push_back(Stem.Get_tree_info());
    ///     scans.push_back(single_scan)
    ///     
    double time_stamp_ ;
    std::vector<Tree_info> tree_infos_;
    Single_scan() 
        : time_stamp_(-1), tree_infos_() {}    
    Single_scan(double time_stamp) 
        : time_stamp_(time_stamp), tree_infos_() {}
};





/*
Class definition
*/
class Circle
{
public:
    // MSE Threshold
    static constexpr reals MSE_MAX = 0.56;// =0.1

	// The fields of a Circle
	reals Px, Py, Pz, r, s;  // (Px,Py,Pz) : center coordinates , r: radius , s : MSE value
	Eigen::Vector3f  normal;// norm Vector
	//Eigen::Vector3f  u;    // Orthonormal vectors n, u, <n,u>=0  for generating points on the circle in space from its coefficients.

	// constructors
	Circle();
	Circle(reals aa, reals bb, reals cc, reals rr);
    // Move constructor
    Circle(Circle&& other) noexcept;
    // Move assignment operator
    Circle& operator=(Circle&& other) noexcept ;



	// helpers
	void computeMSE_3d(const Eigen::Matrix<float, Eigen::Dynamic, 3>& dataPoints);
	void computeMSE_2d(const Eigen::MatrixXf & dataPoints);
	Eigen::Matrix<float, Eigen::Dynamic, 3> generate_circle(void);

	// routines
	void print(void);

	// no destructor we didn't allocate memory by hand.
};




class Stem
{
public:

    // The fields of a Stem
    std::vector<Circle> vector_circles;
    int num_circle;
	reals Px_av, Py_av, Pz_av,r_av, s_av;  // Aveeraged(Px,Py,Pz) : center coordinates , r: radius , s : MSE value
    reals Px_sd, Py_sd, Pz_sd,r_sd, s_sd;  
    reals Px_bottom,Py_bottom,Pz_bottom;
    reals Px_top,Py_top,Pz_top;
	Eigen::Vector3f  normal_av; 

    // Constructors
    Stem() : num_circle(0),Px_av(0), Py_av(0), Pz_av(0), r_av(0), s_av(0), 
    Px_bottom(0),Py_bottom(0),Pz_bottom(std::numeric_limits<reals>::max()),Px_top(0),Py_top(0),Pz_top(std::numeric_limits<reals>::lowest()),
    Px_sd(0), Py_sd(0), Pz_sd(0),r_sd(0), s_sd(0) {
    }



    void pushCircle(Circle&& circle) {
        vector_circles.push_back(std::move(circle));
        num_circle++;
    }

	// helpers
    void get_averages(int MIN_NUM_CIRCLE=6);
    void get_standard_deviation(int MIN_NUM_CIRCLE=6);
    
    /// "  Get_tree_info() "
    /// Helper that fetchs necessary data that represent a stem , construct them as vector and return
    /// Usage:  
    /// Defined as members  in Cloudsegmentation class:     
    ///         vector<   vector< vector<float>>  > scans;
    ///         vector<   vector<float>  > single_scan;
    /// During class Cloudsegmentation's initialization : 
    ///         scans.clear()
    /// Everytime after a new scan arriaves : 
    ///         single_scan.clear()
    /// In processing , once a fitting of stem is valid: do:
    ///         single_scan.push_back( Stem.Get_tree_info);
    /// At the end  of this callback :
    ///         scans.push_back (single_scan);
    /// It is better to have another struct tree to hold these values, instead of using vector, easier for achieving data and data
    /// management i.e. by single_scan[i].rx,  single_scan[i].ry, instead of by single_scan[i][0] etc..---> 
    /// ---> Srtuct Tree_info

    Tree_info Get_tree_info() const{ 
        return Tree_info (Px_av, Py_av, r_av);
    }

	// routines
	void print(void);

	// no destructor we didn't allocate memory by hand.
};


/************************************************************************
			BODY OF THE MEMBER ROUTINES
************************************************************************/


// Default constructor

Circle::Circle()
{
	Px=0.; Py=0.;Pz=0.; r=1.; s=0.; 
}

// Constructor with assignment of the circle parameters only

Circle::Circle(reals aa, reals bb, reals cc, reals rr)
{
	Px=aa; Py=bb; Pz=cc;r=rr;
}



void Circle::computeMSE_3d(const Eigen::Matrix<float, Eigen::Dynamic, 3>& dataPoints)
/*
This function computes the Mean Squared Error (MSE) given a set of data points and an estimated circle.

Formula : MSE = mean (  sqrt (  sqrt( (P-center)^2) -radius )^2 ) 

Parameters:
	dataPoints (Eigen::Matrix<float, Eigen::Dynamic, 3>&): A matrix containing data points, where each row represents a point with (x, y, z) coordinates.
*/
{

    Eigen::Vector3f center(Px, Py, Pz);
    Eigen::Matrix<float, Eigen::Dynamic, 1> squaredDistances = (dataPoints.rowwise() - center.transpose()).array().square().rowwise().sum();

    s  = (squaredDistances.array().sqrt() - r).array().square().sqrt().mean();
    // Print for testing
    // std::cout<<"\n MSE 3d :\n"<<(squaredDistances.array().sqrt() - r).array().square().sqrt().mean();
	


}




// Move constructor
Circle::Circle(Circle&& other) noexcept
        : Px(other.Px), Py(other.Py), Pz(other.Pz), r(other.r), s(other.s),
          normal(std::move(other.normal)) {
        // Other resource transfers if needed
    }

// Move assignment operator
Circle& Circle::operator=(Circle&& other) noexcept {
    if (this != &other) {
        Px = other.Px;
        Py = other.Py;
        Pz = other.Pz;
        r = other.r;
        s = other.s;
        normal = std::move(other.normal);
        // Other resource transfers if needed
    }
    return *this;
}



void Circle::computeMSE_2d(const Eigen::MatrixXf & dataPoints)
/*
This function computes the Mean Squared Error on rotated plane (MSE) given a set of data points and an estimated circle.

MSE = mean (  sqrt (  sqrt( (P-C)^2) -r )^2 )

Parameters:
	dataPoints (Eigen::Matrix<float, Eigen::Dynamic, 3>&): A matrix containing points in 2d shape <n, 2>
*/
{
    
    Eigen::Vector2f center(Px, Py);
    Eigen::Matrix<float, Eigen::Dynamic, 1> squaredDistances = (dataPoints.leftCols<2>().rowwise() - center.transpose()).array().square().rowwise().sum();

    // Calculate the square root of each squared distance, then calculate the mean.
    // s  = (squaredDistances.array().sqrt() - r).array().square().sqrt().mean();

    // Print for testing
	// std::cout<<"\n MSE 2d :\n"<<(squaredDistances.array().sqrt() - r).array().square().sqrt().mean()<<std::endl;

}



// Printing routine

void Circle::print(void)
{
	cout << "\n      Circle info:\n"<<endl;
	cout << setprecision(10) << "center (" <<Px <<","<< Py <<","<< Pz<<")  radius " 
	<< r << "  MSE: " << s << endl;
	cout <<"normal vector\n"<< normal << endl;
}



// 2D fitting algorithms

Circle CircleFitByLeastSquare_vectorized(Eigen::MatrixXf P)
/*
  algorithm :Least Square
  source : 
*/
{
    Circle circle;
    Eigen::MatrixXf A (P.rows(),3);// A = P , but with P's all element in its third colum set to 1  
    Eigen::VectorXf b(P.rows());// = P first colum's suqrare + P second colum's suqrare

    A.leftCols<2>() = P.leftCols<2>(); // Copy first two columns of P to A
    A.col(2).setOnes(); // Set third column of A to 1

    b = P.leftCols<2>().array().square().rowwise().sum(); // Calculate sum of squares for each row

    // Solve by method of least squares
    Eigen::VectorXf c = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);


    circle.Px = c(0) / 2.0;
    circle.Py = c(1) / 2.0;
    circle.r = std::sqrt(c(2) + circle.Px * circle.Px + circle.Py * circle.Py);

    
    return circle;
}

Circle CircleFitByHyper_vectorized (Eigen::MatrixXf P)
/*
  algorithm :Hyper fit  by N. Chernov
  updated from source : https://people.cas.uab.edu/~mosya/cl/CircleFitByHyper.cpp
  original code was not written by using Eigen
*/

{
    int i,iter,IterMAX=99;

    float Mz,Mxy,Mxx,Myy,Mxz,Myz,Mzz,Cov_xy,Var_z;
    float A0,A1,A2,A22;
    float Dy,xnew,x,ynew,y;
    float DET,Xcenter,Ycenter;
    
    Circle circle;

    Eigen::VectorXf Xi(P.rows()) ;
    Eigen::VectorXf Yi(P.rows()) ;
    Eigen::VectorXf Zi(P.rows()) ;

    Eigen::Vector3f P_mean = P.colwise().mean();  // shape <3,1>

    //P.colwise() -= P_mean; wrong!
    P.rowwise() -= P_mean.transpose(); // Centerize along the row axis   <n,3 > - <3,1>.Transpose

    Xi = P.col(0);
    Yi = P.col(1);
    Zi = P.leftCols<2>().array().square().rowwise().sum();


    Mxy = Xi.dot(Yi) /P.rows();
    Mxx = Xi.dot(Xi) /P.rows();
    Myy = Yi.dot(Yi) /P.rows();
    Mxz = Xi.dot(Zi) /P.rows();
    Myz = Yi.dot(Zi) /P.rows();
    Mzz = Zi.dot(Zi) /P.rows();

//    computing the coefficients of the characteristic polynomial

    Mz = Mxx + Myy;
    Cov_xy = Mxx*Myy - Mxy*Mxy;
    Var_z = Mzz - Mz*Mz;

    A2 = Four*Cov_xy - Three*Mz*Mz - Mzz;
    A1 = Var_z*Mz + Four*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
    A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
    A22 = A2 + A2;

//    finding the root of the characteristic polynomial
//    using Newton's method starting at x=0  
//     (it is guaranteed to converge to the right root)

	for (x=0.,y=A0,iter=0; iter<IterMAX; iter++)  // usually, 4-6 iterations are enough
    {
        Dy = A1 + x*(A22 + 16.*x*x);
        xnew = x - y/Dy;
        if ((xnew == x)||(!isfinite(xnew))) break;
        ynew = A0 + xnew*(A1 + xnew*(A2 + Four*xnew*xnew));
        if (abs(ynew)>=abs(y))  break;
        x = xnew;  y = ynew;
    }

//    computing paramters of the fitting circle

    DET = x*x - x*Mz + Cov_xy;
    Xcenter = (Mxz*(Myy - x) - Myz*Mxy)/DET/Two;
    Ycenter = (Myz*(Mxx - x) - Mxz*Mxy)/DET/Two;

//       assembling the output

    circle.Px = Xcenter + P_mean(0);
    circle.Py = Ycenter + P_mean(1);
    circle.r = sqrt(Xcenter*Xcenter + Ycenter*Ycenter + Mz - x - x);
    
    return circle;
}





// Ultilities for applying circle fitting in 3d space

void fit_plane( Eigen::Matrix<float, Eigen::Dynamic, 3>& P, Eigen::Vector3f& P_mean, Eigen::Vector3f& normal) 
/*
This function takes points and find its centriod, then centralize the P by it, and find the normal vector of the plane that most points are close to 
*/
{
    // Centralizing
    P_mean = P.colwise().mean(); // Calculate mean along columns
    P = P.rowwise() - P_mean.transpose();
    
    // Find the normal of the plane , which is most close to all the points
    Eigen::JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, 3>> svd(P, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f V = svd.matrixV();
    normal = -1 * V.col(2); // Third column of V

}



/*
            The following three rodrigues_rot function implementations just do the same thing.
            only their implementations differ, 
            but when input size is big , it is prefered to call the ___lib ,oppsites it is better to call ___original.
*/


Eigen::MatrixXf rodrigues_rot_original(const Eigen::MatrixXf& P, const Eigen::Vector3f& n0, const Eigen::Vector3f& n1)
/*
    This function takes and then rotates the points in 3d from the plane defined by n0 , 
    onto the X-Y plane ,it returns a matrix of shape (n,3)
*/
{

    Eigen::MatrixXf P_rot = Eigen::MatrixXf::Zero(P.rows(), 3);

    Eigen::Vector3f n0_norm =  n0.normalized();
    Eigen::Vector3f n1_norm = n1.normalized();
    
    // Get the axis around which the rotation is be applied
    Eigen::Vector3f k = n0_norm.cross(n1_norm);
    k.normalize();

    // Get the radian for which the rotation is applied 
    float theta = std::acos(n0_norm.dot(n1_norm)); //  has same effort of applying dot product with [0,0,1]

    // Compute the points coords after rotation.
    for (int i = 0; i < P.rows(); ++i) {
        /*
        seeking a way to write it in vectorizd form
        */
        Eigen::Vector3f P_row = P.row(i); // Convert row to a 3D vector
        P_rot.row(i)  = P_row * std::cos(theta) + k.cross(P_row) * std::sin(theta) + k * k.dot(P_row) * (1 - std::cos(theta));
    }

    return P_rot;
}




Eigen::MatrixXf rodrigues_rot_vec(const Eigen::MatrixXf& P, const Eigen::Vector3f& n0, const Eigen::Vector3f& n1) 
/*
    This function takes and then rotates the points in 3d on the plane defined by n0 , 
    onto the X-Y plane ,it returns a matrix of shape (n,3)
*/
{
    Eigen::MatrixXf P_rot = Eigen::MatrixXf::Zero(P.rows(), 3);
    Eigen::Vector3f n0_norm =  n0.normalized(); // -1 *
    Eigen::Vector3f n1_norm = n1.normalized();
    Eigen::Vector3f k = n0_norm.cross(n1_norm);
    k.normalize();


    float theta = std::acos( n0_norm.dot(n1_norm));
    Eigen::Matrix3f rot_matrix;

    rot_matrix = Eigen::AngleAxisf(theta, k);
    Eigen::MatrixXf P_rot2 = Eigen::MatrixXf::Zero(P.rows(), 3);
    P_rot2 = P * rot_matrix.transpose();

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
    This function takes and then rotates the points in 3d from the plane defined by n0 , 
    onto the X-Y plane ,it returns a matrix of shape (n,3)
*/
{
    Eigen::MatrixXf P_rot = Eigen::MatrixXf::Zero(P.rows(), 3);
    Eigen::Matrix3f rot_matrix;

    Eigen::Vector3f n0_norm =  n0.normalized();
    Eigen::Vector3f n1_norm = n1.normalized();
    Eigen::Vector3f k = n0_norm.cross(n1_norm);
    k.normalize();

    float theta = std::acos(n0_norm.dot(n1_norm));
    rot_matrix = Eigen::AngleAxisf(theta, k);
    // std::cout<< "rot matrix \n"<<rot_matrix<<std::endl;
    P_rot = P * rot_matrix.transpose();
    return P_rot;
}




Circle CircleFitting_3D( Eigen::Matrix<float, Eigen::Dynamic, 3> P)
/*
    A wrapper function, which does so by the following steps:

        1. Take a set of points <x,y,z>  : P 
        2. Compute its mean : P_mean and norml: normal. and centralizing P
        3. Use one of the the rodrigues_rot function to rotate the points in 3D onto the x-y plane Pxy
        4. Apply a circle fitting function in 2d , which results  fitted circle' coefficients on the x-y plane
        5. Rerotate the circle back to 3D, and updates its coeffients accordingly.(rodrigues_rot is used again for this procedure)
        6. Ajust its normal so it alway points up towards the sky ,and write it into the circle
        7. Compute the fitted circle's MSE value and save it.
*/
{   
    // Place holders
    Circle circle;
    Eigen::Vector3f P_mean = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();
    Eigen::MatrixXf Pxyz  = Eigen::MatrixXf::Zero(1, 3);
    Eigen::Vector3f n1(0.0, 0.0, 1);// the normal to xy plane

    // Original P will be modified by Centralization, so save one for computing MSE later
    Eigen::Matrix<float, Eigen::Dynamic, 3> P_ori = P; 
    
    // Centeralizing the Points in 3D and compute its normal
    fit_plane(P,P_mean,normal);
    
    // Rotate the points in 3d onto x y plane.
    Eigen::MatrixXf Pxy = rodrigues_rot_lib(P, normal, n1); 

    // Fit a circle on the points rotated onto xy plane, by either of the two 2d fitting fucntion.
    circle = CircleFitByHyper_vectorized (Pxy);
    //circle = CircleFitByLeastSquare_vectorized (Pxy);

    circle.computeMSE_2d(Pxy);   //  MSE 2d  and MSE 3d for well distributed dataset has no much  difference , but with experiments made,
    // MSE 3d tends to have larger value on outlier dataset, but it is not clear whether this is purly due to the outlier datasets have
    // a larger z directional variance  

    // Remap the circle center back into 3d space
    Pxyz  = rodrigues_rot_original(Eigen::Matrix<float, 1, 3> (circle.Px, circle.Py, 0) ,n1,  normal);

    
    // Shift the center back to its postion from the center
    circle.Px = Pxyz(0) + P_mean(0);
    circle.Py = Pxyz(1) + P_mean(1);
    circle.Pz = Pxyz(2) + P_mean(2);

    // Check and adjust normal vecotr's orientation , so it always point up towards the sky
    // And write the normal into circle.
    if(normal(2) < 0) { normal *= -1;}
    circle.normal =  normal;

    // circle.computeMSE(P);
    // As MSE_3d appears slightly better so far, we will use mse_3d to compute the fitted circles mse
    circle.computeMSE_3d(P_ori);  

    return circle;
}





void Stem::get_averages(int MIN_NUM_CIRCLE){
    if (num_circle >MIN_NUM_CIRCLE){
        for(const auto& circle: vector_circles)
        {
            Px_av += circle.Px;
            Py_av += circle.Py;
            Pz_av += circle.Pz;
            r_av += circle.r;
            s_av += circle.s; 
            if(circle.Pz>Pz_top)
            {
                Pz_top=circle.Pz;
                Px_top=circle.Px;Py_top=circle.Py;     
            }
            if(circle.Pz<Pz_bottom)
            {
                Pz_bottom=circle.Pz;
                Px_bottom=circle.Px;Py_bottom=circle.Py;
            }
        }
        Px_av /=num_circle;
        Py_av /=num_circle;
        Pz_av /=num_circle;
        r_av /=num_circle;
        s_av /=num_circle;
    }

}

void Stem::get_standard_deviation(int MIN_NUM_CIRCLE) {
    if (num_circle > MIN_NUM_CIRCLE) {
        for (const auto& circle : vector_circles) {
            Px_sd += (circle.Px - Px_av) * (circle.Px - Px_av);
            Py_sd += (circle.Py - Py_av) * (circle.Py - Py_av);
            Pz_sd += (circle.Pz - Pz_av) * (circle.Pz - Pz_av);
            r_sd += (circle.r - r_av) * (circle.r - r_av);
            s_sd += (circle.s - s_av) * (circle.s - s_av);
        }

        Px_sd = std::sqrt(Px_sd / (num_circle-1));
        Py_sd = std::sqrt(Py_sd / (num_circle-1));
        Pz_sd = std::sqrt(Pz_sd / (num_circle-1));
        r_sd = std::sqrt(r_sd / (num_circle-1));
        s_sd = std::sqrt(s_sd / (num_circle-1));
    }
}

// Printing routine
void Stem::print(void)
{
	cout << "\n      Stem info:\n"<<endl;
    cout<<  "num of circles in this stem:\n"<<num_circle<<endl;
	cout << setprecision(10) << "\naveraged center (" <<Px_av <<","<< Py_av <<","<< Pz_av<<")  ave radius " 	<< r_av 
    << "\nave MSE: " << s_av
    <<"\n standard deviation center: (" <<Px_sd <<","<< Py_sd <<","<< Pz_sd<<")  sd radius " 	<< r_sd 
    << "\nsd MSE: " << s_sd
    << "\n top center:(" <<Px_top <<","<< Py_top <<","<< Pz_top<<")"
    << "\n bottom center:(" <<Px_bottom <<","<< Py_bottom <<","<< Pz_bottom<<")"<<endl;
    
	// cout <<"normal vector\n"<< normal << endl;
}


#endif // CIRCLE_H