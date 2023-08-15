#include "mystuff.h"
#include "data.h"
#include "circle.h"
#include "Utilities.cpp"
#include "CircleFitByTaubin.cpp"
#include "CircleFitByPratt.cpp"
#include "CircleFitByKasa.cpp"
#include "CircleFitByHyper.cpp"
#include "CircleFitByLeastSquare.cpp"
#include <time.h>

inline void simpletest (Data& data, const string& radian, const string& GR, int n)
{
     Circle circle;
     cout.precision(7);
     circle = CircleFitByKasa (data);
     cout << "\n Test from numpy data";
     cout << "\n Num of samples:  "<< n << "    radian: "<<radian<<"Ground truth:  "<<GR;
     cout << "\n  Pratt  fit:  center (" 
          << circle.a <<","<< circle.b <<")  radius "
          << circle.r << "  sigma " << circle.s << endl;

     circle = CircleFitByPratt (data);
     cout << "\n  Pratt  fit:  center (" 
          << circle.a <<","<< circle.b <<")  radius "
          << circle.r << "  sigma " << circle.s << endl;

     circle = CircleFitByTaubin (data);
     cout << "\n  Taubin fit:  center (" 
          << circle.a <<","<< circle.b <<")  radius "
          << circle.r << "  sigma " << circle.s << endl;

     circle = CircleFitByHyper (data);
     cout << "\n  Hyper  fit:  center (" 
          << circle.a <<","<< circle.b <<")  radius "
          << circle.r << "  sigma " << circle.s << endl;
     circle = CircleFitByLeastSquare (data);
     cout << "\n  LS  fit:  center (" 
          << circle.a <<","<< circle.b <<")  radius "
          << circle.r << "  sigma " << circle.s << endl;
}

int main()
//             this code tests algebraic circle fits
{    
    std::string GR,radian; 
    Data data;
    int n;
    reals BenchmarkExampleDataX[6] {1.,2.,5.,7.,9.,3.};
    reals BenchmarkExampleDataY[6] {7.,6.,8.,7.,5.,7.};

    Data data1(6,BenchmarkExampleDataX,BenchmarkExampleDataY);
    Circle circle;
    cout.precision(7);
/*
       Test One:  benchmark example from the journal paper
    
       W. Gander, G. H. Golub, and R. Strebel,
       "Least squares fitting of circles and ellipses"
       BIT, volume 34, (1994), pages 558-578
          
          Correct answers:
          
  Kasa   fit:  center (4.921166,3.835123)  radius 4.123176  sigma 0.4800269

  Pratt  fit:  center (4.615482,2.807354)  radius 4.911302  sigma 0.4610572

  Taubin fit:  center (4.613933,2.795209)  radius 4.879213  sigma 0.4572958

  Hyper  fit:  center (4.615482,2.807354)  radius 4.827575  sigma 0.4571757
*/
    circle = CircleFitByKasa (data1);
    cout << "\nTest One:\n  Kasa   fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByPratt (data1);
    cout << "\n  Pratt  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByTaubin (data1);
    cout << "\n  Taubin fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByHyper (data1);
    cout << "\n  Hyper  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    circle = CircleFitByLeastSquare (data1);
    cout << "\n  LS  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
//            Test Two:  a randomly generated data set
    
    Data data2(10);    //   specify the number of data points
    
//          use the c++ random number generator to simulate data coordinates
    
    srand ( (unsigned)time(NULL) );  //  seed the random generator
    SimulateRandom (data2,1.0);       //  this function is in Utilities.cpp

    circle = CircleFitByKasa (data2);
    cout << "\nTest Two:\n  Kasa   fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByPratt (data2);
    cout << "\n  Pratt  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByTaubin (data2);
    cout << "\n  Taubin fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByHyper (data2);
    cout << "\n  Hyper  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    circle = CircleFitByLeastSquare (data2);
    cout << "\n  LS  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
/*
       Test Thee:  benchmark example from the journal paper
    
       W. Pratt, "Direct least-squares fitting of algebraic surfaces"
       Computer Graphics, volume 21, (1987), pages 145-152.
       
       It demonstrates that Kasa fit may grossly underestimate the circle size
          
          Correct answers:
          
  Kasa   fit:  center (0.0083059,-0.724455)  radius 1.042973  sigma 0.2227621

  Pratt  fit:  center (0.4908357,-22.15212)  radius 22.18006  sigma 0.05416545

  Taubin fit:  center (0.4909211,-22.15598)  radius 22.18378  sigma 0.05416513

  Hyper  fit:  center (0.4908357,-22.15212)  radius 22.17979  sigma 0.05416513
*/
         
    reals BenchmarkExample2DataX[4] {-1.,-0.3,0.3,1.};
    reals BenchmarkExample2DataY[4] {0.,-0.06,0.1,0.};

    Data data3(4,BenchmarkExample2DataX,BenchmarkExample2DataY);

    circle = CircleFitByKasa (data3);
    cout << "\nTest Three:\n  Kasa   fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByPratt (data3);
    cout << "\n  Pratt  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByTaubin (data3);
    cout << "\n  Taubin fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByHyper (data3);
    cout << "\n  Hyper  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    circle = CircleFitByLeastSquare (data3);
    cout << "\n  LS  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;



/*
Numpy data Test One:  

*/


     radian=" 1 " ;
     n= 30 ;
     GR=" [0, 0] 1 ";

     reals test_x_8[n]  {1.03, 1.11, 0.86, 1.04, 0.68, 0.85, 0.71, 0.44, 0.61, 0.37, 0.28, 0.20, 0.23, 0.16, 0.16, 0.04, -0.05, -0.29, -0.34, -0.49, -0.55, -0.81, -0.81, -0.88, -0.80, -0.89, -0.71, -0.87, -0.82, -1.08};
     reals test_y_8[n]  {-0.00, 0.16, 0.28, 0.20, 0.32, 0.62, 0.66, 0.71, 0.97, 0.83, 0.83, 1.16, 0.94, 0.87, 1.06, 0.88, 1.09, 1.05, 0.77, 0.84, 0.93, 0.72, 0.61, 0.61, 0.55, 0.48, 0.33, 0.31, 0.16, 0.03};

    data.assignValues(n,test_x_8,test_y_8);

    cout.precision(7);
    circle = CircleFitByKasa (data);
    cout << "\n Test from numpy data";
    cout << "\n Num of samples:  "<< n << "    radian: "<<radian<<"Ground truth:  "<<GR;
    cout << "\n  Pratt  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByPratt (data);
    cout << "\n  Pratt  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByTaubin (data);
    cout << "\n  Taubin fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    
    circle = CircleFitByHyper (data);
    cout << "\n  Hyper  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
    circle = CircleFitByLeastSquare (data);
    cout << "\n  LS  fit:  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
     


/*
Numpy data Test Two:  

*/


     radian=" 0.5 " ;
     n= 30 ;
     GR=" [1, -3] 4 ";
     reals test_npx_2[n]  {4.94, 5.06, 4.93, 4.97, 4.97, 4.56, 4.74, 4.65, 4.69, 4.48, 4.30, 4.36, 4.10, 4.00, 3.88, 3.66, 3.53, 3.34, 3.20, 3.06, 3.03, 2.64, 2.50, 2.35, 2.15, 1.81, 1.61, 1.47, 1.21, 0.94};
     reals test_npy_2[n]  {-3.02, -2.82, -2.63, -2.37, -2.10, -2.11, -1.62, -1.48, -1.27, -1.24, -0.64, -0.57, -0.45, -0.41, -0.34, -0.10, 0.06, 0.26, 0.47, 0.42, 0.48, 0.56, 0.64, 0.79, 0.93, 0.83, 0.95, 1.02, 0.93, 1.05};

     data.assignValues(n,test_npx_2,test_npy_2);
     simpletest(data,radian,GR,n);
/*
Numpy data Test Three:  

*/
     radian=" 0.5 " ;
     n= 30 ;
     GR=" [-2, 5] 3 ";
     reals test_npx_3[n]  {1.01, 1.09, 0.92, 0.94, 0.86, 0.91, 0.83, 0.97, 0.82, 0.70, 0.61, 0.44, 0.31, 0.25, 0.21, -0.03, 0.02, -0.41, -0.54, -0.39, -0.50, -0.51, -0.90, -0.98, -1.06, -1.34, -1.68, -1.58, -1.91, -1.98};
     reals test_npy_3[n]  {4.78, 5.11, 5.42, 5.55, 5.73, 5.84, 6.12, 5.89, 6.15, 6.23, 6.46, 6.64, 6.74, 6.93, 7.25, 7.19, 7.22, 7.20, 7.64, 7.65, 7.59, 7.76, 7.90, 7.98, 7.86, 7.84, 8.04, 7.78, 8.00, 8.20};
     
     data.assignValues(n,test_npx_3,test_npy_3);
     simpletest(data,radian,GR,n);

/*
Numpy data Test Four:  

*/
     radian=" 1 " ;
     n= 20 ;
     GR=" [3, 2] 2 ";
     reals test_npx_4[n]  {5.18, 5.07, 4.93, 4.60, 4.60, 4.32, 4.15, 3.85, 3.37, 3.10, 2.73, 2.65, 2.18, 2.02, 1.65, 1.45, 1.21, 1.10, 0.84, 0.95};
     reals test_npy_4[n]  {2.21, 2.21, 2.71, 3.07, 3.34, 3.52, 3.69, 3.82, 3.73, 4.07, 4.10, 3.99, 3.93, 3.65, 3.39, 3.02, 2.87, 2.57, 2.26, 2.11};
     data.assignValues(n,test_npx_4,test_npy_4);
     simpletest(data,radian,GR,n);
}



