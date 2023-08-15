#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "circle.h"
#include "data.h"
#include "Utilities.cpp"
using namespace Eigen;

std::tuple<double, double, double> CircleFitByLeastSquare(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::vector<double>& w = std::vector<double>()
) {
    MatrixXd A(x.size(), 3);
    VectorXd b(x.size());

    for (size_t i = 0; i < x.size(); ++i) {
        A(i, 0) = x[i];
        A(i, 1) = y[i];
        A(i, 2) = 1.0;
        b(i) = x[i] * x[i] + y[i] * y[i];
    }

    // Modify A,b for weighted least squares
    if (w.size() == x.size()) {
        MatrixXd W = MatrixXd::Zero(x.size(), x.size());
        for (size_t i = 0; i < w.size(); ++i) {
            W(i, i) = w[i];
        }
        A = W * A;
        b = W * b;
    }

    // Solve by method of least squares
    Vector3d c = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

    // Get circle parameters from solution c
    double xc = c(0) / 2.0;
    double yc = c(1) / 2.0;
    double r = std::sqrt(c(2) + xc * xc + yc * yc);

    return std::make_tuple(xc, yc, r);
}




Circle CircleFitByLeastSquare (Data& data)

{
    Circle circle;
    MatrixXd A(data.n, 3);
    VectorXd b(data.n);

    for (size_t i = 0; i < data.n; ++i) {
        A(i, 0) = data.X[i];
        A(i, 1) = data.Y[i];
        A(i, 2) = 1.0;
        b(i) = data.X[i] * data.X[i] + data.Y[i] * data.Y[i];
    }


    // Solve by method of least squares
    Vector3d c = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

    // Get circle parameters from solution c

    circle.a = c(0) / 2.0;
    circle.b = c(1) / 2.0;
    circle.r = std::sqrt(c(2) + circle.a * circle.a + circle.b * circle.b);
    circle.s = Sigma(data,circle);
    
    return circle;
}
