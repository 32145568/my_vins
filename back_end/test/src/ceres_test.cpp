#include "ceres_test.hpp"
using namespace std;

CeresTest::CeresTest(Eigen::Vector3d x_, Eigen::Vector3d y_) {
    x = x_;
    y = y_;
}

bool CeresTest::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d a(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d b(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d c(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Map<Eigen::Vector3d> residuals_e(residuals);

    residuals_e(0) = a.dot(x) + b.dot(c) - y(0);
    residuals_e(1) = b.dot(x) + a.dot(c) - y(1);
    residuals_e(2) = c.dot(x) + a.dot(b) - y(2);

    if(jacobians){
        if(jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_matrix_1(jacobians[0]);
            jacobian_matrix_1.block<1, 3>(0, 0) = x.transpose();
            jacobian_matrix_1.block<1, 3>(1, 0) = c.transpose();
            jacobian_matrix_1.block<1, 3>(2, 0) = b.transpose();        
        }

        if(jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_matrix_2(jacobians[1]);
            jacobian_matrix_2.block<1, 3>(0, 0) = c.transpose();
            jacobian_matrix_2.block<1, 3>(1, 0) = x.transpose();
            jacobian_matrix_2.block<1, 3>(2, 0) = a.transpose();        
        }

        if(jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_matrix_3(jacobians[2]);
            jacobian_matrix_3.block<1, 3>(0, 0) = b.transpose();
            jacobian_matrix_3.block<1, 3>(1, 0) = c.transpose();
            jacobian_matrix_3.block<1, 3>(2, 0) = x.transpose();
        }
    }

    return true;
}