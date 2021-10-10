#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <vector>

class CeresTest : public ceres::SizedCostFunction<3, 3, 3, 3>
{
public:
    CeresTest(Eigen::Vector3d x_, Eigen::Vector3d y_);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d x;
    Eigen::Vector3d y;
};