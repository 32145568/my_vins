#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "parameter.hpp"
#include "utility.h"

using namespace std;

#ifndef VISUALFACTOR
#define VISUALFACTOR
class VisualFactor : public::ceres::SizedCostFunction<2, 7, 7, 7, 1> 
{
public:
    VisualFactor();
    VisualFactor(BackEndParameter *b_p_, Eigen::Vector3d observation_i_, Eigen::Vector3d observation_j_);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    //bool Evaluate(double const *const *parameters, double *residuals, double **jacobians);

    BackEndParameter *b_p;
    Eigen::Vector3d observation_i;
    Eigen::Vector3d observation_j;
    Eigen::Matrix2d info_matrix;
};

#endif