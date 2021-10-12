#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "parameter.hpp"
#include "utility.h"

#ifndef IMU_FACTOR
#define IMU_FACTOR
class ImuIntergration 
{
public:
    ImuIntergration(Eigen::Vector3d acc_bias_, Eigen::Vector3d gyr_bias_, Eigen::Vector3d acc_0_, Eigen::Vector3d gyr_0_, BackEndParameter *b_p_);
    void add_state(Eigen::Vector3d acc, Eigen::Vector3d gyr, double delta_t);
    void mid_intergration();
    void propagate();
    void re_propagete(Eigen::Vector3d acc_bias_, Eigen::Vector3d gyr_bias_);

    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyr_bias;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    Eigen::Vector3d temp_acc;
    Eigen::Vector3d temp_gyr;
    Eigen::Vector3d last_acc;
    Eigen::Vector3d last_gyr;
    Eigen::Quaterniond temp_rotation;
    Eigen::Vector3d temp_translation;
    Eigen::Vector3d temp_velocity;
    Eigen::Matrix<double, 12, 12> noise_matrix;
    Eigen::Matrix<double, 9, 9> covariance_matrix;
    Eigen::Matrix<double, 15, 15> jacobian_matrix;
    Eigen::Vector3d g_acc{0, 0, -9.81};
    double temp_delta_t;
    double sum_time;

    vector<double> delta_t_q;
    vector<Eigen::Vector3d> acc_q;
    vector<Eigen::Vector3d> gyr_q;

    BackEndParameter *b_p;
};

class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};


class ImuFactor : public ceres::SizedCostFunction<9, 7, 3, 7, 3, 6>
{
public:
    ImuFactor(ImuIntergration *i_p_);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    //bool Evaluate(double const *const *parameters, double *residuals, double **jacobians);

    ImuIntergration *i_p;
};


#endif