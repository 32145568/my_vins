#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "parameter.hpp"

#ifndef IMU_FACTOR
#define IMU_FACTOR

class ImuIntergration 
{
public:
    ImuIntergration();
    ImuIntergration(Eigen::Vector3d acc_bias_, Eigen::Vector3d gyr_bias_, Eigen::Vector3d acc_0_, Eigen::Vector3d gyr_0_);
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
    double temp_delta_t;
    double sum_time;

    queue<double> delta_t_q;
    queue<Eigen::Vector3d> acc_q;
    queue<Eigen::Vector3d> gyr_q;
};

class imu_factor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
public:
    imu_factor(BackEndParameter *b_p_);

    BackEndParameter *b_p;
};

#endif