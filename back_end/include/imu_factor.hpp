#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "parameter.hpp"

#ifndef IMU_FACTOR
#define IMU_FACTOR

class imu_factor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
public:
    imu_factor(BackEndParameter *b_p_);

    BackEndParameter *b_p;
};

#endif