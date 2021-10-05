#include <iostream>
#include <queue>
#include <ceres/ceres.h>
#include "parameter.hpp"

const int window_size = 10;

class optimization {
public:
    optimization();
    optimization(BackEndParameter *b_p_);
    void process();
    void visual_initialization();
    void imu_initialization();
    void marginalization();
    void vio_bundle_adjustment();

    BackEndParameter *b_p;
    Eigen::Vector3d translation_ex;
    Eigen::Matrix3d rotation_ex;
    bool if_initial = false;
    bool if_keyframe = false;
    
    Eigen::Vector3d translation_q[window_size];
    Eigen::Matrix3d rotation_q[window_size];
    Eigen::Quaterniond quaterniond_q[window_size];
    Eigen::Vector3d velocity_q[window_size];
    Eigen::Vector3d acc_bias_q[window_size];
    Eigen::Vector3d gyr_bias_q[window_size];

    double para_pose[window_size + 1][7];
    double para_speed_and_bias[window_size + 1][9];
    double para_rotation_ex[7];
    double td = 0;
};