#include <iostream>
#include <queue>
#include <vector>
#include <list>
#include <ceres/ceres.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include "parameter.hpp"
#include "local_mapping.hpp"

#ifndef OPTIMIZATION
#define OPTIMIZATION

const int window_size = 10;

class Optimization {
public:
    Optimization();
    Optimization(BackEndParameter *b_p_);
    void process(sensor_msgs::PointCloudConstPtr feature_msgs, queue<sensor_msgs::ImuConstPtr> imu_msgs);
    void visual_initialization();
    void imu_initialization();
    void marginalization();
    void vio_bundle_adjustment();

    BackEndParameter *b_p;
    LocalMapping *l_m;
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

    int frame_count = 0;
};

#endif