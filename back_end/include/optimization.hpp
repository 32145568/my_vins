#include <iostream>
#include <queue>
#include <vector>
#include <list>
#include <ceres/ceres.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "parameter.hpp"
#include "local_mapping.hpp"
#include "imu_factor.hpp"

#ifndef OPTIMIZATION
#define OPTIMIZATION

const int window_size = 10;

class Optimization {
public:
    Optimization();
    Optimization(BackEndParameter *b_p_);
    void process(sensor_msgs::PointCloudConstPtr feature_msgs, queue<sensor_msgs::ImuConstPtr> imu_msgs, geometry_msgs::PoseStampedConstPtr pose_msgs);
    void visual_initialization();
    void imu_initialization();
    void marginalization();
    void vio_bundle_adjustment();
    void check_imu_propagate(Eigen::Vector3d g, Eigen::Quaterniond q, Eigen::Vector3d t);

    BackEndParameter *b_p;
    LocalMapping *l_m;
    Eigen::Vector3d translation_ex;
    Eigen::Matrix3d rotation_ex;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    bool if_initial = false;
    bool if_keyframe = false;
    bool if_first_image = true;
    
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
    double last_imu_time = 0;
    double last_image_time = 0;
    double curr_imu_time = 0;
    double curr_image_time = 0;

    int frame_count = 0;
    bool if_key_frame = false;

    vector<Eigen::Vector3d> pose_t_v;
    vector<Eigen::Quaterniond> pose_q_v;
    vector<double> pose_time_v;
};

#endif