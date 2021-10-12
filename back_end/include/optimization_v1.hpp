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
#include "imu_factor_v1.hpp"
#include "visual_measurement_factor.hpp"

#ifndef OPTIMIZATION
#define OPTIMIZATION

const int window_size = 30;

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
    bool if_initial = false;
    bool if_keyframe = false;
    bool if_first_image = true;

    double para_pose[window_size + 1][7];
    double para_speed[window_size + 1][3];
    double para_bias[1][6];
    double para_ex[1][7];
    double para_features[1000][1];
    double td = 0;
    double last_imu_time = 0;
    double last_image_time = 0;
    double curr_imu_time = 0;
    double curr_image_time = 0;

    vector<Eigen::Map<Eigen::Vector3d>> translation_v;
    vector<Eigen::Matrix3d> rotation_m_v;
    vector<Eigen::Map<Eigen::Quaterniond>> rotation_v;
    vector<Eigen::Map<Eigen::Vector3d>> velocity_v;
    vector<Eigen::Map<Eigen::Vector3d>> acc_bias_v;
    vector<Eigen::Map<Eigen::Vector3d>> gyr_bias_v;
    vector<ImuIntergration *> imu_factor_v;
    vector<Eigen::Map<Eigen::Vector3d>> translation_ex;
    Eigen::Matrix3d rotation_ex_matrix;
    vector<Eigen::Map<Eigen::Quaterniond>> rotation_ex;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;

    int frame_count = 0;
    bool if_key_frame = false;

    vector<Eigen::Vector3d> pose_t_v;
    vector<Eigen::Quaterniond> pose_q_v;
    vector<double> pose_time_v;
};

#endif