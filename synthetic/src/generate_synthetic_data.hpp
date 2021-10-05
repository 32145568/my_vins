#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <vector>
#include <unordered_map>
using namespace std;

#ifndef SYNTHETIC_DATA
#define SYNTHETIC_DATA

class SyntheticData 
{
public:
    SyntheticData(double time_, int imu_freq_, int image_freq_, int features_n_, int tracked_n_);
    void generate_pose();
    void generate_imu_data();
    void generate_image_data();
    void mid_point_intergration();
    void check_with_fundmental_matrix();
    void find_correspandence(queue<Eigen::Vector3d> &features_1, queue<Eigen::Vector3d> &features_2, int frame_i, int frame_j);

    double time;
    int imu_freq;
    int image_freq;
    int sampling_freq;
    int features_n;
    int tracked_n;

    int image_col = 752;
    int image_row = 480;
    int internal_x = 75;
    int internal_y = 48;

    double acc_n = sqrt(0.08);
    double gyr_n = sqrt(0.004);
    double depth = 10.0;
    double fx = 461.6;
    double fy = 460.3;
    double cx = 363.0;
    double cy = 248.1;    
    Eigen::Vector3d gyn_bias{0.02, 0.02, 0.02};
    Eigen::Vector3d acc_bias{0.001, 0.001, 0.001};
    Eigen::Vector3d g{0, 0, -9.81};
    Eigen::Vector3d init_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_translation;
    Eigen::Quaterniond init_rotation;

    queue<Eigen::Vector3d> gyr_q;
    queue<Eigen::Vector3d> acc_q;
    queue<int> imu_header;
    queue<int> frame_header;
    
    vector<Eigen::Vector3d> gyr_with_noise_q;
    vector<Eigen::Vector3d> acc_with_noise_q;

    vector<queue<Eigen::Vector3d>> features_pre_frame;
    vector<queue<int>> feature_ids;
    queue<Eigen::Vector3d> last_frame_features;
    queue<int> last_frame_ids;

    queue<Eigen::Quaterniond> rotation_q;
    queue<Eigen::Matrix3d> rotation_m_q;
    queue<Eigen::Vector3d> translation_q; 
    vector<Eigen::Quaterniond> pub_rotation_q;
    vector<Eigen::Matrix3d> pub_rotation_m_q;
    vector<Eigen::Vector3d> pub_translation_q;

    vector<Eigen::Quaterniond> cal_rotation_q;
    vector<Eigen::Vector3d> cal_translation_q; 
    vector<Eigen::Vector3d> cal_velocity_q;

    double pi = 3.1415926535;
    double delta_time;

    Eigen::Vector3d acc_0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyr_0 = Eigen::Vector3d::Zero();
};

#endif

