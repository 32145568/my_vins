#include "optimization.hpp"
using namespace std;

Optimization::Optimization() {
    ;
}

Optimization::Optimization(BackEndParameter *b_p_) {
    b_p = b_p_;
    l_m = new LocalMapping(b_p);

    for(size_t i = 0; i < window_size; i++) {
        translation_q[i] = Eigen::Vector3d::Zero();
        rotation_q[i] = Eigen::Matrix3d::Identity();
        quaterniond_q[i].x() = 0;
        quaterniond_q[i].y() = 0;
        quaterniond_q[i].z() = 0;
        quaterniond_q[i].w() = 1;
        velocity_q[i] = Eigen::Vector3d::Zero();
        acc_bias_q[i] = Eigen::Vector3d::Zero();
        gyr_bias_q[i] = Eigen::Vector3d::Zero();        
    }
}

void Optimization::check_imu_propagate(Eigen::Vector3d g, Eigen::Quaterniond q, Eigen::Vector3d t) {

    Eigen::Quaterniond pose_q_1 = pose_q_v[frame_count - 1];
    Eigen::Quaterniond pose_q_2 = pose_q_v[frame_count];
    Eigen::Vector3d pose_t_1 = pose_t_v[frame_count - 1];
    Eigen::Vector3d pose_t_2 = pose_t_v[frame_count];
    Eigen::Vector3d t_1 = pose_t_v[frame_count - 1] - pose_t_v[frame_count - 2];
    Eigen::Vector3d t_2 = pose_t_v[frame_count] - pose_t_v[frame_count - 1];
    Eigen::Vector3d v;
    double time_1, time_2; 
    time_1 = pose_time_v[frame_count - 1] - pose_time_v[frame_count - 2];
    time_2 = pose_time_v[frame_count] - pose_time_v[frame_count - 1];
    v = 0.5 * (t_1 / time_1 + t_2 / time_2);

    Eigen::Quaterniond q_ = pose_q_1.inverse() * pose_q_2;
    Eigen::Vector3d t_ = pose_t_2 - pose_t_1 + g * time_2* time_2* 0.5 - v * time_2;
    t_ = pose_q_1.inverse() * t_;
    q_ = q_.inverse() * q;
    t_ = t_ - t;
    
    double dx, dy, dz, e_q, e_t;
    dx = q_.x() / q_.w();
    dy = q_.y() / q_.w();
    dz = q_.z() / q_.w();
    e_q = sqrt(dx * dx + dy * dy + dz * dz);
    e_t = sqrt(t_(0) * t_(0) + t_(1) * t_(1) + t_(2) * t_(2));

    //cout<<frame_count<<" : "<<" eq:"<<e_q<<" et:"<<e_t<<"  "<<t_2.norm()<<endl;
}

void Optimization::process(sensor_msgs::PointCloudConstPtr feature_msgs, queue<sensor_msgs::ImuConstPtr> imu_msgs, geometry_msgs::PoseStampedConstPtr pose_msgs) {
    
    Eigen::Vector3d gyr_bias{0.05, 0.05, 0.05};
    Eigen::Vector3d acc_bias{0.001, 0.001, 0.001};
    Eigen::Vector3d g{0, 0, -9.81};

    queue<Eigen::Matrix<double, 7, 1>> temp_features;
    ImuIntergration *imu_pro = nullptr;
    curr_imu_time = 0;
    last_imu_time = 0;
    
    for(size_t i = 0; i < feature_msgs->points.size(); i++) {
        curr_image_time = feature_msgs->header.stamp.toSec();
        Eigen::Matrix<double, 7, 1> temp_feature;

        temp_feature(0, 0) = feature_msgs->points[i].x;
        temp_feature(1, 0) = feature_msgs->points[i].y;
        temp_feature(2, 0) = feature_msgs->channels[0].values[i];
        temp_feature(3, 0) = feature_msgs->channels[1].values[i];
        temp_feature(4, 0) = feature_msgs->channels[2].values[i];
        temp_feature(5, 0) = feature_msgs->channels[3].values[i];
        temp_feature(6, 0) = feature_msgs->channels[4].values[i];

        temp_features.push(temp_feature);
    }

    if_key_frame = l_m->insert_new_frame(frame_count, temp_features);

    if(if_first_image) {
        if_first_image = false;
        while(true) {
            if(imu_msgs.size() == 1) {
                break;
            } else {
                sensor_msgs::ImuConstPtr imu = imu_msgs.front();
                curr_imu_time = imu->header.stamp.toSec();

                imu_msgs.pop();
                gyr_0(0) = imu->angular_velocity.x;
                gyr_0(1) = imu->angular_velocity.y;
                gyr_0(2) = imu->angular_velocity.z;

                acc_0(0) = imu->linear_acceleration.x;
                acc_0(1) = imu->linear_acceleration.y;
                acc_0(2) = imu->linear_acceleration.z;
            }
        }
        last_imu_time = curr_imu_time;
        sensor_msgs::ImuConstPtr imu = imu_msgs.front();
        Eigen::Vector3d temp_acc;
        Eigen::Vector3d temp_gyr;
        imu_msgs.pop();
        temp_gyr(0) = imu->angular_velocity.x;
        temp_gyr(1) = imu->angular_velocity.y;
        temp_gyr(2) = imu->angular_velocity.z;

        temp_acc(0) = imu->linear_acceleration.x;
        temp_acc(1) = imu->linear_acceleration.y;
        temp_acc(2) = imu->linear_acceleration.z;

        curr_imu_time = imu->header.stamp.toSec();
        double w1 = (curr_imu_time - curr_image_time) / (curr_imu_time - last_imu_time);
        double w2 = (curr_image_time - last_imu_time) / (curr_imu_time - last_imu_time);
        acc_0 = w1 * acc_0 + w2 * temp_acc;
        gyr_0 = w1 * gyr_0 + w2 * temp_gyr;

        last_image_time = curr_image_time;
        //cout<<w1<<"  "<<w2<<endl;
    } else {
        imu_pro = new ImuIntergration(acc_bias, gyr_bias, acc_0, gyr_0);
        sensor_msgs::ImuConstPtr imu;
        Eigen::Vector3d temp_acc;
        Eigen::Vector3d temp_gyr;
        double temp_delta_t;

        while(true) {
            if(imu_msgs.size() == 1) {
                break;
            } else {
                imu = imu_msgs.front();
                imu_msgs.pop();
                temp_gyr(0) = imu->angular_velocity.x;
                temp_gyr(1) = imu->angular_velocity.y;
                temp_gyr(2) = imu->angular_velocity.z;

                temp_acc(0) = imu->linear_acceleration.x;
                temp_acc(1) = imu->linear_acceleration.y;
                temp_acc(2) = imu->linear_acceleration.z;

                if(curr_imu_time == 0) {
                    curr_imu_time = imu->header.stamp.toSec();
                    last_imu_time = curr_imu_time;
                    temp_delta_t = curr_imu_time - last_image_time;
                } else {
                    curr_imu_time = imu->header.stamp.toSec();
                    temp_delta_t = curr_imu_time - last_imu_time;
                    last_imu_time = curr_imu_time;
                }
                imu_pro->add_state(temp_acc, temp_gyr, temp_delta_t);
            }
        }
        acc_0 = temp_acc;
        gyr_0 = temp_gyr;
        imu = imu_msgs.front();
        imu_msgs.pop();
        curr_imu_time = imu->header.stamp.toSec();

        temp_gyr(0) = imu->angular_velocity.x;
        temp_gyr(1) = imu->angular_velocity.y;
        temp_gyr(2) = imu->angular_velocity.z;

        temp_acc(0) = imu->linear_acceleration.x;
        temp_acc(1) = imu->linear_acceleration.y;
        temp_acc(2) = imu->linear_acceleration.z;

        double w1 = (curr_imu_time - curr_image_time) / (curr_imu_time - last_imu_time);
        double w2 = (curr_image_time - last_imu_time) / (curr_imu_time - last_imu_time);

        acc_0 = w1 * acc_0 + w2 * temp_acc;
        gyr_0 = w1 * gyr_0 + w2 * temp_gyr;
        temp_delta_t = curr_image_time - last_imu_time;

        imu_pro->add_state(acc_0, gyr_0, temp_delta_t);    
        imu_pro->propagate();
        last_image_time = curr_image_time;
        //cout<<w1<<"  "<<w2<<endl;
    }

    Eigen::Vector3d pose_t;
    Eigen::Quaterniond pose_q;
    double pose_time = pose_msgs->header.stamp.toSec();
    pose_t(0) = pose_msgs->pose.position.x;
    pose_t(1) = pose_msgs->pose.position.x;
    pose_t(2) = pose_msgs->pose.position.x;

    pose_q.x() = pose_msgs->pose.orientation.x;
    pose_q.y() = pose_msgs->pose.orientation.y;
    pose_q.z() = pose_msgs->pose.orientation.z;
    pose_q.w() = pose_msgs->pose.orientation.w;

    pose_t_v.push_back(pose_t);
    pose_q_v.push_back(pose_q);
    pose_time_v.push_back(pose_time);

    if(frame_count >= 2) {
        check_imu_propagate(g, imu_pro->temp_rotation, imu_pro->temp_translation);
    }

    frame_count ++;
}

void Optimization::visual_initialization() {
    ;
}

void Optimization::imu_initialization() {
    ;
}

void Optimization::marginalization() {
    ;
}

void Optimization::vio_bundle_adjustment() {
    ;
}