#include "imu_factor.hpp"

ImuIntergration::ImuIntergration() {
    ;
}

ImuIntergration::ImuIntergration(Eigen::Vector3d acc_bias_, Eigen::Vector3d gyr_bias_, Eigen::Vector3d acc_0_, Eigen::Vector3d gyr_0_) {
    acc_bias = acc_bias_;
    gyr_bias = gyr_bias_;
    acc_0 = acc_0_;
    gyr_0 = gyr_0_;    
    last_acc = acc_0 - acc_bias;
    last_gyr = gyr_0 - gyr_bias;
    temp_rotation.x() = 0;
    temp_rotation.y() = 0;
    temp_rotation.z() = 0;
    temp_rotation.w() = 1;
    temp_translation = Eigen::Vector3d::Zero();
    temp_velocity = Eigen::Vector3d::Zero();
    sum_time = 0;
}

void ImuIntergration::add_state(Eigen::Vector3d acc, Eigen::Vector3d gyr, double delta_t) {
    delta_t_q.push(delta_t);
    acc_q.push(acc);
    gyr_q.push(gyr);
}

void ImuIntergration::mid_intergration() {
    Eigen::Vector3d aver_gyr, aver_acc;

    aver_gyr = (last_gyr + temp_gyr) * 0.5;
    Eigen::Quaterniond delta_q;
    delta_q.w() = 1;
    delta_q.x() = 0.5 * temp_delta_t * aver_gyr(0);
    delta_q.y() = 0.5 * temp_delta_t * aver_gyr(1);
    delta_q.z() = 0.5 * temp_delta_t * aver_gyr(2);
    aver_acc = (temp_rotation * last_acc + temp_rotation * delta_q * temp_acc) * 0.5;
    temp_rotation = temp_rotation * delta_q;
    temp_translation = temp_translation + temp_velocity * temp_delta_t + aver_acc * temp_delta_t * temp_delta_t * 0.5;
    temp_velocity = temp_velocity + aver_acc * temp_delta_t;
    sum_time += temp_delta_t; 
}

void ImuIntergration::propagate() {
    while(true) {
        if(acc_q.empty()) {
            break;
        } else {
            temp_acc = acc_q.front() - acc_bias;
            temp_gyr = gyr_q.front() - gyr_bias;
            temp_delta_t = delta_t_q.front();
            delta_t_q.pop();
            acc_q.pop();
            gyr_q.pop();
            mid_intergration();
            last_acc = temp_acc;
            last_gyr = temp_gyr;
        }
    }
}

void ImuIntergration::re_propagete(Eigen::Vector3d acc_bias_, Eigen::Vector3d gyr_bias_) {
    acc_bias = acc_bias_;
    gyr_bias = gyr_bias_;

    last_acc = acc_0;
    last_gyr = gyr_0;
  
    temp_rotation.x() = 0;
    temp_rotation.y() = 0;
    temp_rotation.z() = 0;
    temp_rotation.w() = 1;
    temp_translation = Eigen::Vector3d::Zero();
    sum_time = 0;

    propagate();
}