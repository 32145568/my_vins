#include <iostream>
#include "generate_synthetic_data.hpp"

using namespace std;

int main(int argc, char **argv) {
    SyntheticData *s_d = new SyntheticData(20, 100, 20, 100, 90);
    s_d->generate_pose();
    s_d->generate_imu_data();
    s_d->mid_point_intergration();
    s_d->generate_image_data();
    s_d->check_with_fundmental_matrix();

    /*for(size_t i = 1; i < s_d->pub_rotation_q.size(); i++) {
        Eigen::Vector3d delta_t;
        Eigen::Quaterniond delta_q;
        Eigen::Vector3d t1, t2;
        Eigen::Quaterniond q1, q2;
        
        t1 = s_d->pub_translation_q.front();
        s_d->pub_translation_q.pop();
        t2 = s_d->cal_translation_q[i];
        q1 = s_d->pub_rotation_q.front();
        s_d->pub_rotation_q.pop();
        q2 = s_d->cal_rotation_q[i];

        delta_t = t1 - t2;
        delta_q = q1.inverse() * q2;
        
        double norm_t = delta_t(0) * delta_t(0) + delta_t(1) * delta_t(1) + delta_t(2) * delta_t(2);
        norm_t = sqrt(norm_t);
        double norm_r = 0;
        norm_r += (delta_q.x()/delta_q.w() * delta_q.x()/delta_q.w());
        norm_r += (delta_q.y()/delta_q.w() * delta_q.y()/delta_q.w());
        norm_r += (delta_q.z()/delta_q.w() * delta_q.z()/delta_q.w());
        norm_r = sqrt(norm_r);
    }*/
}