#include "optimization.hpp"
using namespace std;

Optimization::Optimization() {
    ;
}

Optimization::Optimization(BackEndParameter *b_p_) {
    b_p = b_p_;
    l_m = new LocalMapping(b_p);
}

void Optimization::process(sensor_msgs::PointCloudConstPtr feature_msgs, queue<sensor_msgs::ImuConstPtr> imu_msgs) {
    queue<Eigen::Matrix<double, 7, 1>> temp_features;
    
    for(size_t i = 0; i < feature_msgs->points.size(); i++) {
        //cout<<i<<endl;
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

    bool if_key_frame = l_m->insert_new_frame(frame_count, temp_features);
    if(if_key_frame) {
        //cout<<frame_count<<endl;
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