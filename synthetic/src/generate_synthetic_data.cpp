#include "generate_synthetic_data.hpp"

SyntheticData::SyntheticData(double time_, int imu_freq_, int image_freq_, int features_n_, int tracked_n_) {
    time = time_;
    imu_freq = imu_freq_;
    image_freq = image_freq_;
    features_n = features_n_;
    sampling_freq = 1000;
    delta_time = 1.0 / sampling_freq;
}

void SyntheticData::generate_pose() {
    double w_xy = pi / time;
    double w_z = 2 * pi / time;
    double radius = 10;
    for(size_t i = 0; i < sampling_freq * time; i++) {
        double x = radius * cos(w_xy * i * delta_time);
        double y = radius * sin(w_xy * i * delta_time);
        double z = sin(w_z * i * delta_time + pi / 6);
        double yaw = w_xy * i *delta_time;
        double pitch = pi / 6 * sin(w_z * i * delta_time + pi / 3);
        double roll = pi / 6 * cos(w_z * i * delta_time - pi / 4);
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())* 
                                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())*
                                          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond rotation;
        rotation = rotation_matrix;
        Eigen::Vector3d translation{x, y, z};
        translation_q.push(translation);
        rotation_m_q.push(rotation_matrix);
        rotation_q.push(rotation);
    }    
}

void SyntheticData::generate_imu_data() {
    int imu_n =  static_cast<int>(sampling_freq / imu_freq);
    int frame_n = static_cast<int>(sampling_freq / image_freq);
    
    int i = 1;

    Eigen::Vector3d translation_last = translation_q.front();
    Eigen::Quaterniond rotation_last = rotation_q.front();
    init_rotation = rotation_last;
    init_translation = translation_last;

    while(true) {
        if(translation_q.empty()) {
            break;
        } else {
            Eigen::Vector3d translation = translation_q.front();
            Eigen::Quaterniond rotation = rotation_q.front();
            Eigen::Matrix3d rotation_m = rotation_m_q.front();

            translation_q.pop();
            rotation_q.pop();
            rotation_m_q.pop();

            if(i == 2) {
                Eigen::Quaterniond rotation_next;
                Eigen::Vector3d translation_next;

                translation_next = translation_q.front();
                rotation_next = rotation_q.front();

                Eigen::Quaterniond delta_q = rotation_last.inverse() * rotation_next;
                double delta_rx = delta_q.x() * 2 / delta_q.w();
                double delta_ry = delta_q.y() * 2 / delta_q.w();
                double delta_rz = delta_q.z() * 2 / delta_q.w();
                Eigen::Vector3d gyr{delta_rx / (2 * delta_time), delta_ry / (2 * delta_time), delta_rz / (2 * delta_time)};
                gyr_0 = gyr;
                
                double delta_tx_1 = translation[0] - translation_last[0];
                double delta_tx_2 = translation_next[0] - translation[0];
                double delta_ty_1 = translation[1] - translation_last[1];
                double delta_ty_2 = translation_next[1] - translation[1];
                double delta_tz_1 = translation[2] - translation_last[2];
                double delta_tz_2 = translation_next[2] - translation[2];

                double vx_1 = delta_tx_1 / delta_time;
                double vy_1 = delta_ty_1 / delta_time;
                double vz_1 = delta_tz_1 / delta_time;
                double vx_2 = delta_tx_2 / delta_time;
                double vy_2 = delta_ty_2 / delta_time;
                double vz_2 = delta_tz_2 / delta_time;

                double delta_vx = vx_2 - vx_1;
                double delta_vy = vy_2 - vy_1;
                double delta_vz = vz_2 - vz_1;

                Eigen::Vector3d acc{delta_vx / delta_time, delta_vy / delta_time, delta_vz / delta_time};
                acc_0 = rotation_m.transpose() * acc;

                init_v(0) = (vx_1 + vx_2) * 0.5;
                init_v(1) = (vy_1 + vy_2) * 0.5;
                init_v(2) = (vz_1 + vz_2) * 0.5;
            }

            if(i % imu_n == 0) {
                Eigen::Quaterniond rotation_next;
                Eigen::Vector3d translation_next;

                translation_next = translation_q.front();
                rotation_next = rotation_q.front();

                Eigen::Quaterniond delta_q = rotation_last.inverse() * rotation_next;
                double delta_rx = delta_q.x() * 2 / delta_q.w();
                double delta_ry = delta_q.y() * 2 / delta_q.w();
                double delta_rz = delta_q.z() * 2 / delta_q.w();
                Eigen::Vector3d gyr{delta_rx / (2 * delta_time), delta_ry / (2 * delta_time), delta_rz / (2 * delta_time)};

                double delta_tx_1 = translation[0] - translation_last[0];
                double delta_tx_2 = translation_next[0] - translation[0];
                double delta_ty_1 = translation[1] - translation_last[1];
                double delta_ty_2 = translation_next[1] - translation[1];
                double delta_tz_1 = translation[2] - translation_last[2];
                double delta_tz_2 = translation_next[2] - translation[2];

                double vx_1 = delta_tx_1 / delta_time;
                double vy_1 = delta_ty_1 / delta_time;
                double vz_1 = delta_tz_1 / delta_time;
                double vx_2 = delta_tx_2 / delta_time;
                double vy_2 = delta_ty_2 / delta_time;
                double vz_2 = delta_tz_2 / delta_time;

                double delta_vx = vx_2 - vx_1;
                double delta_vy = vy_2 - vy_1;
                double delta_vz = vz_2 - vz_1;

                Eigen::Vector3d acc{delta_vx / delta_time, delta_vy / delta_time, delta_vz / delta_time};
                acc = rotation_m.transpose() * acc;

                gyr_q.push(gyr);
                acc_q.push(acc);
                imu_header.push(i);

                if(i % frame_n == 0) {

                    pub_rotation_q.push_back(rotation);
                    pub_translation_q.push_back(translation);
                    pub_rotation_m_q.push_back(rotation_m);
                    frame_header.push(i);
                }
            }
            rotation_last = rotation;
            translation_last = translation;
            i++;
        }
    }
}

void SyntheticData::generate_imu_data_with_noise () {
    int imu_n =  static_cast<int>(sampling_freq / imu_freq);
    int frame_n = static_cast<int>(sampling_freq / image_freq);
    
    int i = 1;
    default_random_engine generator;
    normal_distribution<double> acc_noise(0, acc_n);
    normal_distribution<double> gyr_noise(0, gyr_n);

    Eigen::Vector3d translation_last = translation_q.front();
    Eigen::Quaterniond rotation_last = rotation_q.front();

    while(true) {
        if(translation_q.empty()) {
            break;
        } else {
            Eigen::Vector3d translation = translation_q.front();
            Eigen::Quaterniond rotation = rotation_q.front();
            Eigen::Matrix3d rotation_m = rotation_m_q.front();

            translation_q.pop();
            rotation_q.pop();
            rotation_m_q.pop();

            if(translation_q.empty()) {
                break;
            }

            if(i % imu_n == 0) {
                Eigen::Quaterniond rotation_next;
                Eigen::Vector3d translation_next;

                translation_next = translation_q.front();
                rotation_next = rotation_q.front();

                Eigen::Quaterniond delta_q = rotation_last.inverse() * rotation_next;
                double delta_rx = delta_q.x() * 2 / delta_q.w();
                double delta_ry = delta_q.y() * 2 / delta_q.w();
                double delta_rz = delta_q.z() * 2 / delta_q.w();
                Eigen::Vector3d gyr{delta_rx / (2 * delta_time), delta_ry / (2 * delta_time), delta_rz / (2 * delta_time)};

                double delta_tx_1 = translation[0] - translation_last[0];
                double delta_tx_2 = translation_next[0] - translation[0];
                double delta_ty_1 = translation[1] - translation_last[1];
                double delta_ty_2 = translation_next[1] - translation[1];
                double delta_tz_1 = translation[2] - translation_last[2];
                double delta_tz_2 = translation_next[2] - translation[2];

                double vx_1 = delta_tx_1 / delta_time;
                double vy_1 = delta_ty_1 / delta_time;
                double vz_1 = delta_tz_1 / delta_time;
                double vx_2 = delta_tx_2 / delta_time;
                double vy_2 = delta_ty_2 / delta_time;
                double vz_2 = delta_tz_2 / delta_time;

                double delta_vx = vx_2 - vx_1;
                double delta_vy = vy_2 - vy_1;
                double delta_vz = vz_2 - vz_1;

                Eigen::Vector3d acc{delta_vx / delta_time, delta_vy / delta_time, delta_vz / delta_time};
                acc = acc + g;
                acc = rotation_m.transpose() * acc;
                acc(0) += acc_noise(generator);
                acc(1) += acc_noise(generator);
                acc(2) += acc_noise(generator);

                gyr(0) += gyr_noise(generator);
                gyr(1) += gyr_noise(generator);
                gyr(2) += gyr_noise(generator);

                acc = acc + acc_bias;
                gyr = gyr + gyr_bias;

                gyr_with_noise_q.push(gyr);
                acc_with_noise_q.push(acc);
                imu_header.push(i);

                if(i % frame_n == 0) {
                    pub_rotation_q.push_back(rotation);
                    pub_translation_q.push_back(translation);
                    pub_rotation_m_q.push_back(rotation_m);
                    frame_header.push(i);
                }
            }
            rotation_last = rotation;
            translation_last = translation;
            i++;
        }
    }
}

void SyntheticData::generate_image_data() {
    queue<Eigen::Vector3d> temp_frame_features;
    queue<int> temp_frame_ids;
    int global_id = 0;

    for(size_t i = 0; i < 15; i++) {
        for(size_t j = 0; j < 10; j++) {
            double px = i * internal_x;
            double py = j * internal_y;
            px = (px - cx) / fx;
            py = (py - cy) / fy;

            double un_px = px * depth;
            double un_py = py * depth;
            double un_pz = depth;

            Eigen::Vector3d p{un_px, un_py, un_pz};
            temp_frame_features.push(p);
            temp_frame_ids.push(global_id);
            global_id ++;
        }
    }

    features_pre_frame.push_back(temp_frame_features);
    feature_ids.push_back(temp_frame_ids);
    last_frame_features = temp_frame_features;
    last_frame_ids = temp_frame_ids;

    for(size_t i = 1;  i < pub_rotation_q.size(); i++) {
        int tracked_number = 0;
        queue<Eigen::Vector3d> temp_frame_features;
        queue<int> temp_frame_ids;
        cv::Mat mask = cv::Mat(image_row, image_col, CV_8UC1, cv::Scalar(255));

        Eigen::Quaterniond temp_rotation = pub_rotation_q[i];
        Eigen::Vector3d temp_translation = pub_translation_q[i];
        temp_rotation = temp_rotation.inverse();
        temp_translation = temp_rotation * temp_translation;
        temp_translation = - temp_translation;

        while(true) {
            if(last_frame_ids.empty()) {
                break;
            } else {
                Eigen::Vector3d p = last_frame_features.front();
                int temp_id = last_frame_ids.front();
                last_frame_features.pop();
                last_frame_ids.pop();
                p = pub_rotation_q[i - 1] * p;
                p = p + pub_translation_q[i - 1];
                p = temp_rotation * p + temp_translation;

                if(p(2) <= 0.5) {
                    continue;
                }

                double px = p(0) / p(2);
                double py = p(1) / p(2);
                int u = static_cast<int>(px * fx + cx);
                int v = static_cast<int>(py * fy + cy);
                cv::Point2f kp;
                kp.x = u;
                kp.y = v;

                if(u >= 0 && u < image_col && v >= 0 && v < image_row) {
                    tracked_number ++;
                    temp_frame_features.push(p);
                    temp_frame_ids.push(temp_id);
                    cv::circle(mask, kp, 5, 0, -1);
                }
            }
        }
        
        int n = features_n - temp_frame_features.size();
        int m = 0;

        if(n > 0) {
            for(size_t i = 0; i < 15; i++) {
                for(size_t j = 0; j < 10; j++) {
                    if(m == n) {
                        break;
                    }
                    cv::Point2f p;
                    p.x = i * internal_x;
                    p.y = j * internal_y;
                    if(mask.at<uchar>(p) == 255) {
                        p.x = (p.x - cx) / fx;
                        p.y = (p.y - cy) / fy;  
                        Eigen::Vector3d temp_p{p.x * depth, p.y * depth, depth};
                        temp_frame_features.push(temp_p);
                        temp_frame_ids.push(global_id);
                        global_id ++;
                        m ++;
                    }
                }         
            }
        }

        last_frame_features = temp_frame_features;
        last_frame_ids = temp_frame_ids;
        features_pre_frame.push_back(last_frame_features);
        feature_ids.push_back(last_frame_ids);    
    }
}

void SyntheticData::generate_image_data_with_noise() {
    queue<Eigen::Vector3d> temp_frame_features;
    queue<cv::Point2f> temp_frame_features_wn;
    queue<cv::Point2f> temp_features_v;

    queue<int> temp_frame_ids;
    default_random_engine generator;
    normal_distribution<double> image_noise(0, image_n);
    int global_id = 0;

    for(size_t i = 0; i < 15; i++) {
        for(size_t j = 0; j < 10; j++) {
            double px = i * internal_x;
            double py = j * internal_y;

            cv::Point2f kp, v;

            px = (px - cx) / fx;
            py = (py - cy) / fy;

            kp.x = px + image_noise(generator) / fx;
            kp.y = py + image_noise(generator) / fy;
            v.x = -100; v.y = -100;

            double un_px = px * depth;
            double un_py = py * depth;
            double un_pz = depth;

            Eigen::Vector3d p{un_px, un_py, un_pz};
            temp_frame_features.push(p);
            temp_frame_ids.push(global_id);
            temp_frame_features_wn.push(kp);
            temp_features_v.push(v);

            global_id ++;
        }
    }

    features_pre_frame.push_back(temp_frame_features);
    feature_ids.push_back(temp_frame_ids);
    features_pre_frame_wn.push_back(temp_frame_features_wn);
    features_v_pre_frame.push_back(temp_features_v);
    last_frame_features = temp_frame_features;
    last_frame_ids = temp_frame_ids;

    for(size_t i = 1;  i < pub_rotation_q.size(); i++) {
        int tracked_number = 0;
        queue<Eigen::Vector3d> temp_frame_features;
        queue<int> temp_frame_ids;
        queue<cv::Point2f> temp_frame_features_wn;
        queue<cv::Point2f> temp_features_v;
        cv::Mat mask = cv::Mat(image_row, image_col, CV_8UC1, cv::Scalar(255));

        Eigen::Quaterniond temp_rotation = pub_rotation_q[i];
        Eigen::Vector3d temp_translation = pub_translation_q[i];
        temp_rotation = temp_rotation.inverse();
        temp_translation = temp_rotation * temp_translation;
        temp_translation = - temp_translation;

        while(true) {
            if(last_frame_ids.empty()) {
                break;
            } else {
                Eigen::Vector3d p, p_;
                p = last_frame_features.front(); p_ = p;
                int temp_id = last_frame_ids.front();
                last_frame_features.pop();
                last_frame_ids.pop();
                p = pub_rotation_q[i - 1] * p;
                p = p + pub_translation_q[i - 1];
                p = temp_rotation * p + temp_translation;

                if(p(2) <= 0.5) {
                    continue;
                }

                double px = p(0) / p(2);
                double py = p(1) / p(2);
                int u = static_cast<int>(px * fx + cx);
                int v = static_cast<int>(py * fy + cy);
                cv::Point2f kp, kp_v;
                kp.x = u;
                kp.y = v;

                if(u > 0 && u < image_col && v > 0 && v < image_row) {
                    tracked_number ++;
                    temp_frame_features.push(p);
                    temp_frame_ids.push(temp_id);
                    cv::circle(mask, kp, 5, 0, -1);
                    kp.x = px + image_noise(generator) / fx;
                    kp.y = py + image_noise(generator) / fy;

                    kp_v.x = (kp.x - p_(0) / p_(2)) * image_freq;
                    kp_v.y = (kp.y - p_(1) / p_(2)) * image_freq;

                    temp_frame_features_wn.push(kp);
                    temp_features_v.push(kp_v);


                }
            }
        }
        
        int n = features_n - temp_frame_features.size();
        int m = 0;

        if(n > 0) {
            for(size_t i = 0; i < 15; i++) {
                for(size_t j = 0; j < 10; j++) {
                    if(m == n) {
                        break;
                    }
                    cv::Point2f kp, kp_v;
                    kp.x = i * internal_x;
                    kp.y = j * internal_y;
                    if(mask.at<uchar>(kp) == 255) {
                        
                        double px = (kp.x - cx) / fx;
                        double py = (kp.y - cy) / fy;  
                        Eigen::Vector3d temp_p{px * depth, py * depth, depth};

                        kp.x = px + image_noise(generator) / fx;
                        kp.y = py + image_noise(generator) / fy;
                        kp_v.x = -100;
                        kp_v.y = -100;

                        temp_frame_features.push(temp_p);
                        temp_frame_ids.push(global_id);
                        temp_frame_features_wn.push(kp);
                        temp_features_v.push(kp_v);
                        global_id ++;
                        m ++;
                    }
                }         
            }
        }

        last_frame_features = temp_frame_features;
        last_frame_ids = temp_frame_ids;
        features_pre_frame.push_back(last_frame_features);
        feature_ids.push_back(last_frame_ids);    
        features_pre_frame_wn.push_back(temp_frame_features_wn);
        features_v_pre_frame.push_back(temp_features_v);
    }
}

void SyntheticData::find_correspandence(queue<Eigen::Vector3d> &features_1, queue<Eigen::Vector3d> &features_2, int frame_i, int frame_j) {
    queue<Eigen::Vector3d> features_in_i = features_pre_frame[frame_i];
    queue<int> ids_in_i = feature_ids[frame_i];
    queue<int> ids_in_j = feature_ids[frame_j];

    queue<Eigen::Vector3d> features_in_j_ = features_pre_frame[frame_j];
    vector<Eigen::Vector3d> features_in_j;
    unordered_map<int, int> idx; 

    int j = 0;

    while(true) {
        if(features_in_j_.empty()) {
            break;
        } else {
            int temp_id = ids_in_j.front();
            ids_in_j.pop();
            features_in_j.push_back(features_in_j_.front());
            features_in_j_.pop();
            idx[temp_id] = j;
            j++;
        }
    }

    while(true) {
        if(ids_in_i.empty()) {
            break;
        } else {
            int id = ids_in_i.front();
            ids_in_i.pop();
            Eigen::Vector3d temp_p = features_in_i.front();
            features_in_i.pop();
            unordered_map<int, int>::iterator it = idx.find(id);
            if(it == idx.end()) {
                continue;
            }            
            features_1.push(temp_p);
            int pos = it->second;
            temp_p = features_in_j[pos];
            features_2.push(temp_p);
        }
    }
}

void SyntheticData::check_with_fundmental_matrix() {

    for(size_t i = 0; i < features_pre_frame.size() - 1; i++) {
        vector<cv::Point2f> kps_1, kps_2;
        vector<uchar> status;
        queue<Eigen::Vector3d> features_1;
        queue<Eigen::Vector3d> features_2;

        find_correspandence(features_1, features_2, i, i+1);

        while(true) {
            if(features_1.empty()) {
                break;
            } else {
                Eigen::Vector3d p1 = features_1.front();
                features_1.pop();
                Eigen::Vector3d p2 = features_2.front();
                features_2.pop();
                cv::Point2f kp_1;
                cv::Point2f kp_2;

                kp_1.x = (p1(0) / p1(2)) * 460 + image_col * 0.5;
                kp_1.y = (p1(1) / p1(2)) * 460 + image_row * 0.5;
                kp_2.x = (p2(0) / p2(2)) * 460 + image_col * 0.5;
                kp_2.y = (p2(1) / p2(2)) * 460 + image_row * 0.5;      

                kps_1.push_back(kp_1);
                kps_2.push_back(kp_2);            
            }
        }
        cv::findFundamentalMat(kps_1, kps_2, cv::FM_RANSAC, 1, 0.99, status);  

        int inlier_num = 0;
        for(vector<uchar>::iterator it = status.begin(); it != status.end(); it++) {
            if(*(it)) {
                inlier_num ++;
            }
        }      
        cout<<i<<" and "<<i+1<<" have "<<inlier_num<<endl;
    }
}

void SyntheticData::mid_point_intergration() {
    vector<Eigen::Vector3d> temp_acc_q;
    vector<Eigen::Vector3d> temp_gyr_q;

    double imu_delta_t = 1.0 / imu_freq;
    cal_rotation_q.push_back(init_rotation);
    cal_translation_q.push_back(init_translation);
    cal_velocity_q.push_back(init_v);
    
    while(true) {
        if(acc_q.empty() || frame_header.empty()) {
            break;
        } else {
            int f_h = frame_header.front();
            int i_h = imu_header.front();

            if(acc_0 == Eigen::Vector3d::Zero()) {
                acc_0 = acc_q.front();
                gyr_0 = gyr_q.front();
            } 

            if(f_h != i_h) {

                temp_acc_q.push_back(acc_q.front());
                temp_gyr_q.push_back(gyr_q.front());
                acc_q.pop();
                gyr_q.pop();
                imu_header.pop();
            } else {
                temp_acc_q.push_back(acc_q.front());
                temp_gyr_q.push_back(gyr_q.front());
                acc_q.pop();
                gyr_q.pop();
                imu_header.pop();
                frame_header.pop();

                Eigen::Quaterniond q_current(1, 0, 0, 0);
                Eigen::Quaterniond q_last(1, 0, 0, 0);
                Eigen::Vector3d v_current = Eigen::Vector3d::Zero();
                Eigen::Vector3d t_current = Eigen::Vector3d::Zero();
                double sum_time = 0;

                for(size_t i = 0; i < temp_acc_q.size(); i++) {
                    Eigen::Vector3d aver_gyr;
                    Eigen::Vector3d aver_acc;

                    if(i == 0) {
                        aver_gyr = (gyr_0 + temp_gyr_q[0]) * 0.5;
                        Eigen::Quaterniond delta_q;
                        delta_q.w() = 1;
                        delta_q.x() = 0.5 * imu_delta_t * aver_gyr(0);
                        delta_q.y() = 0.5 * imu_delta_t * aver_gyr(1);
                        delta_q.z() = 0.5 * imu_delta_t * aver_gyr(2);
                        q_current = q_last * delta_q;
                        aver_acc = (acc_0 + q_current * temp_acc_q[0]) * 0.5;
                        t_current = aver_acc * imu_delta_t * imu_delta_t * 0.5;
                        v_current = aver_acc * imu_delta_t;
                        q_last = q_current;
                        sum_time += imu_delta_t; 
                    } else {
                        sum_time += imu_delta_t;
                        aver_gyr = (temp_gyr_q[i] + temp_gyr_q[i-1]) * 0.5;
                        Eigen::Quaterniond delta_q;
                        delta_q.w() = 1;
                        delta_q.x() = 0.5 * imu_delta_t * aver_gyr(0);
                        delta_q.y() = 0.5 * imu_delta_t * aver_gyr(1);
                        delta_q.z() = 0.5 * imu_delta_t * aver_gyr(2);
                        q_current = q_last * delta_q;
                        aver_acc = (q_current * temp_acc_q[i] + q_last * temp_acc_q[i-1]) * 0.5;
                        t_current = aver_acc * imu_delta_t * imu_delta_t * 0.5 + v_current * imu_delta_t;
                        v_current = aver_acc * imu_delta_t + v_current;
                        q_last = q_current;
                    }

                }

                acc_0 = temp_acc_q[temp_acc_q.size() - 1];
                gyr_0 = temp_gyr_q[temp_gyr_q.size() - 1];
                temp_acc_q.clear();
                temp_gyr_q.clear();

                Eigen::Quaterniond rotation = cal_rotation_q[cal_rotation_q.size() - 1];
                Eigen::Vector3d translation = cal_translation_q[cal_translation_q.size() - 1];
                Eigen::Vector3d velocity = cal_velocity_q[cal_velocity_q.size() - 1];

                translation = translation + velocity * sum_time + rotation * t_current;
                velocity = velocity + rotation * v_current;
                rotation = rotation * q_current;

                cal_rotation_q.push_back(rotation);
                cal_translation_q.push_back(translation);
                cal_velocity_q.push_back(velocity);
            }
        }
    }
}

