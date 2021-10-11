#include "local_mapping.hpp"

LocalMapping::LocalMapping(BackEndParameter *b_p_) {
    first_map_point = nullptr;
    end_map_point = nullptr;
    b_p = b_p_;
}

void LocalMapping::get_correspondance(int frame_1, int frame_2, queue<Eigen::Vector3d> &points_1, queue<Eigen::Vector3d> &points_2) {
    unordered_map<int, MapPoint*>::iterator it;
    for(list<int>::iterator it_ls = frames[frame_1].features_id.begin(); it_ls != frames[frame_1].features_id.end(); it_ls++) {
        int feature_id = *it_ls;
        it = map_idx.find(feature_id);
        if(it == map_idx.end()) {
            continue;
        } else {
            MapPoint *mp_addr = it->second;
            int start_frame = mp_addr->start_frame_id;
            int end_frame = start_frame + mp_addr->observation.size() - 1;

            if(frame_2 >= start_frame && frame_2 <= end_frame) {
                Eigen::Vector3d point_1;
                Eigen::Vector3d point_2;
                Feature f_1, f_2;
                f_1 = mp_addr->observation[frame_1 - start_frame];
                f_2 = mp_addr->observation[frame_2 - start_frame];
                point_1(0) = f_1.un_px; point_1(1) = f_1.un_py; point_1(2) = f_1.un_pz;
                point_2(0) = f_2.un_px; point_2(1) = f_2.un_py; point_2(2) = f_2.un_pz;
                points_1.push(point_1);
                points_2.push(point_2);
                
            } else {
                continue;
            }
        }
    }
}

bool LocalMapping::insert_new_frame(int frame_id, queue<Eigen::Matrix<double, 7, 1>> &features) {
    list<int> this_frame_ids;
    Frame f_this;
    int tracked_last_features = 0;
    while(true) {
        if(features.empty()) {
            break;
        }

        Feature f;
        Eigen::MatrixXd f_m = features.front();
        features.pop();

        f.un_px = f_m(0, 0);
        f.un_py = f_m(1, 0);
        f.un_pz = 1;
        f.px = f_m(3, 0);
        f.py = f_m(4, 0);
        f.vx = f_m(5, 0);
        f.vy = f_m(6, 0);

        int feature_id = static_cast<int>(f_m(2, 0));
        this_frame_ids.push_back(feature_id);

        if(first_map_point == nullptr) {
            first_map_point = new MapPoint;
            first_map_point->depth = -1;
            first_map_point->start_frame_id = frame_id;
            first_map_point->feature_id = feature_id;
            first_map_point->observation.push_back(f);
            first_map_point->if_outlier = false;
            first_map_point->next_addr = nullptr;
            end_map_point = first_map_point;
            map_idx[feature_id] = first_map_point;
        } else {
            unordered_map<int, MapPoint*>::iterator it; 
            it = map_idx.find(feature_id);
            if(it == map_idx.end()) {
                MapPoint* new_mp = new MapPoint;
                new_mp->depth = -1;
                new_mp->start_frame_id = frame_id;
                new_mp->observation.push_back(f);
                new_mp->if_outlier = false;
                new_mp->next_addr = nullptr;
                end_map_point->next_addr = new_mp;
                end_map_point = new_mp;
                map_idx[feature_id] = new_mp;
            } else {
                MapPoint *mp_addr = it->second;
                mp_addr->observation.push_back(f);
                tracked_last_features ++;
            }
        }
    }
    f_this.features_id = this_frame_ids;
    frames.push_back(f_this);
    //cout<<tracked_last_features<<endl;

    if(frame_id < 2 || tracked_last_features < 20) {
        return true;
    } else {
        return compute_parallax(frame_id);
    }
}

void LocalMapping::remove_old_keyframe() {
    ;
}

void LocalMapping::remove_sub_new_frame() {
    ;
}

void LocalMapping::update_depth() {
    ;
}

bool LocalMapping::compute_parallax(int frame_id) {
    int frame_1 = frame_id - 2;
    int frame_2 = frame_id - 1;
    //check_with_fundmental_matrix(frame_1, frame_2); 
    int parallax_num = 0;
    double parallax_sum = 0;

    list<int> feature_ids_1 = frames[frame_1].features_id;

    for(list<int>::iterator it = feature_ids_1.begin(); it != feature_ids_1.end(); it++) {
        int feature_id = *it;
        unordered_map<int, MapPoint*>::iterator it_um;
        it_um = map_idx.find(feature_id);
        if(it_um == map_idx.end()) {
            continue;
        } else {
            MapPoint *mp_addr = it_um->second;
            int start_frame = mp_addr->start_frame_id;
            int end_frame = mp_addr->start_frame_id + mp_addr->observation.size() - 1;
            if(frame_2 >= start_frame && frame_2 <= end_frame) {
                Feature f_1, f_2;
                f_1 = mp_addr->observation[frame_1 - start_frame];
                f_2 = mp_addr->observation[frame_2 - start_frame];
                double dx = f_1.un_px - f_2.un_px;
                double dy = f_1.un_py - f_2.un_py;
                //cout<<frame_id<<"  "<<sqrt(dx * dx + dy * dy)<<endl;
                parallax_sum += sqrt(dx * dx + dy * dy);
                parallax_num ++;
            }
        }
    }

    //cout<<parallax_sum / parallax_num<<"  "<<b_p->keyframe_parallax<<endl;
    return (parallax_sum / parallax_num >= b_p->keyframe_parallax);
}


void LocalMapping::check_with_fundmental_matrix(int frame_i, int frame_j) {
   
    vector<cv::Point2f> kps_1, kps_2;
    vector<uchar> status;
    queue<Eigen::Vector3d> features_1;
    queue<Eigen::Vector3d> features_2;

    get_correspondance(frame_i, frame_j, features_1, features_2);

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

            kp_1.x = (p1(0) / p1(2)) * 460 + b_p->image_col * 0.5;
            kp_1.y = (p1(1) / p1(2)) * 460 + b_p->image_row * 0.5;
            kp_2.x = (p2(0) / p2(2)) * 460 + b_p->image_col * 0.5;
            kp_2.y = (p2(1) / p2(2)) * 460 + b_p->image_row * 0.5;      

            kps_1.push_back(kp_1);
            kps_2.push_back(kp_2);            
        }
    }
    cv::findFundamentalMat(kps_1, kps_2, cv::FM_RANSAC, 1.5, 0.99, status);  

    int inlier_num = 0;
    for(vector<uchar>::iterator it = status.begin(); it != status.end(); it++) {
        if(*(it)) {
            inlier_num ++;
        }
    }      
    cout<<frame_i<<" and "<<frame_j<<" have "<<inlier_num<<endl;
}