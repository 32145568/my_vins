#include <eigen3/Eigen/Dense>
#include <queue>
#include <vector>
#include <list>
#include <unordered_map>
#include <map>
#include <math.h>
#include "back_end/kf.h"
#include "back_end/up_kf.h"

struct Feature 
{
    double un_px;
    double un_py;
    double un_pz;
    double px;
    double py;
    double vx;
    double vy;
    bool if_start = false;
};

struct MapPoint 
{
    double depth;
    int start_frame_id;
    int feature_id;
    vector<Feature> observation;
    bool if_outlier;
    MapPoint* next_addr;
};

class LocalMapping 
{
public:
    local_mapping(back_end_parameter *b_p_);
    void get_correspondance(int frame_1, int frame_2, queue<Eigen::Vector3d> &points_1, queue<Eigen::Vector3d> &points_2);
    bool insert_new_frame(int frame_id, queue<int> &feature_ids, queue<Eigen::Matrix<double, 7, 1>> &features);
    void remove_old_keyframe();
    void remove_sub_new_frame();
    void update_depth();
    bool compute_parallax(int frame_id);
    
    MapPoint *first_map_point;
    MapPoint *end_map_point;
    unordered_map<int, MapPoint*> map_idx;
    vector<list<int>> features_per_frame; 
    back_end_parameter *b_p;
};