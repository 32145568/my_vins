#include <eigen3/Eigen/Dense>
#include <queue>
#include <vector>
#include <list>
#include <unordered_map>
#include <map>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "back_end/kf.h"
#include "back_end/up_kf.h"
#include "parameter.hpp"
#include "imu_factor.hpp"

using namespace std;

#ifndef LOCAL_Mapping
#define LOCAL_Mapping
struct Feature 
{
    double un_px;
    double un_py;
    double un_pz;
    double px;
    double py;
    double vx;
    double vy;
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

struct Frame 
{
    Eigen::Quaterniond orientation;
    Eigen::Vector3d position;
    list<int> features_id;
    ImuFactor *i_f;
};

class LocalMapping 
{
public:
    LocalMapping(BackEndParameter *b_p_);
    void get_correspondance(int frame_1, int frame_2, queue<Eigen::Vector3d> &points_1, queue<Eigen::Vector3d> &points_2);
    bool insert_new_frame(int frame_id, queue<Eigen::Matrix<double, 7, 1>> &features);
    void remove_old_keyframe();
    void remove_sub_new_frame();
    void update_depth();
    bool compute_parallax(int frame_id);
    void check_with_fundmental_matrix(int frame_i, int frame_j);
    
    MapPoint *first_map_point;
    MapPoint *end_map_point;
    unordered_map<int, MapPoint*> map_idx;
    vector<Frame> frames; 
    BackEndParameter *b_p;
};
#endif