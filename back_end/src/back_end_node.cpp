#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <mutex>
#include <thread>
#include <map>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <condition_variable>
#include "optimization.hpp"
#include "imu_factor.hpp"
#include "parameter.hpp"

BackEndParameter *b_p = new BackEndParameter();
Optimization *o_p;

ros::Publisher pub_kp_msgs;
ros::Publisher pub_up_kf_msgs;

queue<sensor_msgs::ImuConstPtr> imu_q;
queue<sensor_msgs::PointCloudConstPtr> feature_q;
queue<sensor_msgs::ImageConstPtr> raw_image_q; 
queue<geometry_msgs::PoseStampedConstPtr> pose_q;

mutex queue_lock;
mutex optimization_lock;
mutex state_lock;
std::condition_variable con;

bool if_first_image = true;
bool if_pub = false;

struct Measurement {
    sensor_msgs::PointCloudConstPtr feature_msgs;
    queue<sensor_msgs::ImuConstPtr> imu_msgs;
    sensor_msgs::ImageConstPtr image_msgs;
    geometry_msgs::PoseStampedConstPtr pose;
};

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msgs) {
    //cout<<"feature"<<endl;
    if(!if_pub) {
        if_pub = true;
    } else {
        queue_lock.lock();
        feature_q.push(feature_msgs);
        queue_lock.unlock();
        con.notify_one();   
        if_pub = false;     
    }
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msgs) {
    //cout<<"imu"<<endl;
    queue_lock.lock();
    imu_q.push(imu_msgs);
    queue_lock.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msgs) {
    if(restart_msgs->data) {
        ROS_INFO("back end restart");
    }
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msgs) {
    //cout<<"image"<<endl;
    queue_lock.lock();
    raw_image_q.push(image_msgs);
    queue_lock.unlock();
    con.notify_one();
}

void pose_callback(const geometry_msgs::PoseStampedConstPtr &pose_msgs) {
    queue_lock.lock();
    //cout<<pose_msgs->header.stamp.toSec()<<endl;
    pose_q.push(pose_msgs);
    queue_lock.unlock();
    con.notify_one();
}

bool get_measurement(Measurement &m) {
    if(imu_q.empty() || feature_q.empty() || raw_image_q.empty() || pose_q.empty()) {
        return false;
    } 

    if(imu_q.front()->header.stamp.toSec() >= feature_q.front()->header.stamp.toSec() + o_p->td) {
        cout<<21<<endl;
        feature_q.pop();
        while(true) {
            if(raw_image_q.empty() || pose_q.empty()) {
                break;
            }
            
            if(raw_image_q.front()->header.stamp.toSec() < feature_q.front()->header.stamp.toSec()) {
                raw_image_q.pop();
                pose_q.pop();
            } else {
                break;
            }
        }

        return false;
    }

    if(imu_q.back()->header.stamp.toSec() <= feature_q.front()->header.stamp.toSec()) {
        return false;
    }

    m.feature_msgs = feature_q.front();
    feature_q.pop();

    while(true) {
        if(raw_image_q.empty() || pose_q.empty()) {
            break;
        }

        if(raw_image_q.front()->header.stamp.toSec() < m.feature_msgs->header.stamp.toSec()) {
            raw_image_q.pop();
            pose_q.pop();
        } else {
            m.image_msgs = raw_image_q.front();
            raw_image_q.pop();
            m.pose = pose_q.front();
            pose_q.pop();
            break;
        }
    }
    
    while(true) {
        if(imu_q.front()->header.stamp.toSec() <= m.feature_msgs->header.stamp.toSec()) {
            m.imu_msgs.push(imu_q.front());
            imu_q.pop();
        } else {
            m.imu_msgs.push(imu_q.front());
            break;
        }
    }

    return true;
}

void optimization() {        
    //double frame_time = 0;
    while(true) {        
        Measurement m;

        std::unique_lock<std::mutex> lock(queue_lock);
        con.wait(lock, [&] {return get_measurement(m);});
        lock.unlock();

        optimization_lock.lock();
        o_p->process(m.feature_msgs, m.imu_msgs, m.pose);
        //ROS_INFO("image freq: %f", m.feature_msgs->header.stamp.toSec() - frame_time);
        //frame_time = m.feature_msgs->header.stamp.toSec();
        //ROS_INFO("imu: %f", m.imu_msgs.back()->header.stamp.toSec() - m.imu_msgs.front()->header.stamp.toSec());
        //ROS_INFO("image-feature: %f", m.feature_msgs->header.stamp.toSec() - m.pose->header.stamp.toSec());
        //ROS_INFO("image-imu: %f", m.feature_msgs->header.stamp.toSec() - m.imu_msgs.back()->header.stamp.toSec());
        optimization_lock.unlock();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "back_end");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    b_p->read_ros_parameter(n);
    b_p->read_back_end_parameter();
    o_p = new Optimization(b_p);
    //ros::Subscriber sub_feature = n.subscribe("/tracking/feature", 2000, feature_callback);
    //ros::Subscriber sub_imu = n.subscribe(b_p->imu_topic, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    //ros::Subscriber sub_image = n.subscribe(b_p->image_topic, 2000, image_callback, ros::TransportHints().tcpNoDelay());    
    ros::Subscriber sub_feature = n.subscribe("/synthetic/feature", 2000, feature_callback);
    ros::Subscriber sub_imu = n.subscribe("/synthetic/imu", 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/synthetic/image", 2000, image_callback);  
    ros::Subscriber sub_restart = n.subscribe("/tracking/restart", 2000, restart_callback);
    ros::Subscriber sub_pose = n.subscribe("/synthetic/pose", 2000, pose_callback);
    //pub_kp_msgs = n.advertise<back_end::kf>("keyframe", 1000);
    //pub_up_kf_msgs = n.advertise<back_end::up_kf>("update_keyframe", 1000);

    thread optimization_thread{optimization};

    ros::spin();
    optimization_thread.join();
}