#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include "generate_synthetic_data.hpp"

using namespace std;

ros::Publisher pub_image;
ros::Publisher pub_feature;
ros::Publisher pub_imu;
ros::Publisher pub_pose;
ros::Publisher pub_path;
ros::Publisher pub_cal_path;

SyntheticData *s_d = new SyntheticData(20, 100, 20, 100, 90);

void publish_pose() {
    uint32_t shape = visualization_msgs::Marker::CUBE;
    double current_time = ros::Time::now().toSec();
    double last_time = current_time;
    while(true) {
        current_time = ros::Time::now().toSec();
        if(s_d->pub_rotation_q.empty()) {
            break;
        } else if(current_time - last_time > 0.2){

            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "basic_shapes";
            marker.id = 0;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;

            Eigen::Vector3d translation = s_d->pub_translation_q.front();
            s_d->pub_translation_q.pop();
            Eigen::Quaterniond rotation = s_d->pub_rotation_q.front();
            s_d->pub_rotation_q.pop();
            marker.pose.position.x = translation(0);
            marker.pose.position.y = translation(1);
            marker.pose.position.z = translation(2);

            marker.pose.orientation.x = rotation.x();
            marker.pose.orientation.y = rotation.y();
            marker.pose.orientation.z = rotation.z();
            marker.pose.orientation.w = rotation.w();

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration();
            pub_pose.publish(marker);

            last_time = current_time;
        }
    }
}

void publish_path() {
    double current_time = ros::Time::now().toSec();
    double last_time = current_time;
    nav_msgs::Path path;
    path.header.frame_id = "world";

    while(true) {
        current_time = ros::Time::now().toSec();
        if(s_d->pub_rotation_q.empty()) {
            break;
        } else if(current_time - last_time > 0.1){
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = ros::Time::now();
            path.header.stamp = ros::Time::now();
           
            Eigen::Vector3d translation = s_d->pub_translation_q.front();
            s_d->pub_translation_q.pop();
            Eigen::Quaterniond rotation = s_d->pub_rotation_q.front();
            s_d->pub_rotation_q.pop();
            pose.pose.position.x = translation(0);
            pose.pose.position.y = translation(1);
            pose.pose.position.z = translation(2);

            pose.pose.orientation.x = rotation.x();
            pose.pose.orientation.y = rotation.y();
            pose.pose.orientation.z = rotation.z();
            pose.pose.orientation.w = rotation.w();

            path.poses.push_back(pose);
            pub_path.publish(path);
            pub_pose.publish(pose);
            last_time = current_time;
        }
    }
}


void publish_cal_path() {
    double current_time = ros::Time::now().toSec();
    double last_time = current_time;
    nav_msgs::Path path;
    path.header.frame_id = "world";
    int i = 0;

    while(true) {
        current_time = ros::Time::now().toSec();
        if(s_d->pub_rotation_q.empty() || i == static_cast<int>(s_d->cal_rotation_q.size())) {
            break;
        } else if(current_time - last_time > 0.1){
            geometry_msgs::PoseStamped pose;
            geometry_msgs::PoseStamped cal_pose;

            pose.header.frame_id = "world";
            pose.header.stamp = ros::Time::now();
            cal_pose.header = pose.header;
            path.header = pose.header;
           
            Eigen::Vector3d translation = s_d->pub_translation_q.front();
            s_d->pub_translation_q.pop();
            Eigen::Quaterniond rotation = s_d->pub_rotation_q.front();
            s_d->pub_rotation_q.pop();

            pose.pose.position.x = translation(0);
            pose.pose.position.y = translation(1);
            pose.pose.position.z = translation(2);

            pose.pose.orientation.x = rotation.x();
            pose.pose.orientation.y = rotation.y();
            pose.pose.orientation.z = rotation.z();
            pose.pose.orientation.w = rotation.w();

            translation = s_d->cal_translation_q[i];
            rotation = s_d->cal_rotation_q[i];
            i ++;

            cal_pose.pose.position.x = translation(0);
            cal_pose.pose.position.y = translation(1);
            cal_pose.pose.position.z = translation(2);

            cal_pose.pose.orientation.x = rotation.x();
            cal_pose.pose.orientation.y = rotation.y();
            cal_pose.pose.orientation.z = rotation.z();
            cal_pose.pose.orientation.w = rotation.w();

            path.poses.push_back(cal_pose);
            pub_cal_path.publish(path);
            pub_pose.publish(pose);
            
            last_time = current_time;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "synthetic");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_cal_path = n.advertise<nav_msgs::Path>("cal_path", 1000);

    s_d->generate_pose();
    s_d->generate_imu_data();
    s_d->mid_point_intergration();
    //publish_pose();
    publish_cal_path();

    ros::spin();    
}
