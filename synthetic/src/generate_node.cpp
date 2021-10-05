#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include "generate_synthetic_data.hpp"

using namespace std;

ros::Publisher pub_image;
ros::Publisher pub_feature;
ros::Publisher pub_imu;
ros::Publisher pub_pose;
//ros::Publisher pub_path;
//ros::Publisher pub_cal_path;
//double first_publish_time = -1;

SyntheticData *s_d = new SyntheticData(20, 100, 20, 150, 90);

/*void publish_pose() {
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
}*/


/*void publish_cal_path() {
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
}*/


void publish() {
    double first_publish_time = ros::Time::now().toSec();
    double current_time = first_publish_time;
    double imu_delta_t = 1.0 / s_d->imu_freq;
    double image_delta_t = 1.0 / s_d->image_freq;

    int pub_imu_n = 1;
    int pub_image_n = 1;

    //cout<<"imu: "<<s_d->imu_header.size()<<"  "<<s_d->acc_with_noise_q.size()<<endl;
    //cout<<"frame: "<<s_d->frame_header.size()<<"  "<<s_d->features_pre_frame_wn.size()<<endl;

    while(true) {
        if(s_d->imu_header.empty() || s_d->frame_header.empty()) {
            break;
        } else {
            current_time = ros::Time::now().toSec();
            if(current_time < first_publish_time + 1) {
                continue;
            }
            //int i_h = s_d->imu_header.front();
            //int f_h = s_d->frame_header.front();

            if(current_time - first_publish_time - 1 >= imu_delta_t * pub_imu_n) {
                s_d->imu_header.pop();
                std_msgs::Header current_header;
                current_header.stamp = ros::Time::now();

                Eigen::Vector3d acc = s_d->acc_with_noise_q.front();
                Eigen::Vector3d gyr = s_d->gyr_with_noise_q.front();
                s_d->acc_with_noise_q.pop();
                s_d->gyr_with_noise_q.pop();
                
                sensor_msgs::Imu imu_msgs;
                imu_msgs.header = current_header;
                imu_msgs.angular_velocity.x = gyr(0);
                imu_msgs.angular_velocity.y = gyr(1);
                imu_msgs.angular_velocity.z = gyr(2);

                imu_msgs.linear_acceleration.x = acc(0);
                imu_msgs.linear_acceleration.y = acc(1);
                imu_msgs.linear_acceleration.z = acc(2);

                pub_imu.publish(imu_msgs);
                pub_imu_n ++;

                if(current_time - first_publish_time - 1 >= image_delta_t * pub_image_n) {
                    sensor_msgs::PointCloud feature_msgs;
                    sensor_msgs::Image image_msgs;
                    geometry_msgs::PoseStamped pose;
                    sensor_msgs::ChannelFloat32 id_of_kps;
                    sensor_msgs::ChannelFloat32 x_of_kps;
                    sensor_msgs::ChannelFloat32 y_of_kps;
                    sensor_msgs::ChannelFloat32 vx_of_kps;
                    sensor_msgs::ChannelFloat32 vy_of_kps;

                    queue<cv::Point2f> temp_features = s_d->features_pre_frame_wn[pub_image_n - 1];
                    queue<cv::Point2f> temp_features_v = s_d->features_v_pre_frame[pub_image_n - 1];
                    queue<int> temp_ids = s_d->feature_ids[pub_image_n - 1];
                    Eigen::Quaterniond temp_rotation = s_d->pub_rotation_q[pub_image_n - 1];
                    Eigen::Vector3d temp_translation = s_d->pub_translation_q[pub_image_n - 1];
                    feature_msgs.header = current_header; 
                    pose.header = current_header;
                    s_d->frame_header.pop();

                    while(true) {
                        if(temp_ids.empty()) {
                            break;
                        } else {
                            cv::Point2f feature = temp_features.front();
                            cv::Point2f feature_v = temp_features_v.front();
                            int id = temp_ids.front();
                            temp_features.pop();
                            temp_features_v.pop();
                            temp_ids.pop();

                            geometry_msgs::Point32 point;
                            point.x = feature.x;
                            point.y = feature.y;
                            point.z = 1;

                            feature_msgs.points.push_back(point);
                            id_of_kps.values.push_back(id);
                            x_of_kps.values.push_back(feature.x);
                            y_of_kps.values.push_back(feature.y);
                            vx_of_kps.values.push_back(feature_v.x);
                            vy_of_kps.values.push_back(feature_v.y);
                        }
                    }

                    feature_msgs.channels.push_back(id_of_kps);
                    feature_msgs.channels.push_back(x_of_kps);
                    feature_msgs.channels.push_back(y_of_kps);
                    feature_msgs.channels.push_back(vx_of_kps);
                    feature_msgs.channels.push_back(vy_of_kps);

                    image_msgs.header = current_header;
                    pose.pose.position.x = temp_translation(0);
                    pose.pose.position.y = temp_translation(1);
                    pose.pose.position.z = temp_translation(2);
                    pose.pose.orientation.x = temp_rotation.x();
                    pose.pose.orientation.y = temp_rotation.y();
                    pose.pose.orientation.z = temp_rotation.z();
                    pose.pose.orientation.w = temp_rotation.w();

                    pub_feature.publish(feature_msgs);
                    pub_image.publish(image_msgs);
                    pub_pose.publish(pose);
                    pub_image_n ++;
                    //cout<<i_h<<"  "<<f_h<<endl;
                }
            }
        }
    }
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "synthetic");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    pub_feature = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_imu = n.advertise<sensor_msgs::Imu>("imu", 1000);
    pub_image = n.advertise<sensor_msgs::Image>("image", 1000);

    s_d->generate_pose();
    s_d->generate_imu_data_with_noise();
    s_d->generate_image_data_with_noise();
    //s_d->generate_image_data();
    //publish_pose();
    publish();

    ros::spin();    
}
