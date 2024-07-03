#ifndef SBG_LOCALIZATION_PROCESSOR_H
#define SBG_LOCALIZATION_PROCESSOR_H

#include <ros/ros.h>
#include <sbg_driver/msg/sbg_ekf_nav.h>
#include <sbg_driver/msg/sbg_imu_data.h>
#include <sbg_driver/msg/sbg_ekf_quat.h>
#include <sbg_driver/msg/sbg_ekf_euler.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include "localization_manager/LLtoUTM.h"


class SBGLocalizationProcessor {
public:
    SBGLocalizationProcessor();
    void run();

protected:
    void ekfNavCallback(const sbg_driver::msg::SbgEkfNav::SharedPtr msg);
    void imuDataCallback(const sbg_driver::msg::SbgImuData::SharedPtr msg);
    void ekfQuatCallback(const sbg_driver::msg::SbgEkfQuat::SharedPtr msg);
    void ekfEulerCallback(const sbg_driver::msg::SbgEkfEuler::SharedPtr msg);
    void publishOdometry();
    void initCallback();
    bool isFirstMsgReceived();
    void computeOdometryFromSbg(nav_msgs::msg::Odometry &odom_msg);
    void getRobotPoseFromSbg(nav_msgs::msg::Odometry &odom_msg);
    void getRobotTwistFromSbg(nav_msgs::msg::Odometry &odom_msg);
    void fillCovariance(nav_msgs::msg::Odometry &odom_msg);
    void initLocalRefFrame();
    void initStaticTFBodyToImu();
    void fillTransform(const std::string &ref_parent_frame_id, const std::string &ref_child_frame_id, const geometry_msgs::msg::Pose &ref_pose, geometry_msgs::msg::TransformStamped &ref_transform_stamped);
    void getOrientationENUfromSBG(tf2::Quaternion& orientation);
    void convertNEDtoENU(const geometry_msgs::msg::Vector3& ned, geometry_msgs::msg::Vector3& enu);

    ros::NodeHandle nh_;
    
    ros::Subscriber ekf_nav_sub_;
    ros::Subscriber imu_data_sub_;
    ros::Subscriber ekf_quat_sub_;
    ros::Subscriber ekf_euler_sub_;

    ros::Publisher odom_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_(tf_buffer_);
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;
    
    geometry_msgs::msg::TransformStamped local_utm_transform_;
    geometry_msgs::msg::Transform imu_static_transform_;
    utm_utils::UTM0 utm0_;

    bool init_complete_{false};
    double local_ref_latitude_;
    double local_ref_longitude_;
    double local_ref_altitude_;

    sbg_driver::msg::SbgEkfNav ekf_nav_data_;
    sbg_driver::msg::SbgImuData imu_data_;
    sbg_driver::msg::SbgEkfQuat ekf_quat_data_;
    sbg_driver::msg::SbgEkfEuler ekf_euler_data_;
    
    std::string reference_frame_;
    std::string local_reference_frame_;
    std::string target_frame_;
    std::string imu_frame_;

};

#endif  // SBG_LOCALIZATION_PROCESSOR_H