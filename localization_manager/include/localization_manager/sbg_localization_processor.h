#ifndef SBG_LOCALIZATION_PROCESSOR_H
#define SBG_LOCALIZATION_PROCESSOR_H

#include <rclcpp/rclcpp.hpp>
#include <sbg_driver/msg/sbg_ekf_nav.hpp>
#include <sbg_driver/msg/sbg_imu_data.hpp>
#include <sbg_driver/msg/sbg_ekf_quat.hpp>
#include <sbg_driver/msg/sbg_ekf_euler.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "localization_manager/LLtoUTM.h"


class SBGLocalizationProcessor : public rclcpp::Node
{
public:
    SBGLocalizationProcessor();

private:
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
    rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr ekf_nav_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgImuData>::SharedPtr imu_data_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgEkfQuat>::SharedPtr ekf_quat_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgEkfEuler>::SharedPtr ekf_euler_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::TimerBase::SharedPtr init_timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    
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