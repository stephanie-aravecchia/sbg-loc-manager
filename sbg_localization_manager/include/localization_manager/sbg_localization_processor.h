#ifndef SBG_LOCALIZATION_PROCESSOR_H
#define SBG_LOCALIZATION_PROCESSOR_H

#include <ros/ros.h>
#include <sbg_driver/SbgEkfNav.h>
#include <sbg_driver/SbgImuData.h>
#include <sbg_driver/SbgEkfQuat.h>
#include <sbg_driver/SbgEkfEuler.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "localization_manager/LLtoUTM.h"



/* 
    This class is intended to process sbg_driver navigation messages.
    The IMU outputs its estimated localization as custom messages (SbgEkfNav, ...).
    This class process those messages and outputs and Odometry and broadcast the TF
    The Pose of the robot is expressed in UTM, through an intermediate local frame.
    The coordinates of the local frame can be set manually (local_ref params). Else
    they will be defaulted to the coordinates of the robot when the node starts.
    This class makes the following assumptions:
        - the SBG IMU is set as recommanded (X forward, Z down)
        - the SBG IMU is set to "NED"
        - the UTM and local UTM frames are ENU
*/
class SBGLocalizationProcessor {
public:
    SBGLocalizationProcessor();
    void run();

protected:
    void ekfNavCallback(const sbg_driver::SbgEkfNavConstPtr msg);
    void imuDataCallback(const sbg_driver::SbgImuDataConstPtr msg);
    void ekfQuatCallback(const sbg_driver::SbgEkfQuatConstPtr msg);
    void ekfEulerCallback(const sbg_driver::SbgEkfEulerConstPtr msg);
    void publishOdometry();
    void initCallback();
    bool isFirstMsgReceived();
    void computeOdometryFromSbg(nav_msgs::Odometry &odom_msg);
    void getRobotPoseFromSbg(nav_msgs::Odometry &odom_msg);
    void getRobotTwistFromSbg(nav_msgs::Odometry &odom_msg);
    void getIMUPose(geometry_msgs::PoseStamped &sensor_pose);
    void setTransformWithImuParent(nav_msgs::Odometry &odom_msg, geometry_msgs::TransformStamped &transform_stamped);
    void fillCovariance(nav_msgs::Odometry &odom_msg);
    void initLocalRefFrame();
    void initStaticTFBodyToImu();
    void fillTransform(const std::string &ref_parent_frame_id, const std::string &ref_child_frame_id, const geometry_msgs::Pose &ref_pose, geometry_msgs::TransformStamped &ref_transform_stamped);
    void getOrientationENUfromSBG(tf::Quaternion& orientation);
    void convertNEDtoENU(const geometry_msgs::Vector3& ned, geometry_msgs::Vector3& enu);

    ros::NodeHandle nh_;
    
    ros::Subscriber ekf_nav_sub_;
    ros::Subscriber imu_data_sub_;
    ros::Subscriber ekf_quat_sub_;
    ros::Subscriber ekf_euler_sub_;

    ros::Publisher odom_pub_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;
    
    geometry_msgs::TransformStamped local_utm_transform_;
    geometry_msgs::Transform imu_static_transform_;
    utm_utils::UTM0 utm0_;

    bool init_complete_{false};
    double local_ref_latitude_;
    double local_ref_longitude_;
    double local_ref_altitude_;
    bool is_imu_child_;

    sbg_driver::SbgEkfNav ekf_nav_data_;
    sbg_driver::SbgImuData imu_data_;
    sbg_driver::SbgEkfQuat ekf_quat_data_;
    sbg_driver::SbgEkfEuler ekf_euler_data_;
    
    std::string reference_frame_;
    std::string local_reference_frame_;
    std::string target_frame_;
    std::string imu_frame_;

};

#endif  // SBG_LOCALIZATION_PROCESSOR_H