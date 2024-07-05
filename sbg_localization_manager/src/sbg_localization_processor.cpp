#include "localization_manager/sbg_localization_processor.h"
#include <iostream>

SBGLocalizationProcessor::SBGLocalizationProcessor() : nh_("~")
{
    ekf_nav_sub_ = nh_.subscribe("sbg_ekf_nav", 1, &SBGLocalizationProcessor::ekfNavCallback, this);
    imu_data_sub_ = nh_.subscribe("sbg_imu_nav", 1, &SBGLocalizationProcessor::imuDataCallback, this);
    ekf_quat_sub_ = nh_.subscribe("sbg_ekf_quat", 1, &SBGLocalizationProcessor::ekfQuatCallback, this);
    ekf_euler_sub_ = nh_.subscribe("sbg_ekf_euler", 1, &SBGLocalizationProcessor::ekfEulerCallback, this);

    nh_.param<std::string>("reference_frame", reference_frame_,"utm");
    nh_.param<std::string>("local_reference_frame", local_reference_frame_,"utm_local_gte");
    nh_.param<std::string>("target_frame", target_frame_,"base_link");
    nh_.param<std::string>("imu_frame", imu_frame_,"sbg");
    
    nh_.param<double>("local_ref_latitude", local_ref_latitude_,0.0);
    nh_.param<double>("local_ref_longitude", local_ref_longitude_,0.0);
    nh_.param<double>("local_ref_altitude", local_ref_altitude_,0.0);
    nh_.param<bool>("is_imu_child", is_imu_child_,true);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 10);

}

void SBGLocalizationProcessor::run() {
  ros::Rate init_rate(1.0);
  ros::Rate rate(10.0);
  while (ros::ok()){
    if (!init_complete_) {
      //ROS_INFO("Starting initialization.");
      initCallback();
      init_rate.sleep();
    } else {
      publishOdometry();
    }
    rate.sleep();
    ros::spinOnce();
  }

}

void SBGLocalizationProcessor::initCallback() {
    //Check if First Msg is Received:
    if (isFirstMsgReceived()) {
      //And wait for the transform
      try {
          ROS_INFO("Wait for Transfom from %s to %s",imu_frame_.c_str(), target_frame_.c_str());
          tf_listener_.waitForTransform(target_frame_, imu_frame_,ros::Time(0),ros::Duration(5.0));
      } catch (const tf2::TransformException & ex) {
          ROS_INFO("Could not transform %s to %s: %s",
          imu_frame_.c_str(), target_frame_.c_str(), ex.what());
          return;
      }
      //If the value is not valid, we wait
      try {
        initStaticTFBodyToImu();
      } catch (const tf2::TransformException & ex) {
          ROS_INFO("InitStaticTFBodyToImu failed: %s",ex.what());
          return;
      }
      initLocalRefFrame();
      ROS_INFO("Initialization complete.");
      init_complete_ = true;
    } else {
        ROS_INFO("Waiting for first message to start initialization.");
    }

}
bool SBGLocalizationProcessor::isFirstMsgReceived() {
  //Check if a first message is received for ekfNav, imuData and ekfQuat
  //ekfEuler is used to fill some covariance only, not mandatory
  if (ekf_nav_data_.header.frame_id == "") return false;
  if (ekf_quat_data_.header.frame_id == "") return false;
  if (imu_data_.header.frame_id == "") return false;
  return true;
}

void SBGLocalizationProcessor::initStaticTFBodyToImu() {
  ROS_INFO("Initialize static body to imu.");
    geometry_msgs::TransformStamped t_msg;
    tf::StampedTransform t;
    try {
        tf_listener_.lookupTransform(
        target_frame_, imu_frame_,
        ros::Time(0), t);
    } catch (const tf2::TransformException & ex) {
      ROS_INFO("Could not transform %s to %s: %s",
        imu_frame_.c_str(), target_frame_.c_str(), ex.what());
      throw ex;
    }
    tf::transformStampedTFToMsg(t, t_msg);
    
    //Check if the quaternion is valid, else, nothing works:
    bool valid = std::abs((t_msg.transform.rotation.w * t_msg.transform.rotation.w
                          + t_msg.transform.rotation.x * t_msg.transform.rotation.x
                          + t_msg.transform.rotation.y * t_msg.transform.rotation.y
                          + t_msg.transform.rotation.z * t_msg.transform.rotation.z) - 1.0f) < 10e-3;
    if (!valid) {
      throw tf2::TransformException("Invalid Quaternion");
    }
    imu_static_transform_ = t_msg.transform;
}
//If the local ref is provided, we use it as the local ref frame
//Else, we set it to the last value in the msg at the time of initialization
void SBGLocalizationProcessor::initLocalRefFrame() {
  ROS_INFO("Initialize and broadcast static tf local ref frame.");
  // if at least one is zero, no ref is provided
  // we initialize with current value 
  if ((local_ref_latitude_ * local_ref_longitude_ * local_ref_altitude_) < 1)  {
    utm_utils::initUTM(ekf_nav_data_.latitude, ekf_nav_data_.longitude, ekf_nav_data_.altitude, utm0_);
  }  else  {
    utm_utils::initUTM(local_ref_latitude_, local_ref_longitude_, local_ref_altitude_, utm0_);
  }
  // Publish UTM initial transformation.
  // x: easting, y: northing, we assure z: altitude
  geometry_msgs::TransformStamped transform;
  geometry_msgs::Pose pose;
  pose.position.x = utm0_.easting;
  pose.position.y = utm0_.northing;
  pose.position.z = utm0_.altitude;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  fillTransform(reference_frame_, local_reference_frame_, pose, transform);
  tf_static_broadcaster_.sendTransform(transform);
}


void SBGLocalizationProcessor::fillTransform(const std::string &ref_parent_frame_id, const std::string &ref_child_frame_id, const geometry_msgs::Pose &ref_pose, geometry_msgs::TransformStamped &refTransformStamped)
{

  refTransformStamped.header.stamp = ros::Time::now();
  refTransformStamped.header.frame_id = ref_parent_frame_id;
  refTransformStamped.child_frame_id = ref_child_frame_id;
  refTransformStamped.transform.translation.x = ref_pose.position.x;
  refTransformStamped.transform.translation.y = ref_pose.position.y;
  refTransformStamped.transform.translation.z = ref_pose.position.z;
  refTransformStamped.transform.rotation.x = ref_pose.orientation.x;
  refTransformStamped.transform.rotation.y = ref_pose.orientation.y;
  refTransformStamped.transform.rotation.z = ref_pose.orientation.z;
  refTransformStamped.transform.rotation.w = ref_pose.orientation.w;

}

void SBGLocalizationProcessor::ekfNavCallback(const sbg_driver::SbgEkfNavConstPtr msg)
{
    ekf_nav_data_ = *msg;
}

void SBGLocalizationProcessor::imuDataCallback(const sbg_driver::SbgImuDataConstPtr msg)
{
    imu_data_ = *msg;
}

void SBGLocalizationProcessor::ekfQuatCallback(const sbg_driver::SbgEkfQuatConstPtr msg)
{
    ekf_quat_data_ = *msg;
}
void SBGLocalizationProcessor::ekfEulerCallback(const sbg_driver::SbgEkfEulerConstPtr msg)
{
    ekf_euler_data_ = *msg;
}

void SBGLocalizationProcessor::publishOdometry()
{
    nav_msgs::Odometry odom_msg = nav_msgs::Odometry();
    computeOdometryFromSbg(odom_msg);
    odom_pub_.publish(odom_msg);

    geometry_msgs::TransformStamped transform_stamped;
    //If the imu frame is child of base_link, default behavior:
    if (is_imu_child_) {
      fillTransform(local_reference_frame_, target_frame_, odom_msg.pose.pose, transform_stamped);
    } else {
      //we need to compute and broadcast the transform between local and sbg
      setTransformWithImuParent(odom_msg, transform_stamped);
    }
    tf_broadcaster_.sendTransform(transform_stamped);
}

void SBGLocalizationProcessor::setTransformWithImuParent(nav_msgs::Odometry &odom_msg, geometry_msgs::TransformStamped &transform_stamped) {
  tf::Transform sensor_pose;
  tf::Pose robot_pose;
  tf::Pose t_robot_to_sensor;
  tf::poseMsgToTF(odom_msg.pose.pose, robot_pose);
  tf::transformMsgToTF(imu_static_transform_, t_robot_to_sensor);

  sensor_pose = robot_pose * t_robot_to_sensor;
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(sensor_pose, pose);
  fillTransform(local_reference_frame_, imu_frame_, pose, transform_stamped);
}

void SBGLocalizationProcessor::convertNEDtoENU(const geometry_msgs::Vector3& ned, geometry_msgs::Vector3& enu) {
  enu.x = ned.x;
  enu.y = -ned.y;
  enu.z = M_PI_2 - ned.z;
}


//Convert the SBG Quaternion (in NED by default on the sensor) into ENU Euler Angles
void SBGLocalizationProcessor::getOrientationENUfromSBG(tf::Quaternion& orientation) {
  //Convert quaternion to Euler (NED)
  tf::Quaternion ned_quat;
  tf::quaternionMsgToTF(ekf_quat_data_.quaternion, ned_quat);
  tf::Matrix3x3 m(ned_quat);
  geometry_msgs::Vector3 ned_euler, enu_euler;
  m.getRPY(ned_euler.x, ned_euler.y, ned_euler.z);
  convertNEDtoENU(ned_euler,enu_euler);
  orientation.setRPY(enu_euler.x, enu_euler.y, enu_euler.z);
}

//We assume the SBG sensor is installed as required, X pointing in the robot forward direction, Z downward
//We also assume the sbg_driver is set to NED
void SBGLocalizationProcessor::computeOdometryFromSbg(nav_msgs::Odometry &odom_msg) {
  
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = local_reference_frame_;
  odom_msg.child_frame_id = target_frame_;

  getRobotPoseFromSbg(odom_msg);
  getRobotTwistFromSbg(odom_msg);
  fillCovariance(odom_msg);
  

}

void SBGLocalizationProcessor::getIMUPose(geometry_msgs::PoseStamped &sensor_pose) {
  //First, we convert the coordinates of the sensor in UTM
  double utm_northing, utm_easting;
  utm_utils::LLtoUTM(ekf_nav_data_.latitude,ekf_nav_data_.longitude,utm0_.zone,utm_northing,utm_easting);
 
  //Then, we get the orientation of the SBG sensor, in NED
  tf::Quaternion orientation;
  getOrientationENUfromSBG(orientation);

  //We construct the pose of the sensor, in the local UTM frame
  sensor_pose.pose.position.x = utm_easting - utm0_.easting;;
  sensor_pose.pose.position.y = utm_northing - utm0_.northing;
  sensor_pose.pose.position.z = ekf_nav_data_.altitude - utm0_.altitude;

  tf::quaternionTFToMsg(orientation, sensor_pose.pose.orientation);

}

void SBGLocalizationProcessor::getRobotPoseFromSbg(nav_msgs::Odometry &odom_msg) {
  
  geometry_msgs::PoseStamped sensor_pose;
  getIMUPose(sensor_pose);
  //And finally, we can construct the Pose of the robot (base_frame) in the local UTM Frame
  //We assume there is a translation between the imu and the robot
  //We also assume the sensor is set as recommanded, z down, but that rotation is ignored here, 
  //because the position and orientation in the odom msg are already in ENU

  odom_msg.pose.pose.position.x = sensor_pose.pose.position.x + imu_static_transform_.translation.x;
  odom_msg.pose.pose.position.y = sensor_pose.pose.position.y + imu_static_transform_.translation.y;
  odom_msg.pose.pose.position.z = sensor_pose.pose.position.z + imu_static_transform_.translation.z;
  
  odom_msg.pose.pose.orientation = sensor_pose.pose.orientation;

}

void SBGLocalizationProcessor::getRobotTwistFromSbg(nav_msgs::Odometry &odom_msg) {
  //We assume the Twist of the robot is simply the Twist of the IMU after a rotation of Pi around x
  odom_msg.twist.twist.linear.x      = ekf_nav_data_.velocity.x;
  odom_msg.twist.twist.linear.y      = -ekf_nav_data_.velocity.y;
  odom_msg.twist.twist.linear.z      = -ekf_nav_data_.velocity.z;
  odom_msg.twist.twist.angular.x     = imu_data_.gyro.x;
  odom_msg.twist.twist.angular.y     = -imu_data_.gyro.y;
  odom_msg.twist.twist.angular.z     = -imu_data_.gyro.z;
}
  
void SBGLocalizationProcessor::fillCovariance(nav_msgs::Odometry &odom_msg) {
  // Fill the covariance as in SBG driver
  // Compute convergence angle.
  double longitudeRad      = utm_utils::degToRadD(ekf_nav_data_.longitude);
  double latitudeRad       = utm_utils::degToRadD(ekf_nav_data_.latitude);
  double central_meridian  = utm_utils::degToRadD(utm_utils::computeMeridian(utm0_.zone));
  double convergence_angle = atan(tan(longitudeRad - central_meridian) * sin(latitudeRad));
  double std_east  = ekf_nav_data_.position_accuracy.x;
  double std_north = ekf_nav_data_.position_accuracy.y;
  double std_x = std_north * cos(convergence_angle) - std_east * sin(convergence_angle);
  double std_y = std_north * sin(convergence_angle) + std_east * cos(convergence_angle);
  double std_z = ekf_nav_data_.position_accuracy.z;
  odom_msg.pose.covariance[0*6 + 0] = std_x * std_x;
  odom_msg.pose.covariance[1*6 + 1] = std_y * std_y;
  odom_msg.pose.covariance[2*6 + 2] = std_z * std_z;
  odom_msg.pose.covariance[3*6 + 3] = ekf_euler_data_.accuracy.x * ekf_euler_data_.accuracy.x;
  odom_msg.pose.covariance[4*6 + 4] = ekf_euler_data_.accuracy.y * ekf_euler_data_.accuracy.y;
  odom_msg.pose.covariance[5*6 + 5] = ekf_euler_data_.accuracy.z * ekf_euler_data_.accuracy.z;
  odom_msg.twist.covariance[0*6 + 0] = ekf_nav_data_.velocity_accuracy.x * ekf_nav_data_.velocity_accuracy.x;
  odom_msg.twist.covariance[1*6 + 1] = ekf_nav_data_.velocity_accuracy.y * ekf_nav_data_.velocity_accuracy.y;
  odom_msg.twist.covariance[2*6 + 2] = ekf_nav_data_.velocity_accuracy.z * ekf_nav_data_.velocity_accuracy.z;
  odom_msg.twist.covariance[3*6 + 3] = 0;
  odom_msg.twist.covariance[4*6 + 4] = 0;
  odom_msg.twist.covariance[5*6 + 5] = 0;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "sbg_localization_processor");
  SBGLocalizationProcessor loc;
  loc.run();
  ros::shutdown();
  return 0;
}