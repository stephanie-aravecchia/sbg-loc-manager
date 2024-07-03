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

    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SBGLocalizationProcessor::initCallback, this));
    
    odom_pub_ = nh_.advertise<nav_msgs::msg::Odometry>("odometry", 10);

}

void SBGLocalizationProcessor::run() {
  ros::Rate rate(10.0);
  while (ros::ok()){
    if (!init_complete_) {
      ROS_INFO("Starting initialization.")
      initCallback();
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
          listener_.waitForTransform(target_frame_, imu_frame_,ros::Time(0),ros::Duration(5.0));
      } catch (const tf2::TransformException & ex) {
          ROS_INFO("Could not transform %s to %s: %s",
          imu_frame_.c_str(), target_frame_.c_str(), ex.what());
          return;
      }
      initLocalRefFrame();
      initStaticTFBodyToImu();
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
    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf_buffer_.lookupTransform(
        target_frame_, imu_frame_,
        ros::Time(0));
    } catch (const tf2::TransformException & ex) {
      ROS_INFO("Could not transform %s to %s: %s",
        imu_frame_.c_str(), target_frame_.c_str(), ex.what());
      return;
    }
    imu_static_transform_ = t.transform;
}
//If the local ref is provided, we use it as the local ref frame
//Else, we set it to the last value in the msg at the time of initialization
void SBGLocalizationProcessor::initLocalRefFrame() {
  // if at least one is zero, no ref is provided
  // we initialize with current value 
  if ((local_ref_latitude_ * local_ref_longitude_ * local_ref_altitude_) < 1)  {
    utm_utils::initUTM(ekf_nav_data_.latitude, ekf_nav_data_.longitude, ekf_nav_data_.altitude, utm0_);
  }  else  {
    utm_utils::initUTM(local_ref_latitude_, local_ref_longitude_, local_ref_altitude_, utm0_);
  }
  // Publish UTM initial transformation.
  // UTM is always x: easting, y: northing, we assure z: altitude
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::Pose pose;
  pose.position.x = utm0_.easting;
  pose.position.y = utm0_.northing;
  pose.position.z = utm0_.altitude;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  fillTransform(reference_frame_, local_reference_frame_, pose, transform);
  static_tf_broadcaster_.sendTransform(transform);
}


void SBGLocalizationProcessor::fillTransform(const std::string &ref_parent_frame_id, const std::string &ref_child_frame_id, const geometry_msgs::msg::Pose &ref_pose, geometry_msgs::msg::TransformStamped &refTransformStamped)
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

void SBGLocalizationProcessor::ekfNavCallback(const sbg_driver::msg::SbgEkfNav::SharedPtr msg)
{
    ekf_nav_data_ = *msg;
}

void SBGLocalizationProcessor::imuDataCallback(const sbg_driver::msg::SbgImuData::SharedPtr msg)
{
    imu_data_ = *msg;
}

void SBGLocalizationProcessor::ekfQuatCallback(const sbg_driver::msg::SbgEkfQuat::SharedPtr msg)
{
    ekf_quat_data_ = *msg;
}
void SBGLocalizationProcessor::ekfEulerCallback(const sbg_driver::msg::SbgEkfEuler::SharedPtr msg)
{
    ekf_euler_data_ = *msg;
}

void SBGLocalizationProcessor::publishOdometry()
{
    nav_msgs::msg::Odometry odom_msg = nav_msgs::msg::Odometry();
    computeOdometryFromSbg(odom_msg);
    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped transform_stamped;
    fillTransform(local_reference_frame_, target_frame_, odom_msg.pose.pose, transform_stamped);

    tf_broadcaster_.sendTransform(transform_stamped);
}
    
void SBGLocalizationProcessor::convertNEDtoENU(const geometry_msgs::msg::Vector3& ned, geometry_msgs::msg::Vector3& enu) {
  enu.x = ned.x;
  enu.y = -ned.y;
  enu.z = M_PI_2 - ned.z;
}


//Convert the SBG Quaternion (in NED by default on the sensor) into ENU Euler Angles
void SBGLocalizationProcessor::getOrientationENUfromSBG(tf2::Quaternion& orientation) {
  //Convert quaternion to Euler (NED)
  tf2::Quaternion ned_quat;
  tf2::fromMsg(ekf_quat_data_.quaternion, ned_quat);
  tf2::Matrix3x3 m(ned_quat);
  geometry_msgs::msg::Vector3 ned_euler, enu_euler;
  m.getRPY(ned_euler.x, ned_euler.y, ned_euler.z);
  convertNEDtoENU(ned_euler,enu_euler);
  orientation.setRPY(enu_euler.x, enu_euler.y, enu_euler.z);
}

//We assume the SBG sensor is installed as required, X pointing in the robot forward direction, Z downward
//Then, how are we dealing with the flip in TF, in conversion, ....
void SBGLocalizationProcessor::computeOdometryFromSbg(nav_msgs::msg::Odometry &odom_msg) {
  
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = local_reference_frame_;
  odom_msg.child_frame_id = target_frame_;

  getRobotPoseFromSbg(odom_msg);
  getRobotTwistFromSbg(odom_msg);
  fillCovariance(odom_msg);
  

}

void SBGLocalizationProcessor::getRobotPoseFromSbg(nav_msgs::msg::Odometry &odom_msg) {
  //First, we convert the coordinates of the sensor in UTM
  double utm_northing, utm_easting;
  utm_utils::LLtoUTM(ekf_nav_data_.latitude,ekf_nav_data_.longitude,utm0_.zone,utm_northing,utm_easting);
 
  //Then, we get the orientation of the SBG sensor, in NED
  tf2::Quaternion orientation;
  getOrientationENUfromSBG(orientation);

  //We construct the pose of the sensor, in the local UTM frame
  geometry_msgs::msg::PoseStamped sensor_pose;
  sensor_pose.pose.position.x = utm_easting - utm0_.easting;;
  sensor_pose.pose.position.y = utm_northing - utm0_.northing;
  sensor_pose.pose.position.z = ekf_nav_data_.altitude - utm0_.altitude;
  
  //And finally, we can construct the Pose of the robot (base_frame) in the local UTM Frame
  //We assume there is a translation between the imu and the robot
  //We also assume the sensor is set as recommanded, z down, but that rotation is ignored here, 
  //because the position and orientation are already in ENU
  odom_msg.pose.pose.position.x = sensor_pose.pose.position.x + imu_static_transform_.translation.x;
  odom_msg.pose.pose.position.y = sensor_pose.pose.position.y + imu_static_transform_.translation.y;
  odom_msg.pose.pose.position.z = sensor_pose.pose.position.z + imu_static_transform_.translation.z;
  odom_msg.pose.pose.orientation = tf2::toMsg(orientation);

}

void SBGLocalizationProcessor::getRobotTwistFromSbg(nav_msgs::msg::Odometry &odom_msg) {
  //We assume the Twist of the robot is simply the Twist of the IMU after a rotation of Pi around x
  odom_msg.twist.twist.linear.x      = ekf_nav_data_.velocity.x;
  odom_msg.twist.twist.linear.y      = -ekf_nav_data_.velocity.y;
  odom_msg.twist.twist.linear.z      = -ekf_nav_data_.velocity.z;
  odom_msg.twist.twist.angular.x     = imu_data_.gyro.x;
  odom_msg.twist.twist.angular.y     = -imu_data_.gyro.y;
  odom_msg.twist.twist.angular.z     = -imu_data_.gyro.z;
}
  
void SBGLocalizationProcessor::fillCovariance(nav_msgs::msg::Odometry &odom_msg) {
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
  ros::init(argc, argv);
  SBGLocalizationProcessor loc;
  loc.run();
  ros::shutdown();
  return 0;
}