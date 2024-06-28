#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sbg_driver/msg/sbg_ekf_nav.hpp>

class Test: public rclcpp::Node 
{
  private:
    void testCallback(const sbg_driver::msg::SbgEkfNav::SharedPtr msg) {
    //void testCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    //void testCallback(const std_msgs::msg::Int16::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "msg ");
      };
      //rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_;
      //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
      rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr ekf_nav_sub_;

  public:
    Test() : Node("test") {
     // sub_ = this->create_subscription<std_msgs::msg::Int16>(
     //     "/test_msg", 1,
     //     std::bind(&Test::testCallback, this, std::placeholders::_1));
     // sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
     //     "/hus/platform/odom", 1,
     //     std::bind(&Test::testCallback, this, std::placeholders::_1));
    ekf_nav_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
        "/hus/sbg/ekf_nav", 1,
        std::bind(&Test::testCallback, this, std::placeholders::_1));
    //};
    };
};    
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test>());
  std::cout<<"spin returned"<<std::endl;
  rclcpp::shutdown();

  return 0;
}
