#include "rm_interfaces/msg/rune_gimbal_cmd.hpp"
#include "rm_power_rune/PowerRune.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pka::power_rune {
    

class PowerRuneNode : public rclcpp::Node {
public:
  PowerRuneNode(const rclcpp::NodeOptions &options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  std::unique_ptr<PowerRune> initPowerRune();
  
  // 大符检测器
  std::unique_ptr<PowerRune> power_rune_;
  // 图像订阅者
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // 云台信息发布
  rclcpp::Publisher<rm_interfaces::msg::RuneGimbalCmd>::SharedPtr gimbal_pub_;

  // 订阅云台tf
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  
  // 获取云台俯仰角、偏航角
  bool getGimbalPose(double &pitch, double &yaw);
  double gimbal_pitch_;
  double gimbal_yaw_;
};
}