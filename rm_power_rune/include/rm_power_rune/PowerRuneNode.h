#include "rm_interfaces/msg/rune_gimbal_cmd.hpp"
#include "rm_power_rune/PowerRune.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

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
  
};
}