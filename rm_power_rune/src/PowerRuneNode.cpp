#include "rm_power_rune/PowerRuneNode.h"
#include "cv_bridge/cv_bridge.h"
#include "rm_interfaces/msg/rune_gimbal_cmd.hpp"
#include "rm_power_rune/PowerRune.h"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/url_resolver.hpp"
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

namespace pka::power_rune {

PowerRuneNode::PowerRuneNode(const rclcpp::NodeOptions &options)
    : Node("power_rune", options) {
  PKA_REGISTER_LOGGER("power_rune", "~/fyt2024-log", INFO);
  PKA_INFO("power_rune", "Starting PowerRuneNode!");

  // 创建图像订阅者
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      std::bind(&PowerRuneNode::imageCallback, this, std::placeholders::_1));

  // 创建云台信息发布
  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::RuneGimbalCmd>(
      "power_rune/gimbal_cmd", rclcpp::SensorDataQoS());

  // 初始化大符检测器
  power_rune_ = initPowerRune();

  // tf2初始化
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}

void PowerRuneNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  // 获取图像
  cv::Mat image = cv_bridge::toCvShare(img_msg, "bgr8")->image;

  rm_interfaces::msg::RuneGimbalCmd gimbal_cmd;
  
  // 运行大符检测器
  if (power_rune_ &&
      PowerRuneNode::getGimbalPose(gimbal_cmd.pitch, gimbal_cmd.yaw) &&
      power_rune_->runOnce(image, gimbal_cmd.pitch, gimbal_cmd.yaw)) {
    std::pair<double, double> gimbal_position = power_rune_->get_predict();
    gimbal_cmd.pitch = gimbal_position.first;
    gimbal_cmd.yaw = gimbal_position.second;
  } else {
    gimbal_cmd.pitch = 0.0;
    gimbal_cmd.yaw = 0.0;
  }
  // 发布云台信息
  gimbal_pub_->publish(gimbal_cmd);
}

std::unique_ptr<PowerRune> initPowerRune() {
  // 获取配置文件路径
  namespace fs = std::filesystem;
  fs::path model_path = utils::URLResolver::getResolvedPath(
      "package://rm_power_rune/config/config.yaml");

  // 创建PowerRune类
  auto power_rune = std::make_unique<PowerRune>(model_path);
  return power_rune;
}

/**
 * 获取当前云台的pitch和yaw角度
 * @param pitch 输出参数，俯仰角(弧度)
 * @param yaw 输出参数，偏航角(弧度)
 * @return 是否成功获取到云台姿态
 */
bool PowerRuneNode::getGimbalPose(double &pitch, double &yaw) {
  try {
    // 查询最新的云台变换
    geometry_msgs::msg::TransformStamped transform_stamped =
        tf2_buffer_->lookupTransform("odom", "gimbal_link", tf2::TimePointZero);

    // 从四元数提取欧拉角
    tf2::Quaternion q;
    tf2::fromMsg(transform_stamped.transform.rotation, q);

    // 获取欧拉角(按RPY顺序)
    double roll;
    tf2::getEulerYPR(q, yaw, pitch, roll); // 注意顺序是YPR

    // 如果需要角度制而非弧度制
    // pitch = pitch * 180.0 / M_PI;
    // yaw = yaw * 180.0 / M_PI;

    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "获取云台姿态失败: %s", ex.what());
    return false;
  }
}
} // namespace pka::power_rune