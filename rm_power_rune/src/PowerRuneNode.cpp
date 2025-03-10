#include "rm_power_rune/PowerRuneNode.h"
#include "rm_power_rune/PowerRune.h"
#include "rm_utils/url_resolver.hpp"
#include "rm_utils/logger/log.hpp"
#include "cv_bridge/cv_bridge.h"

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
}

void PowerRuneNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    // 获取图像
    cv::Mat image = cv_bridge::toCvShare(img_msg, "bgr8")->image;

    rm_interfaces::msg::RuneGimbalCmd gimbal_cmd;
    // 运行大符检测器
    if (power_rune_ && power_rune_->runOnce(image, 0.0, 0.0)) {
      std::pair<double, double> gimbal_position =
          power_rune_->get_predict();
        gimbal_cmd.pitch = gimbal_position.first;
        gimbal_cmd.yaw = gimbal_position.second;
    }else{
        gimbal_cmd.pitch = 0.0;
        gimbal_cmd.yaw = 0.0;
    }
    // 发布云台信息
    gimbal_pub_->publish(gimbal_cmd);
}

std::unique_ptr<PowerRune> initPowerRune(){
    // 获取配置文件路径
    namespace fs = std::filesystem;
    fs::path model_path = utils::URLResolver::getResolvedPath(
        "package://rm_power_rune/config/config.yaml");
    
    // 创建PowerRune类
    auto power_rune = std::make_unique<PowerRune>(model_path);
    return power_rune;
}

}