#pragma once

#include "rm_power_rune/Calculator.h"
#include "rm_power_rune/Detector.h"
#include "rm_power_rune/Param.h"
#include "rm_power_rune/Utility.h"
#include "rm_utils/url_resolver.hpp"
#include <filesystem>
#include <vector>

// #define CONFIG_PATH "../config.yaml"
namespace fs = std::filesystem;
namespace pka::power_rune {

class PowerRune {
   public:
    PowerRune(fs::path config_path);
    bool runOnce(const cv::Mat& image, double pitch, double yaw, double roll = 0.0);
    std::pair<double, double> get_predict();

  private:
    Param m_param;
    Detector m_detector;
    Calculator m_calculator;
};

} // namespace pka::power_rune
