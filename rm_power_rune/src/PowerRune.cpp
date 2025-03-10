#include "rm_power_rune/PowerRune.h"

#include <thread>

namespace pka::power_rune {

extern std::mutex MUTEX;

// 构造函数，读取配置文件
PowerRune::PowerRune(fs::path config_path) : m_param{config_path} {}

// 执行一次检测
bool PowerRune::runOnce(const cv::Mat& image, double pitch, double yaw, double roll) {
    // 创建一个帧对象
    // 传入图像、时间戳、俯仰角、偏航角、横滚角
    Frame frame{image, std::chrono::steady_clock::now(), pitch, yaw, roll};
    
// 输出当前时间点，当前的俯仰角、偏航角、横滚角
#if CONSOLE_OUTPUT >= 2
    MUTEX.lock();
    std::cout << "------" << std::endl;
    std::cout
        << "current time point: "
        << std::chrono::duration_cast<std::chrono::microseconds>(frame.m_time.time_since_epoch()).count()
        << std::endl;
    std::cout << "current row: " << roll << std::endl;
    std::cout << "current pitch: " << pitch << std::endl;
    std::cout << "current yaw: " << yaw << std::endl;
    MUTEX.unlock();
#endif
    // 核心部分，首先检测目标，获取相机坐标系的坐标，然后计算目标点
    // 检测是否检测到目标
    if (m_detector.detect(frame) == false) {
        return false;
    }
    // 得到像素坐标系特征点，分别为装甲板内灯条的左上，右上，外灯条的中上，左下，右下，中心R。
    auto cameraPoints{m_detector.getCameraPoints()};
    // 计算目标点
    bool result = m_calculator.calculate(frame, cameraPoints);
    

#if SHOW_IMAGE >= 1
    if (result == true) {
        m_detector.drawTargetPoint(m_calculator.getPredictPixel());
    }
    m_detector.visualize();
    char key = cv::waitKey(1);
    if (key == ' ') {
        cv::waitKey(0);
    } else if (key == 'q') {
        std::exit(1);
    }
#endif
    return result;
}

std::pair<double, double> PowerRune::get_predict() {
  std::pair<double, double> result = m_calculator.getPredictPitchYaw();
  return result;
}

}  // namespace power_rune