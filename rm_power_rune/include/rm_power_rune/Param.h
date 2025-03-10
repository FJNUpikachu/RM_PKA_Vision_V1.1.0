#pragma once
#include "rm_power_rune/Utility.h"

namespace power_rune {

/**
 * 0: 不显示
 * 1: 显示箭头、装甲板、中心、预测点
 * 2: 在 1 的基础上显示灯条、roi
 * 3: 在 2 的基础上显示二值化图片
 */
#define SHOW_IMAGE 0

// 控制台输出级别
#define CONSOLE_OUTPUT 1

struct Param {
    Param() = default;
    Param(const std::string& filename);

    void load(const std::string& filename);

    inline static float IMAGE_WIDTH, IMAGE_HEIGHT;  // 图像宽度和高度

    // 定义常用颜色
    const inline static cv::Scalar RED{0, 0, 255};        // 红色
    const inline static cv::Scalar BLUE{255, 0, 0};       // 蓝色
    const inline static cv::Scalar GREEN{0, 255, 0};      // 绿色
    const inline static cv::Scalar WHITE{255, 255, 255};  // 白色
    const inline static cv::Scalar YELLOW{0, 255, 255};   // 黄色
    const inline static cv::Scalar PURPLE{128, 0, 128};   // 紫色

    inline static int FPS;  // 帧率

    inline static cv::Scalar DRAW_COLOR;  // 绘图颜色

    // 装甲板尺寸参数
    inline static double ARMOR_OUTSIDE_WIDTH;   // 装甲板外部宽度
    inline static double ARMOR_INSIDE_WIDTH;    // 装甲板内部宽度
    inline static double ARMOR_INSIDE_Y;        // 装甲板内部Y坐标
    inline static double ARMOR_OUTSIDE_Y;       // 装甲板外部Y坐标
    inline static double ARMOR_OUTSIDE_HEIGHT;  // 装甲板外部高度

    inline static const double POWER_RUNE_RADIUS{700.0};  // 能量机关半径(mm)

    inline static Mode MODE;  // 运行模式(小符/大符)

    inline static const double GRAVITY{9.800};  // 重力加速度

    // 子弹速度参数
    inline static double CURRENT_BULLET_SPEED;  // 当前子弹速度
    inline static double MIN_BULLET_SPEED;      // 最小子弹速度
    inline static double DEFAULT_BULLET_SPEED;  // 默认子弹速度

    // 相机到云台的转换参数
    inline static std::array<double, 3> CAMERA_TO_GIMBAL_TRANSLATION_VECTOR;  // 相机到云台的平移向量
    inline static const std::array<double, 3> CAMERA_TO_GIMBAL_ROTATION_VECTOR{0.0, 0.0,
                                                                               0.0};  // 相机到云台的旋转向量

    inline static const double ANGLE_BETWEEN_FAN_BLADES{72 * CV_PI / 180};  // 扇叶之间的角度(弧度)

    // 补偿参数
    inline static double COMPANSATE_TIME;   // 时间补偿
    inline static double COMPANSATE_PITCH;  // 俯仰角补偿
    inline static double COMPANSATE_YAW;    // 偏航角补偿

    inline static const double SMALL_POWER_RUNE_ROTATION_SPEED{1.04719};  // 小能量机关旋转速度(弧度/秒)

    // 目标距离参数
    inline static const double MIN_DISTANCE_TO_TARGET{4};   // 与目标的最小距离(m)
    inline static const double MAX_DISTANCE_TO_TARGET{10};  // 与目标的最大距离(m)

    // 相机内参和畸变系数
    inline static cv::Mat INTRINSIC_MATRIX;  // 相机内参矩阵
    inline static cv::Mat DIST_COEFFS;       // 畸变系数

    inline static Color COLOR;  // 颜色模式(红色/蓝色)

    // 亮度阈值
    inline static int ARROW_BRIGHTNESS_THRESHOLD;  // 箭头亮度阈值
    inline static int ARMOR_BRIGHTNESS_THRESHOLD;  // 装甲板亮度阈值
    inline static const int MAX_BRIGHTNESS{255};   // 最大亮度值

    // ROI(感兴趣区域)参数
    inline static double LOCAL_ROI_DISTANCE_RATIO;  // 局部ROI距离比例
    inline static float LOCAL_ROI_WIDTH;            // 局部ROI宽度

    inline static float ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD;  // 装甲板中心垂直距离阈值

    inline static double GLOBAL_ROI_LENGTH_RATIO;  // 全局ROI长度比例

    // 箭头灯条参数
    inline static double MIN_ARROW_LIGHTLINE_AREA;  // 最小箭头灯条面积
    inline static double MAX_ARROW_LIGHTLINE_AREA;  // 最大箭头灯条面积

    inline static double MAX_ARROW_LIGHTLINE_ASPECT_RATIO;  // 最大箭头灯条长宽比

    inline static int MIN_ARROW_LIGHTLINE_NUM;  // 最小箭头灯条数量
    inline static int MAX_ARROW_LIGHTLINE_NUM;  // 最大箭头灯条数量

    // 箭头几何参数
    inline static double MIN_ARROW_ASPECT_RATIO;  // 最小箭头长宽比
    inline static double MAX_ARROW_ASPECT_RATIO;  // 最大箭头长宽比

    inline static double MAX_ARROW_AREA;  // 最大箭头面积

    inline static double MAX_SAME_ARROW_AREA_RATIO;  // 同一箭头的最大面积比

    // 装甲板灯条参数
    inline static double MIN_ARMOR_LIGHTLINE_AREA;  // 最小装甲板灯条面积
    inline static double MAX_ARMOR_LIGHTLINE_AREA;  // 最大装甲板灯条面积

    inline static double MIN_ARMOR_LIGHTLINE_CONTOUR_AREA;  // 最小装甲板灯条轮廓面积
    inline static double MAX_ARMOR_LIGHTLINE_CONTOUR_AREA;  // 最大装甲板灯条轮廓面积

    inline static double MIN_ARMOR_LIGHTLINE_ASPECT_RATIO;  // 最小装甲板灯条长宽比
    inline static double MAX_ARMOR_LIGHTLINE_ASPECT_RATIO;  // 最大装甲板灯条长宽比

    inline static double MAX_SAME_ARMOR_AREA_RATIO;  // 同一装甲板的最大面积比

    // 同一装甲板距离参数
    inline static double MIN_SAME_ARMOR_DISTANCE;  // 同一装甲板的最小距离
    inline static double MAX_SAME_ARMOR_DISTANCE;  // 同一装甲板的最大距离

    // 中心R标识参数
    inline static double MIN_CENTER_AREA;  // 最小中心R面积
    inline static double MAX_CENTER_AREA;  // 最大中心R面积

    inline static double MAX_CENTER_ASPECT_RATIO;  // 最大中心R长宽比

    // 拟合数据参数
    inline static int MIN_FIT_DATA_SIZE;  // 最小拟合数据大小
    inline static int MAX_FIT_DATA_SIZE;  // 最大拟合数据大小
};

}  // namespace power_rune