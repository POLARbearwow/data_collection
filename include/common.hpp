#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <string>

struct Plane {
    cv::Vec3d point{0.0, 0.0, 0.0};      // 平面上一点 (世界坐标系)
    cv::Vec3d normal{0.0, 1.0, 0.0};     // 平面法向量 (需归一化)
};

// 相机类型枚举
enum class CameraType {
    HIK,    // 海康相机
    ZED     // ZED相机
};

struct Config {
    // 相机参数
    cv::Mat K;              // 内参矩阵 3x3
    cv::Mat distCoeffs;     // 畸变系数
    CameraType cameraType = CameraType::HIK;  // 相机类型

    // ArUco 参数
    int   arucoDictId   = cv::aruco::DICT_4X4_50;
    float arucoMarkerLength = 0.05f;  // 单位: m
    double H_marker = 0.0;            // ArUco 中心点离地高度 (m)

    // 数据记录相关
    bool  recordEnabled = false;      // 是否启用记录功能
    cv::Vec3d originOffset{0.0, 0.0, 0.0}; // 新坐标系原点相对 ArUco 坐标系的偏移
    double launchRPM = 0.0;           // 发射篮球时的转速 (RPM)

    // 记录文件保存目录（可选，默认当前目录）
    std::string recordDir{"."};

    // 轨迹绘制：若连续两帧篮球中心距离超过该值，将拆分轨迹 (像素)
    double maxBallGap = 120.0;

    // HSV 阈值
    cv::Scalar hsvLow{0, 167, 37};
    cv::Scalar hsvHigh{19, 240, 147};

    // 运动平面定义
    Plane motionPlane;

    // ROI 边缘比例 (0~0.5)。可分别设置左右 / 上下。
    double roiLeftMarginRatio   = 0.1;   // 默认左侧去除 10%
    double roiRightMarginRatio  = 0.1;   // 默认右侧去除 10%
    double roiTopMarginRatio    = 0.0;   // 默认不去除顶部
    double roiBottomMarginRatio = 0.0;   // 默认不去除底部

    // ROI 视频录制保存目录
    std::string roiVideoDir{"."};
};

// 从 YAML 文件加载所有配置参数
// 成功返回 true，失败返回 false
bool loadConfig(const std::string &filePath, Config &cfg); 