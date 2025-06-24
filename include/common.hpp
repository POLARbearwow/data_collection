#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <string>

struct Plane {
    cv::Vec3d point{0.0, 0.0, 0.0};      // 平面上一点 (世界坐标系)
    cv::Vec3d normal{0.0, 1.0, 0.0};     // 平面法向量 (需归一化)
};

struct Config {
    // 相机参数
    cv::Mat K;              // 内参矩阵 3x3
    cv::Mat distCoeffs;     // 畸变系数

    // ArUco 参数
    int   arucoDictId   = cv::aruco::DICT_6X6_250;
    float arucoMarkerLength = 0.05f;  // 单位: m
    double H_marker = 0.0;            // ArUco 中心点离地高度 (m)

    // HSV 阈值
    cv::Scalar hsvLow{0, 167, 37};
    cv::Scalar hsvHigh{19, 240, 147};

    // 运动平面定义
    Plane motionPlane;
};

// 从 YAML 文件加载所有配置参数
// 成功返回 true，失败返回 false
bool loadConfig(const std::string &filePath, Config &cfg); 