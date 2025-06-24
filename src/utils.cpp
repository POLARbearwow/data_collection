#include "common.hpp"
#include <opencv2/core.hpp>
#include <iostream>
#include <vector>

bool loadConfig(const std::string &filePath, Config &cfg)
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "无法打开配置文件: " << filePath << std::endl;
        return false;
    }

    fs["camera_matrix"] >> cfg.K;
    fs["dist_coeffs"] >> cfg.distCoeffs;
    fs["aruco_dict_id"] >> cfg.arucoDictId;
    fs["aruco_marker_length"] >> cfg.arucoMarkerLength;
    fs["H_marker"] >> cfg.H_marker;

    cv::FileNode hsv = fs["hsv_range"];
    if (!hsv.empty())
    {
        hsv["low"] >> cfg.hsvLow;
        hsv["high"] >> cfg.hsvHigh;
    }

    cv::FileNode plane = fs["motion_plane"];
    if (!plane.empty())
    {
        std::vector<double> p(3), n(3);
        plane["point"] >> p;
        plane["normal"] >> n;
        cfg.motionPlane.point = cv::Vec3d(p[0], p[1], p[2]);
        cfg.motionPlane.normal = cv::normalize(cv::Vec3d(n[0], n[1], n[2]));
    }
    return true;
} 