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

    // 尝试读取旧格式的运动平面参数
    cv::Mat planePoint, planeNormal;
    fs["motion_plane_point"] >> planePoint;
    fs["motion_plane_normal"] >> planeNormal;
    
    if (!planePoint.empty() && !planeNormal.empty())
    {
        // 使用旧格式的参数
        cfg.motionPlane.point = cv::Vec3d(planePoint.at<double>(0), planePoint.at<double>(1), planePoint.at<double>(2));
        cfg.motionPlane.normal = cv::normalize(cv::Vec3d(planeNormal.at<double>(0), planeNormal.at<double>(1), planeNormal.at<double>(2)));
    }
    else
    {
        // 尝试读取新格式的运动平面参数
        cv::FileNode plane = fs["motion_plane"];
        if (!plane.empty())
        {
            cv::Mat p, n;
            plane["point"] >> p;
            plane["normal"] >> n;
            
            if (!p.empty() && !n.empty())
            {
                cfg.motionPlane.point = cv::Vec3d(p.at<double>(0), p.at<double>(1), p.at<double>(2));
                cv::Vec3d normal(n.at<double>(0), n.at<double>(1), n.at<double>(2));
                double norm = cv::norm(normal);
                if (norm < 1e-6)
                {
                    std::cerr << "警告: 平面法向量接近零向量，使用默认值 [0,0,1]" << std::endl;
                    cfg.motionPlane.normal = cv::Vec3d(0, 0, 1);
                }
                else
                {
                    cfg.motionPlane.normal = normal / norm;
                }
            }
            else
            {
                std::cerr << "警告: motion_plane 参数不完整，使用默认值" << std::endl;
            }
        }
        else
        {
            std::cerr << "警告: 未找到运动平面参数，使用默认值" << std::endl;
        }
    }

    fs.release();
    return true;
} 