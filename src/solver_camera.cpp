#include "solver.hpp"
#include "hik_camera.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>  // 新增：用于计时
#include <deque>  // 用于存储轨迹点

using namespace cv;
using namespace std;
using namespace std::chrono;  // 新增：用于计时

// 定义一组不同的轨迹颜色
const std::vector<cv::Scalar> TRAJECTORY_COLORS = {
    cv::Scalar(0, 255, 255),   // 黄色
    cv::Scalar(255, 0, 255),   // 洋红
    cv::Scalar(0, 255, 0),     // 绿色
    cv::Scalar(255, 128, 0),   // 橙色
    cv::Scalar(255, 255, 0),   // 青色
    cv::Scalar(128, 0, 255)    // 紫色
};

// 像素坐标 -> 相机坐标系射线
static Vec3d pixelToCameraRay(const Point2f &uv, const Mat &K)
{
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    Vec3d dir((uv.x - cx) / fx, (uv.y - cy) / fy, 1.0);
    return normalize(dir);
}

// 射线与平面求交
static bool intersectRayPlane(const Vec3d &origin, const Vec3d &dir,
                              const Plane &plane, Vec3d &intersection)
{
    double denom = plane.normal.dot(dir);
    if (fabs(denom) < 1e-6) return false;
    double t = plane.normal.dot(plane.point - origin) / denom;
    if (t < 0) return false;
    intersection = origin + t * dir;
    return true;
}

//================= 实时相机接口 =================//
void runTrajectorySolverCamera(const Config &cfg)
{
    HikCamera camera;
    if(!camera.openCamera())
    {
        std::cerr << "[ERROR] Failed to open HikVision camera" << std::endl;
        return;
    }

    cv::Mat frame, undistorted;
    int frameIdx = 0;
    
    // 用于控制日志输出频率
    auto lastLogTime = steady_clock::now();
    const auto logInterval = milliseconds(500);  // 0.5秒
    bool shouldLog = false;

    // 检测功能开关
    bool detectEnabled = true;  // 默认开启检测
    const string windowName = "Basketball Detection";
    cv::namedWindow(windowName);
    // 添加新的窗口用于显示处理过程
    const string binaryWindowName = "Binary Process";
    cv::namedWindow(binaryWindowName);

    // 存储轨迹点和对应的颜色索引
    struct TrajectorySegment {
        std::deque<cv::Point2f> points;
        size_t colorIndex;
    };
    std::vector<TrajectorySegment> trajectorySegments;
    trajectorySegments.push_back({std::deque<cv::Point2f>(), 0});
    const size_t maxTrajectoryPoints = 1000;

    // 添加面积阈值
    const double MIN_CONTOUR_AREA = 5000.0;  // 最小轮廓面积阈值
    double currentMaxArea = 0.0;  // 用于显示当前最大轮廓面积

    // 创建形态学操作的核
    cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    
    // 创建一个用于显示处理过程的图像
    cv::Mat processViz;

    while (true)
    {
        if(!camera.getFrame(frame))
        {
            std::cerr << "[WARN] Failed to get camera frame, retrying..." << std::endl;
            continue;
        }

        ++frameIdx;
        
        auto now = steady_clock::now();
        if (now - lastLogTime >= logInterval) {
            shouldLog = true;
            lastLogTime = now;
        } else {
            shouldLog = false;
        }

        cv::undistort(frame, undistorted, cfg.K, cfg.distCoeffs);

        // 显示检测状态
        string statusText = detectEnabled ? "Detection: ON [Space to toggle]" : "Detection: OFF [Space to toggle]";
        cv::putText(undistorted, statusText, 
                   cv::Point(15, undistorted.rows - 90),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                   detectEnabled ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

        // 显示清除轨迹的提示
        cv::putText(undistorted, "Press 'C' to clear trajectory", 
                   cv::Point(15, undistorted.rows - 120),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                   cv::Scalar(255, 255, 255), 2);

        // 篮球检测和处理（始终进行）
        cv::Mat hsv, mask, morphed;
        cv::cvtColor(undistorted, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cfg.hsvLow, cfg.hsvHigh, mask);

        // 形态学操作
        cv::erode(mask, morphed, morphKernel, cv::Point(-1,-1), 2);
        cv::dilate(morphed, morphed, morphKernel, cv::Point(-1,-1), 2);

        // 创建可视化图像
        const int vizWidth = mask.cols * 2;
        const int vizHeight = mask.rows;
        processViz = cv::Mat::zeros(vizHeight, vizWidth, CV_8UC3);

        // 转换掩码为彩色图像以便显示
        cv::Mat maskViz, morphedViz;
        cv::cvtColor(mask, maskViz, cv::COLOR_GRAY2BGR);
        cv::cvtColor(morphed, morphedViz, cv::COLOR_GRAY2BGR);

        // 在可视化图像中并排显示原始掩码和处理后的图像
        maskViz.copyTo(processViz(cv::Rect(0, 0, mask.cols, mask.rows)));
        morphedViz.copyTo(processViz(cv::Rect(mask.cols, 0, mask.cols, mask.rows)));

        // 添加标题
        cv::putText(processViz, "Original Mask", 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                   0.8, cv::Scalar(0,255,0), 2);
        cv::putText(processViz, "After Morphology", 
                   cv::Point(mask.cols + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                   0.8, cv::Scalar(0,255,0), 2);

        // 显示处理过程
        cv::imshow(binaryWindowName, processViz);

        // 轮廓检测（始终进行）
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(morphed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        double maxArea = 0; 
        int maxIdx = -1;
        
        // 计算最大面积的轮廓
        for (size_t i = 0; i < contours.size(); ++i)
        {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                maxIdx = (int)i;
            }
        }

        currentMaxArea = maxArea;  // 更新当前最大面积

        // 显示当前最大轮廓面积
        cv::putText(undistorted, 
                   cv::format("Max Contour Area: %.1f", currentMaxArea),
                   cv::Point(15, undistorted.rows - 150),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6,
                   currentMaxArea >= MIN_CONTOUR_AREA ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
                   2);

        // 显示面积阈值
        cv::putText(undistorted,
                   cv::format("Area Threshold: %.1f", MIN_CONTOUR_AREA),
                   cv::Point(15, undistorted.rows - 180),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6,
                   cv::Scalar(255, 255, 255),
                   2);

        if (detectEnabled && maxIdx >= 0 && maxArea >= MIN_CONTOUR_AREA)
        {
            // 计算篮球中心点
            cv::Moments m = cv::moments(contours[maxIdx]);
            cv::Point2f center2D(static_cast<float>(m.m10/m.m00), static_cast<float>(m.m01/m.m00));
            
            // 添加新的轨迹点到当前段
            auto& currentSegment = trajectorySegments.back();
            currentSegment.points.push_back(center2D);
            if (currentSegment.points.size() > maxTrajectoryPoints) {
                currentSegment.points.pop_front();
            }

            // 绘制所有轨迹段
            for (const auto& segment : trajectorySegments) {
                const cv::Scalar& color = TRAJECTORY_COLORS[segment.colorIndex];
                for (size_t i = 1; i < segment.points.size(); ++i) {
                    cv::line(undistorted, 
                            segment.points[i-1], 
                            segment.points[i], 
                            color, 
                            2);
                }
            }

            // 绘制当前点（红色）和轮廓
            cv::circle(undistorted, center2D, 5, cv::Scalar(0,0,255), -1);
            cv::drawContours(undistorted, contours, maxIdx, cv::Scalar(0,255,0), 2);

            // ArUco 检测和坐标计算
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cfg.arucoDictId);
            cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
            cv::aruco::detectMarkers(undistorted, dictionary, corners, markerIds, detectorParams);

            if (!markerIds.empty())
            {
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, cfg.arucoMarkerLength, cfg.K, cfg.distCoeffs, rvecs, tvecs);
                cv::aruco::drawDetectedMarkers(undistorted, corners, markerIds);
                for (size_t i = 0; i < markerIds.size(); ++i)
                {
                    cv::aruco::drawAxis(undistorted, cfg.K, cfg.distCoeffs, rvecs[i], tvecs[i], cfg.arucoMarkerLength);
                    cv::putText(undistorted, cv::format("ID:%d", markerIds[i]), corners[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255), 2);
                }

                // 显示第一个ArUco标记相对于相机的信息
                cv::Vec3d tvec_cam = tvecs[0];
                double dist_cam = cv::norm(tvec_cam);
                cv::String cam_info_pos = cv::format("Marker Pos @Cam: (%.2f, %.2f, %.2f)m", tvec_cam[0], tvec_cam[1], tvec_cam[2]);
                cv::String cam_info_dist = cv::format("Marker Dist @Cam: %.2f m", dist_cam);
                cv::putText(undistorted, cam_info_pos, cv::Point(15, undistorted.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
                cv::putText(undistorted, cam_info_dist, cv::Point(15, undistorted.rows - 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

                cv::Mat Rmat;
                cv::Rodrigues(rvecs[0], Rmat);
                cv::Matx33d R(Rmat);
                cv::Vec3d rayWorld = R.t()*pixelToCameraRay(center2D, cfg.K);
                cv::Vec3d tvecVec(tvecs[0][0], tvecs[0][1], tvecs[0][2]);
                cv::Vec3d camPosWorld = -(R.t()*tvecVec);

                cv::Vec3d intersection;
                if(intersectRayPlane(camPosWorld, rayWorld, cfg.motionPlane, intersection))
                {
                    double height = cfg.H_marker + intersection[0];
                    if (shouldLog) {
                        std::cout << "[LOG] World coordinates = (" << intersection[0] << ", " << intersection[1] << ", " << intersection[2] << ") Height = " << height << " m" << std::endl;
                    }
                    cv::putText(undistorted, cv::format("Pos: (%.2f,%.2f,%.2f)m", intersection[0], intersection[1], intersection[2]),
                                cv::Point(15,30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,0),2);
                    cv::putText(undistorted, cv::format("Height: %.2f m", height), cv::Point(15,60), cv::FONT_HERSHEY_SIMPLEX,0.8, cv::Scalar(255,0,0),2);
                }
            }
        }

        // 显示图像和检查按键
        cv::imshow(windowName, undistorted);
        int key = cv::waitKey(1);
        if (key == 27) {
            cv::destroyWindow(binaryWindowName);
            break;        // ESC: exit program
        }
        if (key == 32) {
            detectEnabled = !detectEnabled;  // Space: toggle detection only
            if (detectEnabled) {
                size_t newColorIndex = (trajectorySegments.back().colorIndex + 1) % TRAJECTORY_COLORS.size();
                trajectorySegments.push_back({std::deque<cv::Point2f>(), newColorIndex});
            }
        }
        if (key == 'c' || key == 'C') {
            trajectorySegments.clear();
            trajectorySegments.push_back({std::deque<cv::Point2f>(), 0});
            if (shouldLog) {
                std::cout << "[INFO] Trajectory cleared" << std::endl;
            }
        }
    }

    camera.closeCamera();
} 