#include "solver.hpp"
#include "hik_camera.hpp"
#include "basketball_detector.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>  // 新增：用于计时
#include <deque>  // 用于存储轨迹点
#include <fstream>
#include <ctime>
#include <filesystem>

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
    
    // 记录相关
    std::ofstream recordFile;
    bool recording = false;
    int  sessionIndex = 0;  // 用于文件命名

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
    const double MIN_CONTOUR_AREA = 300.0;  // 最小轮廓面积阈值
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

        // 持续进行ArUco检测
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cfg.arucoDictId);
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(undistorted, dictionary, corners, markerIds, detectorParams);

        bool hasValidAruco = false;
        std::vector<cv::Vec3d> rvecs, tvecs;
        
        if (!markerIds.empty())
        {
            hasValidAruco = true;
            cv::aruco::estimatePoseSingleMarkers(corners, cfg.arucoMarkerLength, cfg.K, cfg.distCoeffs, rvecs, tvecs);
            
            // 绘制检测到的ArUco标记
            cv::aruco::drawDetectedMarkers(undistorted, corners, markerIds);
            
            for (size_t i = 0; i < markerIds.size(); ++i)
            {
                // 绘制坐标轴
                cv::aruco::drawAxis(undistorted, cfg.K, cfg.distCoeffs, rvecs[i], tvecs[i], cfg.arucoMarkerLength);
                
                // 在标记上显示ID
                cv::putText(undistorted, 
                          cv::format("ArUco ID: %d", markerIds[i]), 
                          corners[i][0], 
                          cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                          cv::Scalar(0,255,255), 2);

                // 显示ArUco标记的3D位置
                cv::Vec3d tvec = tvecs[i];
                cv::putText(undistorted,
                          cv::format("ArUco %d Pos: (%.2f, %.2f, %.2f)m", 
                                   markerIds[i], tvec[0], tvec[1], tvec[2]),
                          cv::Point(15, 30 + i * 30),
                          cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cv::Scalar(255,255,0), 2);
            }
        }

        // 篮球检测
        bool hasValidBall = false;
        cv::Point2f center2D;
        
        BasketballDetector detector;
        detector.setHSVRange(cfg.hsvLow, cfg.hsvHigh);
        auto ballResult = detector.detect(undistorted);
        
        // 创建可视化图像
        const int vizWidth = ballResult.mask.cols * 2;
        const int vizHeight = ballResult.mask.rows;
        processViz = cv::Mat::zeros(vizHeight, vizWidth, CV_8UC3);

        // 转换掩码为彩色图像以便显示
        cv::Mat maskViz, morphedViz;
        cv::cvtColor(ballResult.mask, maskViz, cv::COLOR_GRAY2BGR);
        
        // 如果检测到篮球，显示检测结果
        if (ballResult.found) {
            cv::cvtColor(ballResult.mask, morphedViz, cv::COLOR_GRAY2BGR);
            // 在掩码上绘制检测结果
            for (const auto &p : ballResult.inliers) {
                cv::circle(morphedViz, p, 2, cv::Scalar(255,0,255), -1);
            }
            cv::circle(morphedViz, ballResult.center, (int)ballResult.radius, cv::Scalar(0,255,0), 2);
        } else {
            cv::cvtColor(ballResult.mask, morphedViz, cv::COLOR_GRAY2BGR);
        }

        // 在可视化图像中并排显示原始掩码和处理后的图像
        maskViz.copyTo(processViz(cv::Rect(0, 0, ballResult.mask.cols, ballResult.mask.rows)));
        morphedViz.copyTo(processViz(cv::Rect(ballResult.mask.cols, 0, ballResult.mask.cols, ballResult.mask.rows)));

        // 添加标题
        cv::putText(processViz, "Original Mask", 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                   0.8, cv::Scalar(0,255,0), 2);
        cv::putText(processViz, "Detection Result", 
                   cv::Point(ballResult.mask.cols + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                   0.8, cv::Scalar(0,255,0), 2);

        // 显示处理过程
        cv::imshow(binaryWindowName, processViz);

        if (detectEnabled && ballResult.found)
        {
            hasValidBall = true;
            center2D = ballResult.center;
            
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

            // 绘制当前篮球位置
            cv::circle(undistorted, center2D, 5, cv::Scalar(0,0,255), -1);
            
            // 显示篮球像素坐标
            cv::putText(undistorted, 
                       cv::format("Ball Pixel Pos: (%.1f, %.1f)", center2D.x, center2D.y),
                       cv::Point(15, undistorted.rows - 150),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255), 2);
        }

        // 当同时检测到篮球和ArUco标记时，进行坐标解算
        if (hasValidBall && hasValidAruco)
        {
            // 计算并显示世界坐标
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
                    std::cout << "[LOG] World coordinates = (" 
                             << intersection[0] << ", " 
                             << intersection[1] << ", " 
                             << intersection[2] << ") Height = " 
                             << height << " m" << std::endl;
                }
                
                // 在图像上标注坐标解算结果（使用 ArUco 坐标系）
                cv::putText(undistorted, 
                          cv::format("Ball ArUco Pos: (%.2f, %.2f, %.2f)m", 
                                   intersection[0], intersection[1], intersection[2]),
                          cv::Point(15, undistorted.rows - 300),
                          cv::FONT_HERSHEY_SIMPLEX, 1.2,
                          cv::Scalar(255,0,0), 2);
                cv::putText(undistorted, 
                          cv::format("Ball Height H: %.2f m", height),
                          cv::Point(15, undistorted.rows - 270),
                          cv::FONT_HERSHEY_SIMPLEX, 1.2,
                          cv::Scalar(255,0,0), 2);

                // 添加状态指示
                cv::putText(undistorted,
                          "Status: Ball & ArUco detected - Computing coordinates",
                          cv::Point(15, undistorted.rows - 240),
                          cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cv::Scalar(0,255,0), 2);

                // ---- 数据记录 ----
                if (recording && recordFile.is_open()) {
                    // 获取时间戳 (ms)
                    auto now_ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();

                    // 计算新坐标系下的坐标 (减去偏移量)
                    cv::Vec3d newPos = intersection - cfg.originOffset;

                    recordFile << now_ts << ","
                               << intersection[0] << "," << intersection[1] << "," << intersection[2] << ","
                               << newPos[0]       << "," << newPos[1]       << "," << newPos[2] << ","
                               << height << ","
                               << cfg.launchRPM << std::endl;
                }
            }
        }
        else
        {
            // 显示当前检测状态
            std::string status = "Status: ";
            if (!hasValidBall && !hasValidAruco) status += "No Ball & No ArUco detected";
            else if (!hasValidBall) status += "No Ball detected";
            else if (!hasValidAruco) status += "No ArUco detected";
            
            cv::putText(undistorted,
                      status,
                      cv::Point(15, undistorted.rows - 240),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      cv::Scalar(0,0,255), 2);
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

            // 切换检测状态时，处理记录文件开关
            if (cfg.recordEnabled)
            {
                if (detectEnabled) {
                    // 开始新的记录会话
                    if (recordFile.is_open()) recordFile.close();

                    namespace fs = std::filesystem;
                    try {
                        fs::create_directories(cfg.recordDir);
                    } catch (const std::exception &e) {
                        std::cerr << "[ERROR] Failed to create record directory: " << e.what() << std::endl;
                    }

                    char filename[128];
                    std::time_t t = std::time(nullptr);
                    std::tm *tm_ptr = std::localtime(&t);
                    std::strftime(filename, sizeof(filename), "record_%Y%m%d_%H%M%S.csv", tm_ptr);
                    std::string filepath = cfg.recordDir + "/" + filename;

                    recordFile.open(filepath);
                    if (recordFile.is_open()) {
                        recording = true;
                        std::cout << "[INFO] Start recording to " << filepath << std::endl;
                        // 写入表头
                        recordFile << "timestamp_ms,x,y,z,new_x,new_y,new_z,height_m,rpm" << std::endl;
                    } else {
                        std::cerr << "[ERROR] Failed to open record file: " << filepath << std::endl;
                    }
                    ++sessionIndex;
                }
                else {
                    if (recordFile.is_open()) {
                        std::cout << "[INFO] Stop recording, file closed." << std::endl;
                        recordFile.close();
                    }
                    recording = false;
                }
            }

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

    if (recordFile.is_open()) {
        std::cout << "[INFO] Stop recording, file closed." << std::endl;
        recordFile.close();
    }
} 