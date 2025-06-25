#include "basketball_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <random>
#include <chrono>

// 定义M_PI（如果未定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 构造函数 (使用默认参数)
BasketballDetector::BasketballDetector() {
    initKalmanFilter();
}

void BasketballDetector::setHSVRange(const cv::Scalar &low, const cv::Scalar &high)
{
    hsvLow_  = low;
    hsvHigh_ = high;
}

void BasketballDetector::getHSVRange(cv::Scalar &low, cv::Scalar &high) const
{
    low = hsvLow_;
    high = hsvHigh_;
}

void BasketballDetector::setMorphParams(int kernelSize, int closeIterations)
{
    if (kernelSize % 2 == 0) ++kernelSize;          // 保证为奇数
    kernelSize = std::max(3, kernelSize);
    morphKernelSize_ = kernelSize;
    morphIterations_ = std::max(1, closeIterations);
}

void BasketballDetector::setMedianKernel(int kernelSize)
{
    if (kernelSize <= 0)
    {
        medianKernelSize_ = 0;
        return;
    }
    if (kernelSize % 2 == 0) ++kernelSize;
    medianKernelSize_ = kernelSize;
}

void BasketballDetector::setRansacParams(int minInliers, float maxJumpPixels, int maxLostFrames)
{
    minInliers_ = std::max(1, minInliers);
    maxJumpPixels_ = std::max(1.0f, maxJumpPixels);
    maxLostFrames_ = std::max(1, maxLostFrames);
}

void BasketballDetector::resetTracking()
{
    hasKalman_ = false;
    lostFrameCount_ = 0;
}

void BasketballDetector::initKalmanFilter()
{
    // 状态向量: [x, y, radius, vx, vy, vr] (位置 + 速度)
    int stateSize = 6;
    // 测量向量: [x, y, radius] (只测量位置)
    int measurementSize = 3;
    // 控制向量: 0 (无控制输入)
    int controlSize = 0;
    
    kalmanFilter_ = cv::KalmanFilter(stateSize, measurementSize, controlSize);
    
    // 状态转移矩阵 A
    kalmanFilter_.transitionMatrix = cv::Mat::eye(stateSize, stateSize, CV_32F);
    kalmanFilter_.transitionMatrix.at<float>(0, 3) = 1.0f; // x += vx
    kalmanFilter_.transitionMatrix.at<float>(1, 4) = 1.0f; // y += vy
    kalmanFilter_.transitionMatrix.at<float>(2, 5) = 1.0f; // radius += vr
    
    // 测量矩阵 H
    kalmanFilter_.measurementMatrix = cv::Mat::zeros(measurementSize, stateSize, CV_32F);
    kalmanFilter_.measurementMatrix.at<float>(0, 0) = 1.0f; // 测量x
    kalmanFilter_.measurementMatrix.at<float>(1, 1) = 1.0f; // 测量y
    kalmanFilter_.measurementMatrix.at<float>(2, 2) = 1.0f; // 测量radius
    
    // 过程噪声协方差矩阵 Q
    kalmanFilter_.processNoiseCov = cv::Mat::eye(stateSize, stateSize, CV_32F) * 1e-3f;
    kalmanFilter_.processNoiseCov.at<float>(3, 3) = 1e-2f; // 速度噪声
    kalmanFilter_.processNoiseCov.at<float>(4, 4) = 1e-2f;
    kalmanFilter_.processNoiseCov.at<float>(5, 5) = 1e-3f;
    
    // 测量噪声协方差矩阵 R
    kalmanFilter_.measurementNoiseCov = cv::Mat::eye(measurementSize, measurementSize, CV_32F) * 1e-1f;
    
    // 后验误差协方差矩阵 P
    kalmanFilter_.errorCovPost = cv::Mat::eye(stateSize, stateSize, CV_32F) * 1.0f;
    
    // 初始状态
    kalmanFilter_.statePost = cv::Mat::zeros(stateSize, 1, CV_32F);
}

void BasketballDetector::resetKalmanFilter(const cv::Point2f& center, float radius)
{
    kalmanFilter_.statePost.at<float>(0) = center.x;
    kalmanFilter_.statePost.at<float>(1) = center.y;
    kalmanFilter_.statePost.at<float>(2) = radius;
    kalmanFilter_.statePost.at<float>(3) = 0.0f; // vx
    kalmanFilter_.statePost.at<float>(4) = 0.0f; // vy
    kalmanFilter_.statePost.at<float>(5) = 0.0f; // vr
    
    hasKalman_ = true;
    lostFrameCount_ = 0;
}

void BasketballDetector::updateKalmanFilter(const cv::Point2f& center, float radius)
{
    if (!hasKalman_)
    {
        // 首次检测到目标，初始化Kalman滤波
        resetKalmanFilter(center, radius);
        return;
    }
    
    // 检查跳变
    cv::Point2f currentCenter;
    currentCenter.x = kalmanFilter_.statePost.at<float>(0);
    currentCenter.y = kalmanFilter_.statePost.at<float>(1);
    
    float jump = std::hypot(center.x - currentCenter.x, center.y - currentCenter.y);
    if (jump < maxJumpPixels_)  // 合理的跳变范围内才更新
    {
        // 准备测量向量
        cv::Mat measurement = cv::Mat::zeros(3, 1, CV_32F);
        measurement.at<float>(0) = center.x;
        measurement.at<float>(1) = center.y;
        measurement.at<float>(2) = radius;
        
        // Kalman预测
        cv::Mat prediction = kalmanFilter_.predict();
        
        // Kalman更新
        cv::Mat corrected = kalmanFilter_.correct(measurement);
        
        lostFrameCount_ = 0; // 成功更新滤波，重置丢失计数
    }
    else
    {
        // 跳变过大，只进行预测，不更新
        kalmanFilter_.predict();
        lostFrameCount_++;
    }
    
    // 检查是否需要重置滤波
    if (lostFrameCount_ > maxLostFrames_)
    {
        hasKalman_ = false;
        lostFrameCount_ = 0;
    }
}

BasketballDetector::Result BasketballDetector::detect(const cv::Mat &frameBGR)
{
    return detect(frameBGR, true); // 默认启用滤波
}

BasketballDetector::Result BasketballDetector::detect(const cv::Mat &frameBGR, bool enableFiltering)
{
    Result out;
    if (frameBGR.empty())
        return out;

    // 1. BGR -> HSV
    cv::Mat hsv;
    cv::cvtColor(frameBGR, hsv, cv::COLOR_BGR2HSV);

    // 2. HSV threshold
    cv::Mat mask;
    cv::inRange(hsv, hsvLow_, hsvHigh_, mask);

    // 如果不需要滤波，直接返回HSV掩码结果
    if (!enableFiltering) {
        out.mask = mask;
        return out;
    }

    // 3. Morphological closing
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                              cv::Size(morphKernelSize_, morphKernelSize_));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), morphIterations_);

    // 4. Optional median filter (to reduce noise)
    if (medianKernelSize_ > 0 && medianKernelSize_ >= 3)
    {
        cv::medianBlur(mask, mask, medianKernelSize_);
    }

    // 5. Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double maxArea = 0.0;
    int    maxIdx  = -1;
    for (size_t i = 0; i < contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            maxIdx  = static_cast<int>(i);
        }
    }

    if (maxIdx >= 0 && contours[maxIdx].size() >= 6)
    {
        // -------- RANSAC Circle Fitting -------- //
        const std::vector<cv::Point> &pts = contours[maxIdx];
        const int ITERATIONS = 200;
        const float RANSAC_THRESH_RATIO = 0.09f; // 允许误差比例 (5%)
        const float MAX_RADIUS = static_cast<float>(frameBGR.cols) * 0.28f;

        int bestInlierCnt = 0;
        cv::Point2f bestCenter;
        float bestRadius = 0.f;
        std::vector<cv::Point> bestInliers;

        std::mt19937 rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_int_distribution<int> uni(0, (int)pts.size() - 1);

        auto calcCircle = [](const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &p3, cv::Point2f &c, float &r)->bool{
            // 计算三点（不共线）外接圆
            float a = p2.x - p1.x;
            float b = p2.y - p1.y;
            float c1 = p3.x - p1.x;
            float d = p3.y - p1.y;
            float e = a * (p1.x + p2.x) + b * (p1.y + p2.y);
            float f = c1 * (p1.x + p3.x) + d * (p1.y + p3.y);
            float g = 2.0f * (a * (p3.y - p2.y) - b * (p3.x - p2.x));
            if (fabs(g) < 1e-6) return false; // 共线或非常接近共线
            c.x = (d * e - b * f) / g;
            c.y = (a * f - c1 * e) / g;
            r = std::sqrt((c.x - p1.x) * (c.x - p1.x) + (c.y - p1.y) * (c.y - p1.y));
            return true;
        };

        for (int iter = 0; iter < ITERATIONS; ++iter)
        {
            // 随机取 3 个点
            int idx1 = uni(rng);
            int idx2 = uni(rng);
            int idx3 = uni(rng);
            if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) { --iter; continue; }

            cv::Point2f c;
            float r;
            if (!calcCircle(pts[idx1], pts[idx2], pts[idx3], c, r)) {
                continue;
            }

            // 半径过大（超过图片宽度一半）则跳过
            if (r > MAX_RADIUS) {
                continue;
            }

            // 统计内点
            float thresh = RANSAC_THRESH_RATIO * r;
            std::vector<cv::Point> inliers;
            for (const auto &p : pts)
            {
                float dist = std::hypot(p.x - c.x, p.y - c.y);
                if (fabs(dist - r) < thresh)
                    inliers.push_back(p);
            }

            // 验证内点数是否超过圆周的1/5 且 至少达到最小内点数要求
            float circumference = 2.0f * M_PI * r;
            float minInliers = circumference / 6.0f;
            bool validCircle = inliers.size() >= minInliers && inliers.size() >= minInliers_;

            if (validCircle && (int)inliers.size() > bestInlierCnt)
            {
                bestInlierCnt = (int)inliers.size();
                bestCenter = c;
                bestRadius = r;
                bestInliers.swap(inliers);
            }
        }

        // 如果找到足够多的内点，则认为检测成功
        if (bestInlierCnt >= minInliers_)
        {
            out.found  = true;
            out.center = bestCenter;
            out.radius = bestRadius;
            out.inliers = bestInliers;
            
            // 更新Kalman滤波器
            updateKalmanFilter(bestCenter, bestRadius);
        }
    }

    // 设置平滑结果
    if (hasKalman_)
    {
        out.smoothedCenter.x = kalmanFilter_.statePost.at<float>(0);
        out.smoothedCenter.y = kalmanFilter_.statePost.at<float>(1);
        out.smoothedRadius = std::max(1.0f, std::min(1000.0f, kalmanFilter_.statePost.at<float>(2)));
        out.hasTracking = true;
    }
    else
    {
        // 如果没有跟踪，使用原始检测结果
        out.smoothedCenter = out.center;
        out.smoothedRadius = out.radius;
        out.hasTracking = false;
    }

    out.mask = mask; // copy (shallow)
    return out;
} 