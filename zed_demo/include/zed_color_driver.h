#ifndef ZED_COLOR_DRIVER_H
#define ZED_COLOR_DRIVER_H

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

using namespace sl;

/**
 * @brief 图像质量统计结构体
 */
struct ColorImageStats {
    int total_frames = 0;           // 总帧数
    int valid_frames = 0;           // 有效帧数
    int corrupted_frames = 0;       // 损坏帧数
    int low_quality_frames = 0;     // 低质量帧数
    int dropped_frames = 0;         // 丢弃的帧数
    
    double get_valid_frame_rate() const {
        return total_frames > 0 ? (double)valid_frames / total_frames : 0.0;
    }
    
    double get_drop_rate() const {
        return total_frames > 0 ? (double)dropped_frames / total_frames : 0.0;
    }
};

/**
 * @brief 颜色图像质量过滤配置
 */
struct ColorFilterConfig {
    bool enable_corruption_filter = true;      // 启用损坏帧过滤
    bool enable_low_quality_filter = true;     // 启用低质量帧过滤
    int max_consecutive_errors = 10;           // 最大连续错误数
    int quality_check_level = 2;               // 质量检测级别 (1-3)
};

/**
 * @brief ZED颜色图像驱动类
 * 
 * 专门用于颜色图像捕获的简化驱动，包含：
 * - 自动初始化和配置ZED相机
 * - 实时图像质量检测和过滤
 * - 异常帧自动丢弃
 * - 相机健康状态监控
 */
class ZEDColorDriver {
public:
    /**
     * @brief 构造函数
     */
    ZEDColorDriver();
    
    /**
     * @brief 析构函数
     */
    ~ZEDColorDriver();
    
    /**
     * @brief 初始化相机（仅颜色模式）
     * @param resolution 相机分辨率
     * @param fps 目标帧率
     * @param config 质量过滤配置
     * @return 初始化是否成功
     */
    bool initialize(RESOLUTION resolution = RESOLUTION::HD1080,
                   int fps = 30,
                   const ColorFilterConfig& config = ColorFilterConfig());
    
    /**
     * @brief 抓取下一帧（自动质量检测和过滤）
     * @return 是否成功抓取到有效帧
     */
    bool grabValidFrame();
    
    /**
     * @brief 获取左侧彩色图像
     * @param image 输出图像
     * @return 是否成功获取
     */
    bool getLeftImage(Mat& image);
    
    /**
     * @brief 获取右侧彩色图像
     * @param image 输出图像
     * @return 是否成功获取
     */
    bool getRightImage(Mat& image);
    
    /**
     * @brief 获取双目图像（左右并排）
     * @param image 输出图像
     * @return 是否成功获取
     */
    bool getSideBySideImage(Mat& image);
    
    /**
     * @brief 获取相机信息
     * @return 相机信息结构体
     */
    CameraInformation getCameraInfo() const;
    
    /**
     * @brief 获取图像质量统计
     * @return 质量统计信息
     */
    ColorImageStats getQualityStats() const;
    
    /**
     * @brief 重置质量统计
     */
    void resetQualityStats();
    
    /**
     * @brief 设置质量过滤配置
     * @param config 新的配置
     */
    void setFilterConfig(const ColorFilterConfig& config);
    
    /**
     * @brief 获取当前质量过滤配置
     * @return 当前配置
     */
    ColorFilterConfig getFilterConfig() const;
    
    /**
     * @brief 检查相机是否已连接
     * @return 是否已连接
     */
    bool isConnected() const;
    
    /**
     * @brief 获取实际帧率
     * @return 当前帧率
     */
    double getCurrentFPS() const;
    
    /**
     * @brief 设置日志回调函数
     * @param callback 日志回调函数
     */
    void setLogCallback(std::function<void(const std::string&)> callback);
    
    /**
     * @brief 关闭相机
     */
    void close();

private:
    std::unique_ptr<Camera> camera_;            // ZED相机对象
    CameraInformation camera_info_;             // 相机信息
    InitParameters init_params_;                // 初始化参数
    RuntimeParameters runtime_params_;          // 运行时参数
    
    bool is_initialized_;                       // 是否已初始化
    ColorFilterConfig filter_config_;           // 质量过滤配置
    ColorImageStats quality_stats_;             // 质量统计
    
    int consecutive_error_count_;               // 连续错误计数
    std::chrono::high_resolution_clock::time_point start_time_;    // 开始时间
    
    std::function<void(const std::string&)> log_callback_;  // 日志回调
    
    /**
     * @brief 检查帧质量
     * @param grab_state 抓取状态
     * @return 帧是否有效
     */
    bool checkFrameQuality(ERROR_CODE grab_state);
    
    /**
     * @brief 检查相机健康状态
     * @return 健康状态是否正常
     */
    bool checkCameraHealth();
    
    /**
     * @brief 记录日志
     * @param message 日志消息
     */
    void logMessage(const std::string& message);
    
    /**
     * @brief 尝试恢复相机连接
     * @return 是否成功恢复
     */
    bool recoverCamera();
};

#endif // ZED_COLOR_DRIVER_H 