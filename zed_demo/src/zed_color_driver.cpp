#include "zed_color_driver.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <iomanip>

ZEDColorDriver::ZEDColorDriver() 
    : camera_(std::make_unique<Camera>())
    , is_initialized_(false)
    , consecutive_error_count_(0)
    , start_time_(std::chrono::high_resolution_clock::now())
{
}

ZEDColorDriver::~ZEDColorDriver() {
    if (is_initialized_) {
        close();
    }
}

bool ZEDColorDriver::initialize(RESOLUTION resolution, int fps, const ColorFilterConfig& config) {
    if (is_initialized_) {
        logMessage("警告: 相机已经初始化!");
        return true;
    }
    
    filter_config_ = config;
    
    // 配置初始化参数 - 仅颜色模式
    init_params_.camera_resolution = resolution;
    init_params_.camera_fps = fps;
    init_params_.depth_mode = DEPTH_MODE::NONE;  // 禁用深度计算
    init_params_.coordinate_units = UNIT::METER;
    init_params_.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_params_.sensors_required = false;       // 不需要传感器
    
    // 启用图像质量检测
    init_params_.enable_image_validity_check = filter_config_.quality_check_level;
    
    // 启用异步抓取恢复机制
    init_params_.async_grab_camera_recovery = true;
    
    // 启用图像增强
    init_params_.enable_image_enhancement = true;
    
    // 禁用不需要的功能以提高性能
    init_params_.enable_right_side_measure = false;
    init_params_.depth_stabilization = 0;  // 不需要深度稳定性
    
    logMessage("正在初始化ZED相机（仅颜色模式）...");
    logMessage("分辨率: " + std::to_string(static_cast<int>(resolution)));
    logMessage("帧率: " + std::to_string(fps));
    logMessage("图像质量检测级别: " + std::to_string(filter_config_.quality_check_level));
    
    // 尝试打开相机
    ERROR_CODE returned_state = camera_->open(init_params_);
    if (returned_state != ERROR_CODE::SUCCESS) {
        logMessage("错误: 无法打开相机 - " + std::string(toString(returned_state)));
        return false;
    }
    
    // 获取相机信息
    camera_info_ = camera_->getCameraInformation();
    is_initialized_ = true;
    
    // 配置运行时参数 - 禁用深度相关功能
    runtime_params_.enable_depth = false;       // 禁用深度计算
    runtime_params_.enable_fill_mode = false;   // 禁用深度填充
    
    // 重置统计信息
    resetQualityStats();
    start_time_ = std::chrono::high_resolution_clock::now();
    
    logMessage("ZED相机初始化成功（仅颜色模式）!");
    logMessage("型号: " + std::string(toString(camera_info_.camera_model)));
    logMessage("序列号: " + std::to_string(camera_info_.serial_number));
    logMessage("固件版本: " + std::to_string(camera_info_.camera_configuration.firmware_version));
    logMessage("实际分辨率: " + std::to_string(camera_info_.camera_configuration.resolution.width) + 
               "x" + std::to_string(camera_info_.camera_configuration.resolution.height));
    logMessage("实际帧率: " + std::to_string(camera_info_.camera_configuration.fps));
    
    return true;
}

bool ZEDColorDriver::grabValidFrame() {
    if (!is_initialized_) {
        logMessage("错误: 相机未初始化!");
        return false;
    }
    
    const int max_retries = 5;  // 最大重试次数
    int retry_count = 0;
    
    while (retry_count < max_retries) {
        ERROR_CODE grab_state = camera_->grab(runtime_params_);
        quality_stats_.total_frames++;
        
        // 检查帧质量
        if (checkFrameQuality(grab_state)) {
            quality_stats_.valid_frames++;
            consecutive_error_count_ = 0;  // 重置连续错误计数
            return true;
        }
        
        // 处理错误情况
        consecutive_error_count_++;
        retry_count++;
        
        // 如果连续错误过多，尝试恢复相机
        if (consecutive_error_count_ >= filter_config_.max_consecutive_errors) {
            logMessage("警告: 连续错误次数过多，尝试恢复相机连接...");
            if (recoverCamera()) {
                consecutive_error_count_ = 0;
            } else {
                logMessage("错误: 相机恢复失败!");
                return false;
            }
        }
        
        // 短暂延迟后重试
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    logMessage("警告: 达到最大重试次数，无法获取有效帧");
    return false;
}

bool ZEDColorDriver::getLeftImage(Mat& image) {
    if (!is_initialized_) {
        logMessage("错误: 相机未初始化!");
        return false;
    }
    
    ERROR_CODE returned_state = camera_->retrieveImage(image, VIEW::LEFT);
    if (returned_state != ERROR_CODE::SUCCESS) {
        logMessage("错误: 无法获取左侧图像 - " + std::string(toString(returned_state)));
        return false;
    }
    
    return true;
}

bool ZEDColorDriver::getRightImage(Mat& image) {
    if (!is_initialized_) {
        logMessage("错误: 相机未初始化!");
        return false;
    }
    
    ERROR_CODE returned_state = camera_->retrieveImage(image, VIEW::RIGHT);
    if (returned_state != ERROR_CODE::SUCCESS) {
        logMessage("错误: 无法获取右侧图像 - " + std::string(toString(returned_state)));
        return false;
    }
    
    return true;
}

bool ZEDColorDriver::getSideBySideImage(Mat& image) {
    if (!is_initialized_) {
        logMessage("错误: 相机未初始化!");
        return false;
    }
    
    ERROR_CODE returned_state = camera_->retrieveImage(image, VIEW::SIDE_BY_SIDE);
    if (returned_state != ERROR_CODE::SUCCESS) {
        logMessage("错误: 无法获取双目图像 - " + std::string(toString(returned_state)));
        return false;
    }
    
    return true;
}

CameraInformation ZEDColorDriver::getCameraInfo() const {
    return camera_info_;
}

ColorImageStats ZEDColorDriver::getQualityStats() const {
    return quality_stats_;
}

void ZEDColorDriver::resetQualityStats() {
    quality_stats_ = ColorImageStats();
    start_time_ = std::chrono::high_resolution_clock::now();
}

void ZEDColorDriver::setFilterConfig(const ColorFilterConfig& config) {
    filter_config_ = config;
    
    // 如果相机已初始化，提醒用户质量检测级别只能在初始化时设置
    if (is_initialized_) {
        logMessage("警告: 质量检测级别只能在初始化时设置");
    }
}

ColorFilterConfig ZEDColorDriver::getFilterConfig() const {
    return filter_config_;
}

bool ZEDColorDriver::isConnected() const {
    return is_initialized_ && camera_->isOpened();
}

double ZEDColorDriver::getCurrentFPS() const {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_);
    
    if (duration.count() > 0 && quality_stats_.valid_frames > 0) {
        return static_cast<double>(quality_stats_.valid_frames) / duration.count();
    }
    
    return 0.0;
}

void ZEDColorDriver::setLogCallback(std::function<void(const std::string&)> callback) {
    log_callback_ = callback;
}

void ZEDColorDriver::close() {
    if (is_initialized_) {
        camera_->close();
        is_initialized_ = false;
        logMessage("相机已关闭");
    }
}

bool ZEDColorDriver::checkFrameQuality(ERROR_CODE grab_state) {
    // 检查基本抓取状态
    if (grab_state == ERROR_CODE::SUCCESS) {
        return true;  // 基本成功
    }
    
    // 处理各种错误状态
    switch (grab_state) {
        case ERROR_CODE::CORRUPTED_FRAME:
            if (filter_config_.enable_corruption_filter) {
                quality_stats_.corrupted_frames++;
                quality_stats_.dropped_frames++;
                
                // 检查详细健康状态
                if (checkCameraHealth()) {
                    logMessage("检测到损坏帧，已丢弃");
                }
                return false;
            }
            break;
            
        case ERROR_CODE::CAMERA_NOT_DETECTED:
            logMessage("错误: 相机未检测到");
            return false;
            
        case ERROR_CODE::CAMERA_REBOOTING:
            logMessage("警告: 相机正在重启");
            return false;
            
        case ERROR_CODE::NO_GPU_COMPATIBLE:
            logMessage("错误: GPU不兼容");
            return false;
            
        default:
            logMessage("抓取错误: " + std::string(toString(grab_state)));
            return false;
    }
    
    return true;
}

bool ZEDColorDriver::checkCameraHealth() {
    if (!is_initialized_) {
        return false;
    }
    
    // 获取详细健康状态
    auto health = camera_->getHealthStatus();
    
    bool health_issues_found = false;
    
    // 检查图像质量状态
    if (health.low_image_quality && filter_config_.enable_low_quality_filter) {
        quality_stats_.low_quality_frames++;
        quality_stats_.dropped_frames++;
        logMessage("检测到低图像质量");
        health_issues_found = true;
    }
    
    return !health_issues_found;
}

void ZEDColorDriver::logMessage(const std::string& message) {
    // 添加时间戳
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
    ss << message;
    
    std::string timestamped_message = ss.str();
    
    // 如果有回调函数，使用回调；否则输出到控制台
    if (log_callback_) {
        log_callback_(timestamped_message);
    } else {
        std::cout << timestamped_message << std::endl;
    }
}

bool ZEDColorDriver::recoverCamera() {
    logMessage("尝试恢复相机连接...");
    
    // 先关闭当前连接
    if (is_initialized_) {
        camera_->close();
        is_initialized_ = false;
    }
    
    // 等待一段时间
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 尝试重新打开
    ERROR_CODE returned_state = camera_->open(init_params_);
    if (returned_state == ERROR_CODE::SUCCESS) {
        is_initialized_ = true;
        camera_info_ = camera_->getCameraInformation();
        logMessage("相机恢复成功!");
        return true;
    } else {
        logMessage("相机恢复失败: " + std::string(toString(returned_state)));
        return false;
    }
} 