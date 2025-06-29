#include "zed_camera.hpp"
#include <iostream>
#include <thread>

#ifdef ZED_AVAILABLE
#include <sl/Camera.hpp>
using namespace sl;
#endif

#ifdef ZED_AVAILABLE
ZEDCamera::ZEDCamera() 
    : camera_(std::make_unique<sl::Camera>())
    , is_initialized_(false)
    , consecutive_error_count_(0)
{
    std::cout << "[ZEDCamera] 构造函数被调用" << std::endl;
}

ZEDCamera::~ZEDCamera() {
    std::cout << "[ZEDCamera] 析构函数被调用" << std::endl;
    closeCamera();
}
#else
// ZED SDK不可用时的空实现
ZEDCamera::ZEDCamera() : is_initialized_(false) {
    std::cout << "[ZEDCamera] 构造函数被调用 (ZED SDK不可用)" << std::endl;
}

ZEDCamera::~ZEDCamera() {
    std::cout << "[ZEDCamera] 析构函数被调用 (ZED SDK不可用)" << std::endl;
}
#endif

#ifdef ZED_AVAILABLE
bool ZEDCamera::openCamera() {
    if (is_initialized_) {
        std::cout << "[ZEDCamera] 相机已经初始化!" << std::endl;
        return true;
    }
    
    std::cout << "[ZEDCamera] 初始化ZED相机..." << std::endl;
    
    // 配置初始化参数 - 仅颜色模式
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD1080;
    init_params.camera_fps = 30;
    init_params.depth_mode = DEPTH_MODE::NONE;  // 禁用深度计算
    init_params.coordinate_units = UNIT::METER;
    init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_params.sensors_required = false;       // 不需要传感器
    
    // 启用图像增强
    init_params.enable_image_enhancement = true;
    
    // 启用图像质量检测
    init_params.enable_image_validity_check = true;
    
    // 启用异步抓取恢复机制
    init_params.async_grab_camera_recovery = true;
    
    // 禁用不需要的功能以提高性能
    init_params.enable_right_side_measure = false;
    init_params.depth_stabilization = 0;  // 不需要深度稳定性
    
    std::cout << "[ZEDCamera] 正在打开相机..." << std::endl;
    
    // 尝试打开相机
    ERROR_CODE returned_state = camera_->open(init_params);
    if (returned_state != ERROR_CODE::SUCCESS) {
        std::cerr << "[ZEDCamera] 无法打开相机 - " << std::string(toString(returned_state)) << std::endl;
        return false;
    }
    
    // 获取相机信息
    CameraInformation camera_info = camera_->getCameraInformation();
    is_initialized_ = true;
    
    std::cout << "[ZEDCamera] ZED相机初始化成功!" << std::endl;
    std::cout << "[ZEDCamera] 型号: " << std::string(toString(camera_info.camera_model)) << std::endl;
    std::cout << "[ZEDCamera] 序列号: " << camera_info.serial_number << std::endl;
    std::cout << "[ZEDCamera] 分辨率: " << camera_info.camera_configuration.resolution.width 
              << "x" << camera_info.camera_configuration.resolution.height << std::endl;
    std::cout << "[ZEDCamera] 帧率: " << camera_info.camera_configuration.fps << std::endl;
    
    return true;
}
#else
bool ZEDCamera::openCamera() {
    std::cerr << "[ZEDCamera] 错误: ZED SDK不可用，无法打开ZED相机" << std::endl;
    return false;
}
#endif

#ifdef ZED_AVAILABLE
void ZEDCamera::closeCamera() {
    if (is_initialized_) {
        std::cout << "[ZEDCamera] 关闭相机..." << std::endl;
        camera_->close();
        is_initialized_ = false;
    }
}

bool ZEDCamera::getFrame(cv::Mat& frame) {
    if (!is_initialized_) {
        std::cerr << "[ZEDCamera] 相机未初始化!" << std::endl;
        return false;
    }
    
    const int max_retries = 5;  // 最大重试次数
    int retry_count = 0;
    
    while (retry_count < max_retries) {
        // 配置运行时参数 - 禁用深度相关功能
        RuntimeParameters runtime_params;
        runtime_params.enable_depth = false;       // 禁用深度计算
        runtime_params.enable_fill_mode = false;   // 禁用深度填充
        
        ERROR_CODE grab_state = camera_->grab(runtime_params);
        
        // 检查帧质量
        if (checkFrameQuality(grab_state)) {
            // 获取左侧彩色图像
            sl::Mat zed_image;
            ERROR_CODE returned_state = camera_->retrieveImage(zed_image, VIEW::LEFT);
            if (returned_state != ERROR_CODE::SUCCESS) {
                std::cerr << "[ZEDCamera] 获取左侧图像失败 - " << std::string(toString(returned_state)) << std::endl;
                retry_count++;
                continue;
            }
            
            // 转换为OpenCV格式
            if (!convertZEDToOpenCV(zed_image, frame)) {
                std::cerr << "[ZEDCamera] 图像格式转换失败" << std::endl;
                retry_count++;
                continue;
            }
            
            consecutive_error_count_ = 0;  // 重置连续错误计数
            return true;
        }
        
        // 处理错误情况
        consecutive_error_count_++;
        retry_count++;
        
        // 短暂延迟后重试
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    std::cerr << "[ZEDCamera] 达到最大重试次数，无法获取有效帧" << std::endl;
    return false;
}

bool ZEDCamera::isConnected() const {
    return is_initialized_ && camera_->isOpened();
}

std::string ZEDCamera::getCameraName() const {
    if (!is_initialized_) {
        return "ZED Camera (Uninitialized)";
    }
    
    CameraInformation camera_info = camera_->getCameraInformation();
    return "ZED " + std::string(toString(camera_info.camera_model)) + 
           " (SN: " + std::to_string(camera_info.serial_number) + ")";
}

bool ZEDCamera::convertZEDToOpenCV(const sl::Mat& zed_image, cv::Mat& cv_image) {
    // 获取图像属性
    int width = zed_image.getWidth();
    int height = zed_image.getHeight();
    sl::MAT_TYPE type = zed_image.getDataType();
    
    // 根据ZED图像类型创建对应的OpenCV图像
    if (type == sl::MAT_TYPE::U8_C4) {
        // BGRA格式
        cv::Mat temp(height, width, CV_8UC4, zed_image.getPtr<sl::uchar1>());
        cv::cvtColor(temp, cv_image, cv::COLOR_BGRA2BGR);
    } else if (type == sl::MAT_TYPE::U8_C3) {
        // BGR格式
        cv_image = cv::Mat(height, width, CV_8UC3, zed_image.getPtr<sl::uchar1>()).clone();
    } else {
        std::cerr << "[ZEDCamera] 不支持的图像格式: " << static_cast<int>(type) << std::endl;
        return false;
    }
    
    return !cv_image.empty();
}

bool ZEDCamera::checkFrameQuality(sl::ERROR_CODE grab_state) {
    // 检查抓取状态
    if (grab_state == ERROR_CODE::SUCCESS) {
        return true;
    } else {
        // 对于任何非成功状态，都认为帧质量有问题
        // 只在重要错误时打印日志，避免日志过多
        static int error_count = 0;
        error_count++;
        if (error_count % 100 == 1) { // 每100个错误打印一次
            std::cerr << "[ZEDCamera] 帧质量问题 (每100次显示1次): " 
                     << std::string(toString(grab_state)) << std::endl;
        }
        return false;
    }
}
#else
// ZED SDK不可用时的空实现
void ZEDCamera::closeCamera() {
    std::cout << "[ZEDCamera] 关闭相机 (ZED SDK不可用)" << std::endl;
}

bool ZEDCamera::getFrame(cv::Mat& frame) {
    std::cerr << "[ZEDCamera] 错误: ZED SDK不可用，无法获取图像" << std::endl;
    return false;
}

bool ZEDCamera::isConnected() const {
    return false;
}

std::string ZEDCamera::getCameraName() const {
    return "ZED Camera (SDK不可用)";
}
#endif 