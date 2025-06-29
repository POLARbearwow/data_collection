#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

#ifdef ZED_AVAILABLE
// 前向声明，避免在头文件中包含ZED SDK
namespace sl {
    class Camera;
    enum class RESOLUTION;
    enum class ERROR_CODE;
    class Mat;
    struct CameraInformation;
}
#endif

/**
 * @brief ZED相机简化驱动类
 * 
 * 专门为trajectory_solver设计的ZED相机接口
 * 提供简单的图像获取功能
 */
class ZEDCamera {
public:
    /**
     * @brief 构造函数
     */
    ZEDCamera();
    
    /**
     * @brief 析构函数
     */
    ~ZEDCamera();
    
    /**
     * @brief 打开相机
     * @return 是否成功打开
     */
    bool openCamera();
    
    /**
     * @brief 关闭相机
     */
    void closeCamera();
    
    /**
     * @brief 获取一帧图像
     * @param frame 输出的OpenCV图像
     * @return 是否成功获取
     */
    bool getFrame(cv::Mat& frame);
    
    /**
     * @brief 检查相机是否已连接
     * @return 是否已连接
     */
    bool isConnected() const;
    
    /**
     * @brief 获取相机名称
     * @return 相机名称
     */
    std::string getCameraName() const;

private:
#ifdef ZED_AVAILABLE
    std::unique_ptr<sl::Camera> camera_;        // ZED相机对象
    bool is_initialized_;                       // 是否已初始化
    int consecutive_error_count_;               // 连续错误计数
    
    /**
     * @brief 将ZED图像转换为OpenCV图像
     * @param zed_image ZED图像
     * @param cv_image 输出的OpenCV图像
     * @return 是否成功转换
     */
    bool convertZEDToOpenCV(const sl::Mat& zed_image, cv::Mat& cv_image);
    
    /**
     * @brief 检查帧质量并过滤有问题的图像
     * @param grab_state 抓取状态
     * @return 帧是否有效
     */
    bool checkFrameQuality(sl::ERROR_CODE grab_state);
#else
    bool is_initialized_;                       // 是否已初始化
#endif
}; 