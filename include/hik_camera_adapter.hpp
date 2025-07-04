#pragma once
#include "camera_interface.hpp"
#include "MvCameraControl.h"
#include <string>

class HikCameraAdapter : public CameraInterface {
public:
    HikCameraAdapter();
    ~HikCameraAdapter() override;

    // CameraInterface实现
    bool openCamera() override;
    void closeCamera() override;
    bool getFrame(cv::Mat& frame) override;
    bool isOpen() const override;
    std::string getCameraInfo() const override;

    // HikCamera特有设置
    bool setExposureTime(float exposureTime);
    bool setGain(float gain);
    bool getExposureTime(float& exposureTime);
    bool getGain(float& gain);

private:
    void* handle;  // 相机句柄
    unsigned char* pData_;  // 图像数据缓存
    bool isOpen_;  // 相机是否打开
    
    // 将原始数据转换为OpenCV格式
    bool convertToMat(unsigned char* pData, cv::Mat& frame);
    
    // 清理资源
    void cleanup();
    
    // 禁用拷贝构造和赋值
    HikCameraAdapter(const HikCameraAdapter&) = delete;
    HikCameraAdapter& operator=(const HikCameraAdapter&) = delete;
}; 