#include "hik_camera_adapter.hpp"
#include <iostream>

HikCameraAdapter::HikCameraAdapter() : handle(nullptr), pData_(nullptr), isOpen_(false) {
}

HikCameraAdapter::~HikCameraAdapter() {
    closeCamera();
}

bool HikCameraAdapter::openCamera() {
    if (isOpen_) {
        std::cout << "HikCamera already opened!" << std::endl;
        return true;
    }

    // 枚举设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(stDeviceList));
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        std::cerr << "MV_CC_EnumDevices fail! nRet = " << nRet << std::endl;
        return false;
    }

    if (stDeviceList.nDeviceNum == 0) {
        std::cerr << "No camera found!" << std::endl;
        return false;
    }

    // 选择第一个设备
    int nDeviceIndex = 0;
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nDeviceIndex]);
    if (MV_OK != nRet) {
        std::cerr << "MV_CC_CreateHandle fail! nRet = " << nRet << std::endl;
        return false;
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        std::cerr << "MV_CC_OpenDevice fail! nRet = " << nRet << std::endl;
        cleanup();
        return false;
    }

    // 设置触发模式为off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
    if (MV_OK != nRet) {
        std::cerr << "MV_CC_SetTriggerMode fail! nRet = " << nRet << std::endl;
        cleanup();
        return false;
    }

    // 开始取流
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
        std::cerr << "MV_CC_StartGrabbing fail! nRet = " << nRet << std::endl;
        cleanup();
        return false;
    }

    isOpen_ = true;
    std::cout << "HikCamera opened successfully!" << std::endl;
    return true;
}

void HikCameraAdapter::closeCamera() {
    if (isOpen_) {
        cleanup();
        isOpen_ = false;
        std::cout << "HikCamera closed." << std::endl;
    }
}

bool HikCameraAdapter::getFrame(cv::Mat& frame) {
    if (!isOpen_) {
        std::cerr << "HikCamera not opened!" << std::endl;
        return false;
    }

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(stImageInfo));
    unsigned char* pData = nullptr;
    int nDataSize = 0;

    int nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
    if (MV_OK == nRet) {
        // 分配内存
        if (pData_ == nullptr || stImageInfo.nFrameLen > nDataSize) {
            delete[] pData_;
            pData_ = new unsigned char[stImageInfo.nFrameLen];
        }
        
        // 拷贝数据
        memcpy(pData_, pData, stImageInfo.nFrameLen);
        
        // 转换为OpenCV格式
        return convertToMat(pData_, frame);
    } else {
        std::cerr << "MV_CC_GetOneFrameTimeout fail! nRet = " << nRet << std::endl;
        return false;
    }
}

bool HikCameraAdapter::isOpen() const {
    return isOpen_;
}

std::string HikCameraAdapter::getCameraInfo() const {
    if (!isOpen_) {
        return "HikCamera: Not opened";
    }
    return "HikCamera: GigE/USB Camera";
}

bool HikCameraAdapter::setExposureTime(float exposureTime) {
    if (!isOpen_) {
        std::cerr << "Camera not opened!" << std::endl;
        return false;
    }

    int nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposureTime);
    if (MV_OK != nRet) {
        std::cerr << "MV_CC_SetExposureTime fail! nRet = " << nRet << std::endl;
        return false;
    }
    return true;
}

bool HikCameraAdapter::setGain(float gain) {
    if (!isOpen_) {
        std::cerr << "Camera not opened!" << std::endl;
        return false;
    }

    int nRet = MV_CC_SetFloatValue(handle, "Gain", gain);
    if (MV_OK != nRet) {
        std::cerr << "MV_CC_SetGain fail! nRet = " << nRet << std::endl;
        return false;
    }
    return true;
}

bool HikCameraAdapter::getExposureTime(float& exposureTime) {
    if (!isOpen_) {
        std::cerr << "Camera not opened!" << std::endl;
        return false;
    }

    MVCC_FLOATVALUE stParam = {0};
    int nRet = MV_CC_GetFloatValue(handle, "ExposureTime", &stParam);
    if (MV_OK != nRet) {
        std::cerr << "MV_CC_GetExposureTime fail! nRet = " << nRet << std::endl;
        return false;
    }
    exposureTime = stParam.fCurValue;
    return true;
}

bool HikCameraAdapter::getGain(float& gain) {
    if (!isOpen_) {
        std::cerr << "Camera not opened!" << std::endl;
        return false;
    }

    MVCC_FLOATVALUE stParam = {0};
    int nRet = MV_CC_GetFloatValue(handle, "Gain", &stParam);
    if (MV_OK != nRet) {
        std::cerr << "MV_CC_GetGain fail! nRet = " << nRet << std::endl;
        return false;
    }
    gain = stParam.fCurValue;
    return true;
}

bool HikCameraAdapter::convertToMat(unsigned char* pData, cv::Mat& frame) {
    if (pData == nullptr) {
        return false;
    }

    // 简化实现：假设图像是BGR格式
    // 在实际使用中，需要根据相机的实际配置来调整
    // 这里使用固定的图像尺寸，实际应该从相机获取
    int width = 1920;  // 默认宽度
    int height = 1080; // 默认高度
    
    // 尝试从相机获取图像尺寸
    MVCC_INTVALUE stParam = {0};
    int nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK == nRet) {
        // 根据数据大小估算图像尺寸
        // 假设是BGR格式，每个像素3字节
        int totalPixels = stParam.nCurValue / 3;
        width = sqrt(totalPixels * 16.0 / 9.0);  // 假设16:9比例
        height = totalPixels / width;
    }
    
    // 创建OpenCV Mat
    frame = cv::Mat(height, width, CV_8UC3, pData);
    
    return true;
}

void HikCameraAdapter::cleanup() {
    if (handle) {
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        handle = nullptr;
    }
    
    if (pData_) {
        delete[] pData_;
        pData_ = nullptr;
    }
} 