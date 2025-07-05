#include "camera_interface.hpp"
#include "hik_camera_adapter.hpp"
#include "kinect_camera_adapter.hpp"
#include <memory>

// 相机工厂类实现
std::unique_ptr<CameraInterface> CameraFactory::createCamera(CameraType type) {
    switch (type) {
        case CameraType::HIK:
            return std::make_unique<HikCameraAdapter>();
        case CameraType::KINECT:
            return std::make_unique<KinectCameraAdapter>();
        default:
            return nullptr;
    }
} 