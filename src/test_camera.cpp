#include "camera_interface.hpp"
#include "common.hpp"
#include "utils.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "用法: " << argv[0] << " <config.yaml>" << std::endl;
        std::cout << "示例: " << argv[0] << " ../config/kinect_config.yaml" << std::endl;
        return -1;
    }

    // 加载配置
    Config cfg;
    if (!loadConfig(argv[1], cfg)) {
        std::cerr << "Failed to load config file: " << argv[1] << std::endl;
        return -1;
    }

    std::cout << "Camera type: " << (cfg.cameraType == CameraType::HIK ? "HIK" : "KINECT") << std::endl;

    // 创建相机实例
    auto camera = CameraFactory::createCamera(cfg.cameraType);
    if (!camera) {
        std::cerr << "Failed to create camera instance" << std::endl;
        return -1;
    }

    // 打开相机
    if (!camera->openCamera()) {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }

    std::cout << "Camera info: " << camera->getCameraInfo() << std::endl;

    // 创建窗口
    const std::string windowName = "Camera Test";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

    std::cout << "Press 'q' to quit, any other key to continue..." << std::endl;

    // 获取并显示图像
    cv::Mat frame;
    while (true) {
        if (!camera->getFrame(frame)) {
            std::cerr << "Failed to get frame" << std::endl;
            continue;
        }

        // 显示图像
        cv::imshow(windowName, frame);

        // 检查按键
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) { // q, Q, or ESC
            break;
        }
    }

    // 清理
    camera->closeCamera();
    cv::destroyAllWindows();

    std::cout << "Test completed successfully!" << std::endl;
    return 0;
} 