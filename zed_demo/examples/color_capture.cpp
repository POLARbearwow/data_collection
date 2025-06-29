#include "zed_color_driver.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>

int main() {
    std::cout << "=== ZED颜色驱动 - 颜色图像捕获示例 ===" << std::endl;
    
    // 创建ZED颜色驱动对象
    ZEDColorDriver driver;
    
    // 配置质量过滤器
    ColorFilterConfig config;
    config.enable_corruption_filter = true;    // 启用损坏帧过滤
    config.enable_low_quality_filter = true;   // 启用低质量帧过滤
    config.max_consecutive_errors = 10;        // 最大连续错误数
    config.quality_check_level = 2;            // 中等质量检测级别
    
    // 初始化相机（仅颜色模式）
    std::cout << "正在初始化相机（仅颜色模式）..." << std::endl;
    if (!driver.initialize(RESOLUTION::HD1080, 30, config)) {
        std::cerr << "相机初始化失败!" << std::endl;
        return -1;
    }
    
    // 获取相机信息
    auto camera_info = driver.getCameraInfo();
    int width = camera_info.camera_configuration.resolution.width;
    int height = camera_info.camera_configuration.resolution.height;
    
    std::cout << "相机初始化成功!" << std::endl;
    std::cout << "分辨率: " << width << "x" << height << std::endl;
    std::cout << "按键说明:" << std::endl;
    std::cout << "  ESC - 退出程序" << std::endl;
    std::cout << "  SPACE - 显示详细统计信息" << std::endl;
    std::cout << "  S - 保存当前图像" << std::endl;
    std::cout << "  L/R - 切换显示左/右图像" << std::endl;
    std::cout << "  B - 显示双目图像" << std::endl;
    
    // 创建显示窗口
    cv::namedWindow("ZED Color Image", cv::WINDOW_NORMAL);
    cv::resizeWindow("ZED Color Image", 1280, 720);
    
    // 图像容器
    sl::Mat color_image;
    
    // 帧率统计
    auto start_time = std::chrono::high_resolution_clock::now();
    int display_frame_count = 0;
    
    // 显示模式
    enum DisplayMode { LEFT_IMAGE, RIGHT_IMAGE, SIDE_BY_SIDE };
    DisplayMode display_mode = LEFT_IMAGE;
    
    // 保存计数
    int saved_count = 0;
    
    while (true) {
        // 抓取有效帧
        if (driver.grabValidFrame()) {
            bool image_retrieved = false;
            
            // 根据显示模式获取图像
            switch (display_mode) {
                case LEFT_IMAGE:
                    image_retrieved = driver.getLeftImage(color_image);
                    break;
                case RIGHT_IMAGE:
                    image_retrieved = driver.getRightImage(color_image);
                    break;
                case SIDE_BY_SIDE:
                    image_retrieved = driver.getSideBySideImage(color_image);
                    break;
            }
            
            if (image_retrieved) {
                // 转换为OpenCV格式显示
                cv::Mat cv_image(color_image.getHeight(), color_image.getWidth(), 
                               CV_8UC4, color_image.getPtr<sl::uchar1>(sl::MEM::CPU));
                cv::Mat display_image;
                cv::cvtColor(cv_image, display_image, cv::COLOR_BGRA2BGR);
                
                // 添加信息overlay
                display_frame_count++;
                auto current_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                    current_time - start_time);
                
                if (duration.count() > 0) {
                    double fps = static_cast<double>(display_frame_count) / duration.count();
                    
                    // 在图像上显示帧率信息
                    std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
                    cv::putText(display_image, fps_text, cv::Point(30, 50), 
                              cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 0), 2);
                    
                    // 显示当前模式
                    std::string mode_text;
                    switch (display_mode) {
                        case LEFT_IMAGE: mode_text = "Mode: Left"; break;
                        case RIGHT_IMAGE: mode_text = "Mode: Right"; break;
                        case SIDE_BY_SIDE: mode_text = "Mode: Side-by-Side"; break;
                    }
                    cv::putText(display_image, mode_text, cv::Point(30, 100), 
                              cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
                    
                    // 显示质量统计
                    auto stats = driver.getQualityStats();
                    std::string quality_text = "Valid: " + std::to_string(stats.valid_frames) + 
                                             "/" + std::to_string(stats.total_frames);
                    cv::putText(display_image, quality_text, cv::Point(30, 140), 
                              cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 0), 2);
                    
                    if (stats.dropped_frames > 0) {
                        std::string drop_text = "Dropped: " + std::to_string(stats.dropped_frames);
                        cv::putText(display_image, drop_text, cv::Point(30, 180), 
                                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                    }
                }
                
                cv::imshow("ZED Color Image", display_image);
            }
        }
        
        // 处理键盘输入
        char key = cv::waitKey(1) & 0xFF;
        if (key == 27) {  // ESC键退出
            break;
        } else if (key == ' ') {  // 空格键显示详细统计
            auto stats = driver.getQualityStats();
            
            std::cout << "\n=== 图像质量统计 ===" << std::endl;
            std::cout << "总帧数: " << stats.total_frames << std::endl;
            std::cout << "有效帧数: " << stats.valid_frames << std::endl;
            std::cout << "损坏帧数: " << stats.corrupted_frames << std::endl;
            std::cout << "低质量帧数: " << stats.low_quality_frames << std::endl;
            std::cout << "丢弃帧数: " << stats.dropped_frames << std::endl;
            std::cout << "有效帧率: " << std::fixed << std::setprecision(2) 
                      << stats.get_valid_frame_rate() * 100 << "%" << std::endl;
            std::cout << "丢帧率: " << std::fixed << std::setprecision(2) 
                      << stats.get_drop_rate() * 100 << "%" << std::endl;
            std::cout << "当前FPS: " << std::fixed << std::setprecision(1) 
                      << driver.getCurrentFPS() << std::endl;
            std::cout << "===================" << std::endl;
        } else if (key == 'l' || key == 'L') {  // 切换到左图像
            display_mode = LEFT_IMAGE;
            std::cout << "切换到左图像模式" << std::endl;
        } else if (key == 'r' || key == 'R') {  // 切换到右图像
            display_mode = RIGHT_IMAGE;
            std::cout << "切换到右图像模式" << std::endl;
        } else if (key == 'b' || key == 'B') {  // 切换到双目图像
            display_mode = SIDE_BY_SIDE;
            std::cout << "切换到双目图像模式" << std::endl;
        } else if (key == 's' || key == 'S') {  // 保存图像
            if (driver.getLeftImage(color_image)) {
                saved_count++;
                std::string filename = "zed_color_" + std::to_string(saved_count) + ".png";
                
                // 转换为OpenCV格式并保存
                cv::Mat cv_image(color_image.getHeight(), color_image.getWidth(), 
                               CV_8UC4, color_image.getPtr<sl::uchar1>(sl::MEM::CPU));
                cv::Mat save_image;
                cv::cvtColor(cv_image, save_image, cv::COLOR_BGRA2BGR);
                
                if (cv::imwrite(filename, save_image)) {
                    std::cout << "图像已保存: " << filename << std::endl;
                } else {
                    std::cout << "保存图像失败: " << filename << std::endl;
                }
            }
        }
    }
    
    // 关闭相机
    driver.close();
    cv::destroyAllWindows();
    
    // 显示最终统计
    auto final_stats = driver.getQualityStats();
    std::cout << "\n=== 最终统计 ===" << std::endl;
    std::cout << "总帧数: " << final_stats.total_frames << std::endl;
    std::cout << "有效帧数: " << final_stats.valid_frames << std::endl;
    std::cout << "丢弃帧数: " << final_stats.dropped_frames << std::endl;
    std::cout << "有效帧率: " << std::fixed << std::setprecision(2) 
              << final_stats.get_valid_frame_rate() * 100 << "%" << std::endl;
    std::cout << "平均FPS: " << std::fixed << std::setprecision(1) 
              << driver.getCurrentFPS() << std::endl;
    std::cout << "程序结束" << std::endl;
    
    return 0;
} 