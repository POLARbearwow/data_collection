#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include "hik_camera.hpp"
#include "basketball_detector.hpp"
#include <ctime>
#include <cmath>
#include <algorithm>
#include <filesystem>

// 定义M_PI（如果未定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace cv;
using namespace std;

//======================== 全局变量 ========================//
static const int DISPLAY_WIDTH = 640;                 // 左右两图各 640 像素, 总宽 1280
static const string MAIN_WIN   = "Basketball Debug";  // 主窗口名称
static const string TRACK_WIN  = "Filter Params";     // 滤波参数窗口

static bool freezeHSV   = false;     // 是否处于 HSV 采样冻结模式 (h 键)
static bool freezeTrack = false;     // 是否处于 参数调节冻结模式 (f 键)
static bool showTrack   = false;
static bool loadMode    = false;     // 是否处于加载模式

static Mat  snapshot;                // 冻结帧
static Mat  loadedImage;             // 加载的图像
static vector<Vec3b> hsvSamples;     // 采样的 HSV 点
static vector<Point> hsvClickPoints; // HSV采样点击位置
static BasketballDetector detector;  // 检测器实例

// 滤波参数 (由 trackbar 控制)
static int tb_close_kernel = 19;    // 闭运算 kernel (奇数)
static int tb_close_iter   = 4;    // 闭运算迭代次数
static int tb_median_kernel  = 19;    // 中值滤波 kernel (奇数,0 关闭)

//-----------------------------------------------------------//
static void updateDetectorFromTrackbar()
{
    detector.setMorphParams(tb_close_kernel, tb_close_iter);
    detector.setMedianKernel(tb_median_kernel);
}

//-----------------------------------------------------------//
static void onMouse(int event, int x, int y, int /*flags*/, void* userdata)
{
    (void)userdata;
    if (event != EVENT_LBUTTONDOWN) return;
    if (!freezeHSV) return;  // 仅在 h 键冻结状态下才采样
    
    // 确定要使用的图像源
    Mat sourceImage;
    if (loadMode) {
        sourceImage = loadedImage;
    } else {
        sourceImage = snapshot;
    }
    
    if (sourceImage.empty()) return;

    // debug 图像宽度 = DISPLAY_WIDTH * 2, 左图为原图
    double scale = static_cast<double>(DISPLAY_WIDTH) / sourceImage.cols;
    if (x >= DISPLAY_WIDTH) {
        // 点击的是右边掩码图像, 忽略
        return;
    }
    // 将点击坐标映射回原图像素坐标
    int origX = static_cast<int>(x / scale);
    int origY = static_cast<int>(y / scale);
    if (origX < 0 || origX >= sourceImage.cols || origY < 0 || origY >= sourceImage.rows) return;

    // 获取 HSV
    Vec3b bgr = sourceImage.at<Vec3b>(origY, origX);
    Mat hsvPix;
    cvtColor(Mat(1,1,CV_8UC3,&bgr), hsvPix, COLOR_BGR2HSV);
    Vec3b hsv = hsvPix.at<Vec3b>(0,0);
    hsvSamples.push_back(hsv);

    // 计算新阈值
    Vec3b low(255,255,255), high(0,0,0);
    for (const auto &v : hsvSamples) {
        for (int i=0;i<3;++i) {
            low[i]  = std::min(low[i],  v[i]);
            high[i] = std::max(high[i], v[i]);
        }
    }
    detector.setHSVRange(Scalar(low[0], low[1], low[2]), Scalar(high[0], high[1], high[2]));
    cout << "[Calib] HSV low=" << (int)low[0] << "," << (int)low[1] << "," << (int)low[2]
         << "  high=" << (int)high[0] << "," << (int)high[1] << "," << (int)high[2] << endl;

    hsvClickPoints.push_back(Point(origX, origY));
}

//-----------------------------------------------------------//
static Mat makeDisplay(const Mat &frame, const BasketballDetector::Result &res)
{
    // 生成可视化图像: 左原图 (含绘制), 右mask
    Mat visFrame = frame.clone();
    
    // 在左边图像上显示HSV采样点击位置（粗红点）
    for (const auto &clickPoint : hsvClickPoints)
    {
        circle(visFrame, clickPoint, 8, Scalar(0,0,255), -1); // 粗红点
    }
    
    // 在左上角显示模式信息
    if (loadMode) {
        putText(visFrame, "LOAD MODE", Point(10, 55), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
        putText(visFrame, "Press 'h' to start HSV sampling", Point(10, 85), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
    } else if (freezeHSV) {
        putText(visFrame, "HSV SAMPLING", Point(10, 55), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
    } else if (freezeTrack) {
        putText(visFrame, "FILTER TUNING", Point(10, 55), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);
    }
    
    // 在load模式下不显示平滑圆形
    if (!loadMode && res.hasTracking)
    {
        int displayRadius = std::max(1, std::min(1000, (int)res.smoothedRadius));
        circle(visFrame, res.smoothedCenter, displayRadius, Scalar(255,255,0), 2);
        circle(visFrame, res.smoothedCenter, 3, Scalar(255,255,0), -1);
        putText(visFrame, format("Kalman: (%.1f,%.1f) r=%.1f", 
                                res.smoothedCenter.x, res.smoothedCenter.y, res.smoothedRadius), 
                Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,0), 2);
    }

    Mat maskBGR;
    cvtColor(res.mask, maskBGR, COLOR_GRAY2BGR);

    // 在load模式下不显示检测结果
    if (!loadMode && res.found)
    {
        // 在掩码图像上绘制 inliers （洋红色）
        for (const auto &p : res.inliers)
        {
            circle(maskBGR, p, 2, Scalar(255,0,255), -1);
        }
        
        // 在掩码图像上绘制检测到的圆形（绿色）
        int detectRadius = std::max(1, std::min(1000, (int)res.radius));
        circle(maskBGR, res.center, detectRadius, Scalar(0,255,0), 2);
        circle(maskBGR, res.center, 4, Scalar(0,0,255), -1);
        
        // 在右上角打印实时检测信息
        putText(maskBGR, format("Detect: (%.1f,%.1f) r=%.1f", 
                               res.center.x, res.center.y, res.radius), 
                Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0), 2);
        putText(maskBGR, format("Inliers: %d", (int)res.inliers.size()), 
                Point(10, 50), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,255), 2);
    }

    // resize 两图到固定宽度
    double scale = static_cast<double>(DISPLAY_WIDTH) / visFrame.cols;
    int dispH = static_cast<int>(visFrame.rows * scale);

    Mat visSmall, maskSmall;
    resize(visFrame, visSmall, Size(DISPLAY_WIDTH, dispH));
    resize(maskBGR, maskSmall,  Size(DISPLAY_WIDTH, dispH));

    Mat combined;
    hconcat(visSmall, maskSmall, combined);

    return combined;
}

//-----------------------------------------------------------//
static void saveCurrentImage(const Mat &frame)
{
    string filename;
    auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
    std::tm tm;
    localtime_r(&t, &tm);
    char buf[64];
    strftime(buf, sizeof(buf), "basketball_frame_%Y%m%d_%H%M%S.png", &tm);
    filename = buf;
    
    if (imwrite(filename, frame)) {
        cout << "[Save] Image saved to " << filename << endl;
    } else {
        cerr << "[ERROR] Failed to save image to " << filename << endl;
    }
}

//-----------------------------------------------------------//
static bool loadImage()
{
    string filename;
    cout << "[Load] Enter image filename (or press Enter for default): ";
    getline(cin, filename);
    
    if (filename.empty()) {
        // 查找最新的保存文件
        vector<string> files;
        try {
            for (const auto& entry : std::filesystem::directory_iterator(".")) {
                if (entry.is_regular_file()) {
                    string path = entry.path().string();
                    if (path.find("basketball_frame_") == 0 && path.find(".png") != string::npos) {
                        files.push_back(path);
                    }
                }
            }
        } catch (const std::exception& e) {
            cerr << "[ERROR] Failed to search for files: " << e.what() << endl;
            return false;
        }
        
        if (files.empty()) {
            cerr << "[ERROR] No saved images found" << endl;
            return false;
        }
        // 按文件名排序，取最新的
        sort(files.begin(), files.end());
        filename = files.back();
        cout << "[Load] Using latest file: " << filename << endl;
    }
    
    loadedImage = imread(filename);
    if (loadedImage.empty()) {
        cerr << "[ERROR] Failed to load image: " << filename << endl;
        return false;
    }
    
    cout << "[Load] Image loaded: " << filename << " (" << loadedImage.cols << "x" << loadedImage.rows << ")" << endl;
    return true;
}

//============================= 主函数 =============================//
int main()
{
    HikCamera camera;
    if (!camera.openCamera()) {
        cerr << "[ERROR] Failed to open Hik camera" << endl;
        return -1;
    }

    // 打印初始HSV参数
    cv::Scalar hsvLow, hsvHigh;
    detector.getHSVRange(hsvLow, hsvHigh);
    cout << "\n=== Basketball Detection Tool Started ===" << endl;
    cout << "Initial HSV parameters:" << endl;
    cout << "  Low:  (" << (int)hsvLow[0] << ", " << (int)hsvLow[1] << ", " << (int)hsvLow[2] << ")" << endl;
    cout << "  High: (" << (int)hsvHigh[0] << ", " << (int)hsvHigh[1] << ", " << (int)hsvHigh[2] << ")" << endl;
    cout << "Press '?' for help" << endl;
    cout << "=======================================" << endl;

    namedWindow(MAIN_WIN, WINDOW_AUTOSIZE);
    setMouseCallback(MAIN_WIN, onMouse, nullptr);

    // 初始化 trackbar window (但默认隐藏)
    namedWindow(TRACK_WIN, WINDOW_AUTOSIZE);
    createTrackbar("CloseKernel", TRACK_WIN, &tb_close_kernel, 21, [](int,void*){ updateDetectorFromTrackbar(); });
    createTrackbar("CloseIter",   TRACK_WIN, &tb_close_iter,   10, [](int,void*){ updateDetectorFromTrackbar(); });
    createTrackbar("MedianKernel",  TRACK_WIN, &tb_median_kernel,  21, [](int,void*){ updateDetectorFromTrackbar(); });
    updateDetectorFromTrackbar();
    // 默认隐藏参数窗口
    moveWindow(TRACK_WIN, 10, 10); // 保证窗口有位置
    setWindowProperty(TRACK_WIN, WND_PROP_VISIBLE, 0);

    bool recording = false;
    VideoWriter writer;

    while (true)
    {
        Mat frame;
        if (loadMode) {
            // 加载模式：使用加载的图像
            if (loadedImage.empty()) {
                cerr << "[ERROR] No image loaded" << endl;
                break;
            }
            frame = loadedImage.clone();
        } else if (!freezeHSV && !freezeTrack) {
            // 只有在非冻结状态下更新图像
            if (!camera.getFrame(frame)) {
                cerr << "[WARN] failed to grab frame" << endl;
                continue;
            }
            snapshot = frame; // 更新最新帧 (以便 f 键冻结时用)
        } else {
            frame = snapshot; // 冻结帧
        }

        if (frame.empty()) continue;

        BasketballDetector::Result res;
        if (freezeHSV || loadMode) {
            // HSV采样模式：不进行滤波，只显示HSV掩码
            res = detector.detect(frame, false);
        } else {
            // 正常模式：进行完整检测和滤波
            res = detector.detect(frame, true);
        }
        
        Mat disp = makeDisplay(frame, res);
        imshow(MAIN_WIN, disp);

        char key = (char)waitKey(1);
        if (key == 27 || key == 'q') break; // ESC / q 退出
        else if (key == 'h' || key == 'H') {
            freezeHSV = !freezeHSV;
            if (freezeHSV) {
                if (loadMode) {
                    cout << "[Info] HSV sampling mode activated in load mode." << endl;
                } else {
                    snapshot = frame.clone();
                    cout << "[Info] Freeze for HSV sampling, click on image to add sample." << endl;
                }
            } else {
                cout << "[Info] Exit HSV sampling." << endl;
            }
        }
        else if (key == 'c') {
            hsvSamples.clear();
            hsvClickPoints.clear();
            cout << "[Info] Clear HSV samples and click points." << endl;
        }
        else if (key == 'f' || key == 'F') {
            freezeTrack = !freezeTrack;
            showTrack  = freezeTrack;
            if (freezeTrack) {
                snapshot = frame.clone();
                setWindowProperty(TRACK_WIN, WND_PROP_VISIBLE, 1);
                cout << "[Info] Freeze for filter tuning." << endl;
            } else {
                setWindowProperty(TRACK_WIN, WND_PROP_VISIBLE, 0);
                cout << "[Info] Exit filter tuning." << endl;
            }
        }
        else if (key == 's' || key == 'S') {
            // 保存当前图像
            saveCurrentImage(frame);
        }
        else if (key == 'l' || key == 'L') {
            // 加载图像
            if (loadImage()) {
                loadMode = true;
                freezeHSV = false;
                freezeTrack = false;
                hsvSamples.clear();
                hsvClickPoints.clear();
                cout << "[Info] Entered load mode for HSV calibration" << endl;
            }
        }
        else if (key == 'e' || key == 'E') {
            // 退出加载模式，返回实时模式
            if (loadMode) {
                loadMode = false;
                loadedImage.release();
                cout << "[Info] Exited load mode, returning to real-time mode" << endl;
            }
        }
        else if (key == '?') {
            // 显示帮助信息
            cout << "\n=== Basketball Detection Tool Help ===" << endl;
            cout << "h/H: Toggle HSV sampling mode" << endl;
            cout << "c: Clear HSV samples and click points" << endl;
            cout << "f/F: Toggle filter parameter tuning" << endl;
            cout << "s/S: Save current image" << endl;
            cout << "l/L: Load image for HSV calibration" << endl;
            cout << "e/E: Exit load mode (return to real-time)" << endl;
            cout << "r/R: Toggle recording" << endl;
            cout << "q/ESC: Quit" << endl;
            cout << "?: Show this help" << endl;
            cout << "================================" << endl;
        }
        else if (key == 'r' || key == 'R') {
            recording = !recording;
            if (recording) {
                string filename;
                auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
                std::tm tm;
                localtime_r(&t, &tm);
                char buf[64];
                strftime(buf, sizeof(buf), "basketball_%Y%m%d_%H%M%S.avi", &tm);
                filename = buf;

                int fourcc = VideoWriter::fourcc('M','J','P','G');
                double fps = 30.0;
                writer.open(filename, fourcc, fps, Size(frame.cols, frame.rows));
                if (!writer.isOpened()) {
                    cerr << "[ERROR] cannot open VideoWriter" << endl;
                    recording = false;
                } else {
                    cout << "[Record] Start recording to " << filename << endl;
                }
            } else {
                writer.release();
                cout << "[Record] Stop recording" << endl;
            }
        }

        if (recording && writer.isOpened()) {
            writer.write(frame);
        }
    }

    if (writer.isOpened()) writer.release();
    destroyAllWindows();
    return 0;
} 