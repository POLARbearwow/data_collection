#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include "common.hpp"
#include "solver.hpp"
#include <vector>
#include "utils.hpp"
#include <filesystem>
namespace fs = std::filesystem;

using namespace std;

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        cerr << "用法: " << argv[0] << " <mode> <path> [config.yaml]\n"
             << "  mode: video | image\n"
             << "示例: " << argv[0] << " video myvideo.mp4 config.yaml" << endl;
        return -1;
    }

    string mode = argv[1];
    string inputPath = argv[2];
    string configPath;
    if (argc >= 4)
        configPath = argv[3];
    else
    {
        std::vector<std::string> candidates = {
            "../config/example_config.yaml",   // build 目录执行
            "config/example_config.yaml",      // 源码根目录执行
            "./example_config.yaml"            // 当前目录
        };

        for(const auto& path : candidates){
            if(fs::exists(path)){
                configPath = path;
                break;
            }
        }
    }

    cout << "加载配置文件: " << configPath << endl;

    Config cfg;
    if (!loadConfig(configPath, cfg))
        return -1;

    if (mode == "video")
    {
        runTrajectorySolver(inputPath, cfg);
    }
    else if (mode == "image")
    {
        runTrajectorySolverImage(inputPath, cfg);
    }
    else
    {
        cerr << "未知模式: " << mode << " (需要 video 或 image)" << endl;
        return -1;
    }
    return 0;
} 