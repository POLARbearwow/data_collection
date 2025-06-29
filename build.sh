#!/bin/bash

# 篮球轨迹追踪系统编译脚本
# 该脚本自动化CMake编译过程

echo "=========================================="
echo "篮球轨迹追踪系统 - 编译脚本"
echo "=========================================="

# 检查是否安装了cmake
if ! command -v cmake &> /dev/null; then
    echo "错误: 未找到cmake。请先安装cmake:"
    echo "  sudo apt-get install cmake"
    exit 1
fi

# 检查是否安装了make
if ! command -v make &> /dev/null; then
    echo "错误: 未找到make。请先安装build-essential:"
    echo "  sudo apt-get install build-essential"
    exit 1
fi

# 创建build目录
echo "创建build目录..."
mkdir -p build
cd build

# 运行cmake配置
echo "运行CMake配置..."
if cmake ..; then
    echo "CMake配置成功"
else
    echo "错误: CMake配置失败"
    exit 1
fi

# 编译项目
echo "编译项目..."
if make -j$(nproc); then
    echo ""
    echo "=========================================="
    echo "编译成功！"
    echo "=========================================="
    echo "生成的可执行文件:"
    echo "  - trajectory_solver (主程序)"
    echo "  - basketball_detect (调试工具)"
    echo ""
    echo "使用方法:"
    echo "  ./trajectory_solver camera ../config/example_config.yaml"
    echo ""
    echo "或者使用演示脚本:"
    echo "  cd ../scripts && ./demo_camera_selection.sh"
    echo "=========================================="
else
    echo "错误: 编译失败"
    exit 1
fi 