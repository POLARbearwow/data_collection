#!/bin/bash

echo "=== ZED驱动框架构建脚本 ==="

# 检查当前目录
if [ ! -f "CMakeLists.txt" ]; then
    echo "错误: 请在zed_demo目录下运行此脚本"
    exit 1
fi

# 创建构建目录
echo "创建构建目录..."
mkdir -p build
cd build

# 运行cmake配置
echo "配置项目..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# 检查cmake是否成功
if [ $? -ne 0 ]; then
    echo "错误: CMake配置失败"
    echo "请检查："
    echo "  - ZED SDK是否正确安装"
    echo "  - OpenCV是否正确安装"
    echo "  - CUDA是否正确安装"
    exit 1
fi

# 编译项目
echo "编译项目..."
make -j$(nproc)

# 检查编译是否成功
if [ $? -eq 0 ]; then
    echo "✅ 编译成功!"
    echo ""
    echo "可执行文件:"
    echo "  - 颜色图像捕获: ./color_capture"
    echo ""
    echo "运行示例:"
    echo "  cd build"
    echo "  ./color_capture"
else
    echo "❌ 编译失败"
    exit 1
fi 