#!/bin/bash

# 构建脚本
echo "Building Basketball Trajectory Solver with Multi-Camera Support..."

# 创建构建目录
mkdir -p build
cd build

# 运行CMake
echo "Running CMake..."
cmake ..

# 检查CMake是否成功
if [ $? -ne 0 ]; then
    echo "CMake failed!"
    exit 1
fi

# 编译
echo "Compiling..."
make -j$(nproc)

# 检查编译是否成功
if [ $? -ne 0 ]; then
    echo "Compilation failed!"
    exit 1
fi

echo "Build completed successfully!"
echo ""
echo "Available executables:"
echo "  - trajectory_solver: Main trajectory solver with camera support"
echo "  - basketball_detect: Basketball detection debug tool"
echo "  - test_camera: Camera interface test tool"
echo ""
echo "Usage examples:"
echo "  ./trajectory_solver camera ../config/kinect_config.yaml"
echo "  ./trajectory_solver camera ../config/hik_config.yaml"
echo "  ./test_camera ../config/kinect_config.yaml" 