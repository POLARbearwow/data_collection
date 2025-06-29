#!/bin/bash

# 演示相机选择功能脚本
# 本脚本展示如何在trajectory_solver中使用不同的相机类型

echo "============================================"
echo "Trajectory Solver 相机选择功能演示"
echo "============================================"

# 检查可执行文件是否存在
if [ ! -f "../build/trajectory_solver" ]; then
    echo "错误: 未找到可执行文件。请先编译项目："
    echo "  mkdir -p build && cd build"
    echo "  cmake .. && make"
    exit 1
fi

# 检查配置文件是否存在
if [ ! -f "../config/example_config.yaml" ]; then
    echo "错误: 未找到配置文件 config/example_config.yaml"
    exit 1
fi

echo "可用的相机类型:"
echo "1. HIK (海康相机) - 默认"
echo "2. ZED (ZED相机)"
echo ""

# 询问用户选择相机类型
read -p "请选择相机类型 (1 或 2，默认为1): " camera_choice

# 创建临时配置文件
config_file="../config/temp_config.yaml"
cp "../config/example_config.yaml" "$config_file"

case $camera_choice in
    2)
        echo "设置相机类型为 ZED..."
        # 修改配置文件中的相机类型
        sed -i 's/camera_type: "hik"/camera_type: "zed"/' "$config_file"
        camera_name="ZED相机"
        ;;
    *)
        echo "设置相机类型为 HIK (默认)..."
        # 确保使用HIK相机
        sed -i 's/camera_type: "zed"/camera_type: "hik"/' "$config_file"
        camera_name="HIK相机"
        ;;
esac

echo ""
echo "============================================"
echo "启动 Trajectory Solver，使用 $camera_name"
echo "============================================"
echo "配置文件: $config_file"
echo ""
echo "按键说明:"
echo "  空格键: 开启/关闭检测"
echo "  C键:    清除轨迹"
echo "  V键:    开启/关闭视频录制"
echo "  ESC键:  退出程序"
echo ""
echo "启动中..."

# 切换到build目录并运行程序
cd ../build
./trajectory_solver camera "../config/temp_config.yaml"

# 清理临时配置文件
echo ""
echo "清理临时文件..."
rm -f "../config/temp_config.yaml"

echo "演示结束。" 