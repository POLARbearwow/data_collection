# Basketball Trajectory Solver

## 部署流程

### 1. 环境依赖

#### 必备依赖
- C++17 编译器（如 g++ 9+）
- CMake >= 3.10
- OpenCV >= 4.5
- Git

#### 相机SDK依赖
- **海康威视相机**：MVS SDK（推荐安装到 `/opt/MVS/`）
- **Azure Kinect DK**：Azure Kinect SDK（推荐安装 `libk4a-dev`）

##### Ubuntu 安装示例：
```bash
sudo apt update
sudo apt install build-essential cmake git libopencv-dev libk4a-dev
# 海康MVS SDK请参考官方文档安装
```

### 2. 获取代码
```bash
git clone <your_repo_url>
cd data_collection
```

### 3. 编译项目
```bash
./build.sh
```
- 脚本会自动创建 build 目录并完成所有依赖检测和编译。
- 编译成功后，`build/` 目录下会生成：
  - `trajectory_solver` 主程序
  - `test_camera` 相机测试程序
  - `basketball_detect` 调试工具

### 4. 配置文件准备

- 配置文件位于 `config/` 目录。
- 推荐复制并修改 `config/kinect_config.yaml` 或 `config/example_config.yaml`。
- 主要参数：
  - `camera_type: kinect` 或 `camera_type: hik`
  - `camera_matrix` 和 `dist_coeffs`（请用你自己的标定数据）
  - 其他参数可参考注释说明

### 5. 运行程序

#### 运行主程序（Kinect示例）
```bash
cd build
./trajectory_solver camera ../config/kinect_config.yaml
```

#### 运行主程序（海康示例）
```bash
cd build
./trajectory_solver camera ../config/hik_config.yaml
```

#### 测试相机单独采集
```bash
cd build
./test_camera ../config/kinect_config.yaml
```

### 6. 常见问题
- **Kinect无法打开**：
  - 检查是否有其他程序占用Kinect（如上次未正常退出的进程）
  - 检查USB供电和线缆
  - 检查用户是否有video组权限
- **OpenCV配置文件报错**：
  - 确保YAML文件有`%YAML:1.0`头部
  - 确保所有矩阵参数格式与示例一致
- **海康相机无法打开**：
  - 检查MVS SDK是否安装、环境变量是否配置
  - 检查相机连接和IP设置

### 7. 其他说明
- 运行时可用快捷键：
  - 空格：切换检测开关
  - C：清除轨迹
  - V：开始/停止ROI视频录制
  - ESC：退出程序

---

如需更多帮助，请查阅 `README_CAMERA.md` 或联系开发者。 