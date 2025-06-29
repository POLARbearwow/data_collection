# 篮球轨迹追踪与三维解算系统

本项目是一个基于计算机视觉的篮球运动轨迹分析工具。它使用单个摄像头，通过识别场景中的 ArUco 标记来建立世界坐标系，然后对视频或单张图片中的篮球进行 HSV 颜色分割与定位，最终解算出篮球在三维空间中的坐标和运动轨迹。

## 主要功能

- **ArUco 标记识别**：自动检测视野内的 ArUco 标记，并以此建立稳定的世界坐标系。
- **三维位姿估计**：实时计算并显示 ArUco 标记相对于相机的位置、姿态和距离。
- **篮球检测**：通过可配置的 HSV 颜色阈值分割出篮球，并找到其像素中心。
- **三维坐标解算**：结合相机内参与 ArUco 位姿，将篮球的 2D 像素位置反向投影，计算出其在世界坐标系下的 3D 坐标。
- **视频与图像支持**：支持对视频流和单张静态图片进行分析。
- **辅助工具**：提供一个 Python 脚本，可方便地将视频文件拆分为图片帧序列。

## 环境依赖

在运行项目之前，请确保您的系统已安装以下依赖：

### C++ 环境 (轨迹解算器)
- **CMake** (版本 >= 3.10)
- **C++ 编译器** (支持 C++17, 例如 g++)
- **OpenCV** (版本 >= 4.0)，需要包含 `core`, `calib3d`, `imgproc`, `highgui`, `aruco` 模块。

在基于 Debian/Ubuntu 的系统上，您可以使用以下命令安装：
```bash
sudo apt-get update
sudo apt-get install build-essential cmake libopencv-dev
```

### Python 环境 (辅助脚本)
- **Python 3**
- **pip**
- **opencv-python**

使用 pip 安装 Python 依赖：
```bash
pip install opencv-python
```

## 编译 C++ 代码

本项目使用 CMake 进行构建。请按照以下步骤编译 C++ 源代码：

1.  **创建 build 文件夹**：
    ```bash
    mkdir build
    cd build
    ```

2.  **运行 CMake**：
    ```bash
    cmake ..
    ```

3.  **编译项目**：
    ```bash
    make
    ```
    编译成功后，`build` 目录下会生成两个可执行文件：`trajectory_solver` 和 `basketball_detect`。本项目主要使用 `trajectory_solver`。

## 配置文件说明

所有关键参数都集中在 `config/example_config.yaml` 文件中，在运行程序前请务必根据您的实际情况进行配置。

- `camera_type`: 相机类型，可选 `"hik"` (海康相机) 或 `"zed"` (ZED相机)。默认为 `"hik"`。
- `camera_matrix`: 3x3 相机内参矩阵。
- `dist_coeffs`: 1x5 相机畸变系数。  
- `aruco_dict_id`: 使用的 ArUco 字典类型枚举值 (例如，`DICT_5X5_1000` 对应 `7`)。
- `aruco_marker_length`: ArUco 标记的实际物理边长（单位：米）。
- `H_marker`: ArUco 标记中心点到真实地面的垂直高度（单位：米）。
- `hsv_low` / `hsv_high`: 用于分割篮球的 HSV 颜色阈值。
- `motion_plane_point` / `motion_plane_normal`: 用于定义篮球运动平面的点和法向量，这是实现 3D 解算的关键假设。

## 如何运行

### 1. 运行轨迹解算器 (C++)

`trajectory_solver` 是主程序，支持视频、图像和实时相机三种模式。

**命令格式**:
```bash
./build/trajectory_solver <mode> <input_path> [config_path]
```
- `<mode>`: 模式，可选 `video` (视频文件)、`image` (单张图片) 或 `camera` (实时相机)。
- `<input_path>`: 输入的视频或图片文件路径。当使用 `camera` 模式时可省略。
- `[config_path]`: (可选) 配置文件路径。如果省略，程序会自动在默认位置查找。

**示例 1：分析视频文件**
```bash
./build/trajectory_solver video ../videos/my_basketball_video.mp4 ../config/example_config.yaml
```

**示例 2：分析单张图片**
```bash
./build/trajectory_solver image ../pictures/test_frame.jpg
```

**示例 3：实时相机模式**
```bash
# 使用海康相机 (默认)
./build/trajectory_solver camera ../config/example_config.yaml

# 使用ZED相机 (需要修改配置文件中的camera_type为"zed")
./build/trajectory_solver camera ../config/zed_config.yaml
```

**快速演示**：
使用提供的演示脚本快速体验相机选择功能：
```bash
cd scripts
chmod +x demo_camera_selection.sh
./demo_camera_selection.sh
```

程序运行时会打开一个窗口，实时显示检测结果，包括：
- **左上角 (蓝色)**: 篮球在 ArUco 世界坐标系下的 (X,Y,Z) 坐标，以及相对于真实地面的最终高度 `H`。
- **左下角 (青色)**: ArUco 标记在相机坐标系下的坐标 `Pos @Cam`，以及距离相机的直线距离 `Dist @Cam`。
- **ArUco 标记**: 会被绿色框标出，并显示其三维坐标轴（红:X, 绿:Y, 蓝:Z）和 ID。
- **篮球**: 会被绿色轮廓标出，中心有一个红色圆点。

### 2. 运行视频抽帧脚本 (Python)

如果您需要将视频拆解成图片以供分析或调试，可以使用 `extract_frames.py` 脚本。

**命令格式**:
```bash
python3 scripts/extract_frames.py --video <video_path> --out <output_dir> [options]
```
- `--video`: 输入视频路径。
- `--out`: 输出图片的目标文件夹（会自动创建）。
- `--stride`: (可选) 抽帧间隔。例如 `--stride 5` 表示每隔 5 帧保存一张。默认为 1。
- `--start` / `--end`: (可选) 指定抽帧的起始和结束帧号。

**示例：每 10 帧提取一张图片**
```bash
python3 scripts/extract_frames.py --video videos/my_basketball_video.mp4 --out extracted_images --stride 10
```
图片将以 `frame_xxxxxx.jpg` 的格式保存在 `extracted_images` 文件夹中。 