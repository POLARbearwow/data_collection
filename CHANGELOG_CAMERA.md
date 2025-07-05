# 相机接口扩展变更日志

## 概述
为trajectory solver程序添加了多相机支持，现在除了海康威视相机外，还支持Azure Kinect相机。

## 新增文件

### 头文件
- `include/camera_interface.hpp` - 抽象相机接口定义
- `include/hik_camera_adapter.hpp` - 海康相机适配器头文件
- `include/kinect_camera_adapter.hpp` - Kinect相机适配器头文件

### 源文件
- `src/camera_factory.cpp` - 相机工厂类实现
- `src/hik_camera_adapter.cpp` - 海康相机适配器实现
- `src/kinect_camera_adapter.cpp` - Kinect相机适配器实现
- `src/test_camera.cpp` - 相机接口测试程序

### 配置文件
- `config/kinect_config.yaml` - Kinect相机配置示例

### 文档
- `README_CAMERA.md` - 相机接口使用说明
- `CHANGELOG_CAMERA.md` - 本变更日志

### 脚本
- `build.sh` - 构建脚本

## 修改的文件

### 核心文件
- `include/common.hpp` - 添加相机类型枚举和配置
- `src/utils.cpp` - 添加相机类型配置加载
- `src/solver_camera.cpp` - 使用新的相机接口
- `CMakeLists.txt` - 添加新源文件和依赖

## 主要功能

### 1. 抽象相机接口
- 定义了`CameraInterface`抽象类
- 统一了不同相机的接口
- 支持动态相机类型选择

### 2. 相机工厂模式
- `CameraFactory`类用于创建相机实例
- 支持运行时相机类型选择
- 易于扩展新的相机类型

### 3. 海康相机适配器
- 基于现有`HikCamera`类实现
- 实现`CameraInterface`接口
- 保持原有功能不变

### 4. Kinect相机适配器
- 基于`kinect_demo`中的`KinectCamera`实现
- 支持多种图像格式（BGRA32, MJPG, YUY2, NV12）
- 自动格式转换到OpenCV BGR格式

### 5. 配置系统扩展
- 支持`camera_type`配置项
- 可选值：`hik` 或 `kinect`
- 向后兼容现有配置文件

## 编译支持

### 依赖检测
- 自动检测Azure Kinect SDK
- 如果未找到Kinect SDK，会显示警告但不会阻止编译
- 支持多种SDK安装路径

### 链接库
- 海康相机：`libMvCameraControl.so`
- Kinect相机：`libk4a.so`（如果可用）

## 使用方法

### 1. 配置文件设置
```yaml
# 选择相机类型
camera_type: kinect  # 或 hik
```

### 2. 编译项目
```bash
./build.sh
```

### 3. 运行程序
```bash
# 使用Kinect相机
./build/trajectory_solver camera config/kinect_config.yaml

# 使用海康相机
./build/trajectory_solver camera config/hik_config.yaml

# 测试相机接口
./build/test_camera config/kinect_config.yaml
```

## 技术细节

### 设计模式
- **工厂模式**：用于创建相机实例
- **适配器模式**：将现有相机类适配到统一接口
- **策略模式**：运行时选择不同的相机实现

### 内存管理
- 使用智能指针管理相机实例
- 自动资源清理
- RAII原则

### 错误处理
- 完善的错误检查和报告
- 优雅的降级处理
- 详细的日志输出

## 兼容性

### 向后兼容
- 现有配置文件无需修改（默认使用海康相机）
- 现有API保持不变
- 现有功能完全保留

### 扩展性
- 易于添加新的相机类型
- 清晰的接口定义
- 模块化设计

## 测试

### 测试程序
- `test_camera`程序用于验证相机接口
- 支持实时图像显示
- 简单的用户交互

### 测试覆盖
- 相机创建和初始化
- 图像获取和显示
- 资源清理
- 错误处理

## 未来扩展

### 可能的改进
1. 添加更多相机类型支持（如USB相机、网络相机等）
2. 支持相机参数动态配置
3. 添加相机状态监控
4. 支持多相机同步

### 扩展指南
1. 实现`CameraInterface`接口
2. 在`CameraFactory`中添加新类型
3. 更新CMakeLists.txt
4. 添加相应的配置支持 