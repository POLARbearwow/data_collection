# 相机接口使用说明

本项目现在支持多种相机类型，包括海康威视相机和Azure Kinect相机。

## 支持的相机类型

### 1. 海康威视相机 (HIK)
- 支持GigE和USB接口的海康威视工业相机
- 需要安装海康威视MVS SDK

### 2. Azure Kinect相机 (KINECT)
- 支持Azure Kinect DK和Azure Kinect DK for Developers
- 需要安装Azure Kinect SDK

## 配置方法

### 1. 配置文件设置

在配置文件中添加相机类型选择：

```yaml
# 相机类型: kinect 或 hik
camera_type: kinect  # 或 hik
```

### 2. 相机参数配置

#### 海康相机配置
使用现有的配置文件格式，确保相机内参和畸变系数正确标定。

#### Kinect相机配置
- 相机内参矩阵：需要根据实际Kinect相机进行标定
- 畸变系数：通常Kinect相机的畸变较小，可以设置为零
- 参考 `config/kinect_config.yaml` 示例

## 编译要求

### 海康相机
- 安装海康威视MVS SDK
- SDK路径：`/opt/MVS/`

### Kinect相机
- 安装Azure Kinect SDK
- 常见安装路径：
  - Ubuntu: `/usr/include/k4a/` 和 `/usr/lib/libk4a.so`
  - 自定义安装: `/opt/azure-kinect-sdk/`

## 使用方法

### 1. 编译项目
```bash
mkdir build && cd build
cmake ..
make
```

### 2. 运行程序
```bash
# 使用海康相机
./trajectory_solver ../config/hik_config.yaml

# 使用Kinect相机
./trajectory_solver ../config/kinect_config.yaml
```

## 相机标定

### Kinect相机标定
1. 使用OpenCV的标定工具对Kinect相机进行标定
2. 获取相机内参矩阵和畸变系数
3. 更新配置文件中的相应参数

### 标定工具推荐
- OpenCV的calibrateCamera函数
- 使用棋盘格或ArUco标记进行标定

## 注意事项

1. **SDK依赖**：确保相应的相机SDK已正确安装
2. **权限问题**：Kinect相机可能需要USB权限，确保用户有访问权限
3. **相机连接**：确保相机正确连接并被系统识别
4. **配置文件**：根据实际相机参数调整配置文件

## 故障排除

### Kinect相机问题
1. 检查Azure Kinect SDK是否正确安装
2. 确认相机USB连接正常
3. 检查用户权限（可能需要将用户添加到video组）

### 海康相机问题
1. 检查MVS SDK安装路径
2. 确认相机网络连接（GigE）或USB连接正常
3. 检查相机IP设置（GigE相机）

## 扩展支持

如需添加其他类型的相机支持：
1. 实现`CameraInterface`接口
2. 在`CameraFactory`中添加新的相机类型
3. 更新CMakeLists.txt添加相应的依赖 