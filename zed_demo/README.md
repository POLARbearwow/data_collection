# ZED颜色图像驱动

这是一个专为ZED相机设计的颜色图像驱动，提供了图像质量检测和异常处理功能。

## 功能特点

### ZED颜色驱动 (zed_color_driver.h)
- 专门用于颜色图像捕获
- 简化的API，高性能优化
- 自动图像质量检测和异常帧过滤
- 支持左图像、右图像、双目图像获取
- 自动相机恢复和错误处理机制

## 主要特性

✅ **自动图像质量检测** - 检测损坏帧、低质量帧等异常情况  
✅ **异常帧自动丢弃** - 自动过滤不正常的图像  
✅ **相机健康监控** - 实时监控相机状态  
✅ **自动恢复机制** - 连接断开时自动重连  
✅ **详细统计信息** - 帧率、丢帧率、质量统计等  
✅ **日志系统** - 可自定义日志输出  

## 编译要求

- ZED SDK 4.0+
- OpenCV 4.0+
- CUDA 11.0+
- CMake 3.12+
- C++17

## 编译方法

```bash
cd zed_demo
mkdir build && cd build
cmake ..
make -j4
```

## 使用示例

### 基本颜色图像捕获

```cpp
#include "zed_color_driver.h"

int main() {
    ZEDColorDriver driver;
    
    // 配置质量过滤器
    ColorFilterConfig config;
    config.enable_corruption_filter = true;
    config.enable_low_quality_filter = true;
    config.quality_check_level = 2;
    
    // 初始化相机
    if (!driver.initialize(RESOLUTION::HD1080, 30, config)) {
        return -1;
    }
    
    sl::Mat image;
    while (true) {
        // 抓取有效帧（自动过滤异常）
        if (driver.grabValidFrame()) {
            // 获取左侧图像
            if (driver.getLeftImage(image)) {
                // 处理图像...
            }
        }
    }
    
    driver.close();
    return 0;
}
```

### 图像质量统计

```cpp
// 获取质量统计信息
auto stats = driver.getQualityStats();
std::cout << "总帧数: " << stats.total_frames << std::endl;
std::cout << "有效帧数: " << stats.valid_frames << std::endl;
std::cout << "丢弃帧数: " << stats.dropped_frames << std::endl;
std::cout << "有效帧率: " << stats.get_valid_frame_rate() * 100 << "%" << std::endl;
std::cout << "当前FPS: " << driver.getCurrentFPS() << std::endl;
```

## 运行示例程序

### 颜色图像捕获
```bash
./color_capture
```
**按键说明:**
- `ESC` - 退出程序
- `SPACE` - 显示详细统计信息
- `S` - 保存当前图像
- `L/R` - 切换显示左/右图像
- `B` - 显示双目图像

## 配置说明

### ColorFilterConfig 参数

```cpp
struct ColorFilterConfig {
    bool enable_corruption_filter = true;      // 启用损坏帧过滤
    bool enable_low_quality_filter = true;     // 启用低质量帧过滤
    int max_consecutive_errors = 10;           // 最大连续错误数
    int quality_check_level = 2;               // 质量检测级别 (1-3)
};
```

**质量检测级别说明:**
- `1` - 基本健康检查
- `2` - 图像质量处理（推荐）
- `3` - 高级模糊和质量检查（最严格）

## API 参考

### ZEDColorDriver 主要方法

| 方法 | 说明 |
|-----|------|
| `initialize()` | 初始化相机（仅颜色模式） |
| `grabValidFrame()` | 抓取有效帧（自动质量检测） |
| `getLeftImage()` | 获取左侧彩色图像 |
| `getRightImage()` | 获取右侧彩色图像 |
| `getSideBySideImage()` | 获取双目图像 |
| `getQualityStats()` | 获取图像质量统计 |
| `getCurrentFPS()` | 获取当前帧率 |
| `isConnected()` | 检查相机连接状态 |
| `close()` | 关闭相机 |

## 错误处理

驱动会自动处理以下异常情况：
- 损坏帧检测和丢弃
- 低图像质量检测
- 相机连接丢失自动恢复
- 连续错误时的重连机制

## 性能优化

颜色驱动通过以下方式优化性能：
- 禁用深度计算 (`DEPTH_MODE::NONE`)
- 禁用传感器数据采集
- 禁用不必要的测量功能
- 专注于颜色图像的快速获取

## 注意事项

1. 此驱动专门为颜色图像捕获优化，无深度功能
2. 质量检测级别只能在初始化时设置
3. 建议在实际应用中设置日志回调函数
4. 确保CUDA和ZED SDK正确安装

## 故障排除

**相机初始化失败:**
- 检查ZED相机是否正确连接
- 确认ZED SDK版本兼容性
- 检查CUDA驱动是否正确安装

**帧率较低:**
- 降低分辨率设置
- 调整质量检测级别
- 检查USB连接质量

**大量丢帧:**
- 检查光照条件
- 调整质量过滤配置
- 检查相机镜头是否清洁 