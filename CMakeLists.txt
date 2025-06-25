cmake_minimum_required(VERSION 3.10)

# 项目名称
project(BasketballTrajectorySolver LANGUAGES CXX)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV
find_package(OpenCV REQUIRED core calib3d imgproc highgui video aruco)

# 如果未找到 OpenCV，则给出提示
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found. Please install OpenCV and set OpenCV_DIR.")
endif()

# 公共源文件 (通用配置加载等)
set(COMMON_SRC src/utils.cpp)

# 新增: 海康相机源码
set(HIK_CAMERA_SRC src/hik_camera.cpp)

# 头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)
# 新增: 海康 SDK 头文件路径
include_directories(/opt/MVS/include)

# 主轨迹求解器
add_executable(trajectory_solver src/main.cpp src/solver.cpp src/solver_camera.cpp ${COMMON_SRC} ${HIK_CAMERA_SRC})

# 篮球检测调试器
add_executable(basketball_detect src/basketball_detect.cpp ${COMMON_SRC})

# 链接 OpenCV 和海康相机 SDK
target_link_libraries(trajectory_solver PRIVATE ${OpenCV_LIBS} /opt/MVS/lib/64/libMvCameraControl.so)
target_link_libraries(basketball_detect PRIVATE ${OpenCV_LIBS}) 