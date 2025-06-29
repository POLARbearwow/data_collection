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

# 查找 ZED SDK (可选)
find_package(ZED 3 QUIET)
if(ZED_FOUND)
    message(STATUS "ZED SDK found")
    add_definitions(-DZED_AVAILABLE)
else()
    message(WARNING "ZED SDK not found. ZED camera support will be disabled.")
endif()

# 公共源文件 (通用配置加载等)
set(COMMON_SRC src/utils.cpp)

# 新增: 海康相机源码
set(HIK_CAMERA_SRC src/hik_camera.cpp)

# 新增: ZED相机源码 (如果可用)
set(ZED_CAMERA_SRC "")
if(ZED_FOUND)
    set(ZED_CAMERA_SRC src/zed_camera.cpp)
endif()

# 头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)
# 新增: 海康 SDK 头文件路径
include_directories(/opt/MVS/include)

# 主轨迹求解器
add_executable(trajectory_solver src/main.cpp src/solver.cpp src/solver_camera.cpp ${COMMON_SRC} ${HIK_CAMERA_SRC} ${ZED_CAMERA_SRC})

# 篮球检测调试器
add_executable(basketball_detect src/basketball_detect.cpp ${COMMON_SRC})

# 链接 OpenCV 和海康相机 SDK
target_link_libraries(trajectory_solver PRIVATE ${OpenCV_LIBS} /opt/MVS/lib/64/libMvCameraControl.so)

# 如果找到ZED SDK，则链接ZED库
if(ZED_FOUND)
    target_link_libraries(trajectory_solver PRIVATE ${ZED_LIBRARIES})
    target_include_directories(trajectory_solver PRIVATE ${ZED_INCLUDE_DIRS})
    target_compile_definitions(trajectory_solver PRIVATE ${ZED_DEFINITIONS})
endif()

target_link_libraries(basketball_detect PRIVATE ${OpenCV_LIBS}) 