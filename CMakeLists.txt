cmake_minimum_required(VERSION 3.10)

# 项目名称
project(BasketballTrajectorySolver LANGUAGES CXX)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV
find_package(OpenCV REQUIRED core calib3d imgproc highgui aruco)

# 如果未找到 OpenCV，则给出提示
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found. Please install OpenCV and set OpenCV_DIR.")
endif()

# 公共源文件 (通用配置加载等)
set(COMMON_SRC src/utils.cpp)

# 头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)

# 主轨迹求解器
add_executable(trajectory_solver src/main.cpp src/solver.cpp ${COMMON_SRC})

# 篮球检测调试器
add_executable(basketball_detect src/basketball_detect.cpp ${COMMON_SRC})

# 链接 OpenCV
target_link_libraries(trajectory_solver PRIVATE ${OpenCV_LIBS})
target_link_libraries(basketball_detect PRIVATE ${OpenCV_LIBS}) 