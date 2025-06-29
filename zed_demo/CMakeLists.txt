cmake_minimum_required(VERSION 3.12)
project(ZEDColorDriver)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# 查找依赖包
find_package(ZED 4 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

# 包含头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# 设置库文件目录
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

# 源文件
set(DRIVER_SOURCES
    src/zed_color_driver.cpp
)

# 创建颜色驱动库
add_library(zed_color_driver_lib STATIC ${DRIVER_SOURCES})
target_link_libraries(zed_color_driver_lib 
    ${ZED_LIBRARIES} 
    ${CUDA_LIBRARIES} 
    ${OpenCV_LIBS}
)

# 颜色捕获示例程序
add_executable(color_capture examples/color_capture.cpp)
target_link_libraries(color_capture zed_color_driver_lib)

# 安装规则
install(TARGETS zed_color_driver_lib color_capture
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
) 