cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(TARGET_NAME infantry_new)
project(${TARGET_NAME})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS  "$ENV{CXXFLAGS} -g")

# 开启多线程
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
SET(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -pthread")
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV 4 QUIET)

    
include_directories(
    /opt/MVS/include
    camera/include
    ${OpenCV_INCLUDE_DIRS}
    )

link_directories(
    /opt/MVS/lib/aarch64
    /usr/local/lib
    )
add_subdirectory(camera)
add_executable(infantry_new main.cpp)
target_link_libraries(infantry_new
    Camera
    ${OpenCV_LIBS}
    MvCameraControl
    )
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
