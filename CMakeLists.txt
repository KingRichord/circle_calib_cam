cmake_minimum_required(VERSION 2.8.11)

PROJECT(CIRCLECALIBRATION)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(OpenCV_DIR "/home/moi/APP/include/opencv/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

add_executable(circlecalib circlecalib.cpp)
target_link_libraries(circlecalib     ${OpenCV_LIBRARIES})


