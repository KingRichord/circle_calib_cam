cmake_minimum_required(VERSION 3.0.0)

PROJECT(CIRCLECALIBRATION)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(OpenCV_DIR "/home/moi/APP/include/opencv/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
file(GLOB SOURCES apriltag/*.cc)
add_executable(circlecalib
  main.cpp
  GridCalibrationTargetAprilgrid.cpp
  GridCalibrationTargetBase.cpp
  ${SOURCES}
)
target_link_libraries(circlecalib     ${OpenCV_LIBRARIES} Eigen3::Eigen )

target_include_directories(circlecalib PRIVATE ${EIGEN3_INCLUDE_DIR})