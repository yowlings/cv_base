cmake_minimum_required(VERSION 2.8)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
project(opencv_base)
add_executable(opencv_base main.cpp cv_base.cpp)

target_link_libraries( opencv_base ${OpenCV_LIBS} )
