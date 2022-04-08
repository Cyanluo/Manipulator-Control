cmake_minimum_required(VERSION 3.10)
project(stereocalib)

set(CMAKE_CXX_STANDARD 11)

# 寻找OpenCV库
find_package( OpenCV 4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(getDistance binocularCamera.cpp main.cpp binocularCamera.h)
target_link_libraries(getDistance ${OpenCV_LIBS})
