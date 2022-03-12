cmake_minimum_required(VERSION 3.10)
project(stereocalib)

set(CMAKE_CXX_STANDARD 11)

# 寻找OpenCV库
find_package( OpenCV 4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(calib calib.cpp)
target_link_libraries(calib ${OpenCV_LIBS})

add_executable(getImages getImages.cpp)
target_link_libraries(getImages ${OpenCV_LIBS})

add_executable(testAccuracy testAccuracy.cpp)
target_link_libraries(testAccuracy ${OpenCV_LIBS})

add_executable(use use.cpp)
target_link_libraries(use ${OpenCV_LIBS})

add_executable(getDistance getDistance.cpp)
target_link_libraries(getDistance ${OpenCV_LIBS})

