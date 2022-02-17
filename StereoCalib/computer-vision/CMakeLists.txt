cmake_minimum_required(VERSION 3.15)
project(ComputerVision)

set(CMAKE_CXX_STANDARD 17)

add_executable(calibration calibration/calibration.cpp)
add_executable(test test.cpp include/SingleCameraOpt.h src/SingleCameraOpt.cpp include/DoubleCameraOpt.h src/DoubleCameraOpt.cpp)
add_executable(stereoCalibration calibration/stereoCalibration.cpp)
add_executable(takePhotoSingle capture/takePhotoSingle.cpp)
add_executable(takePhotoDouble capture/takePhotoDouble.cpp)
add_executable(takePhotoDoubleAfter capture/takePhotoDoubleAfter.cpp include/SingleCameraOpt.h src/SingleCameraOpt.cpp)
add_executable(recordVideo capture/recordVideo.cpp)
add_executable(testCpp testCpp.cpp)
add_executable(recordProcessing recordProcessing.cpp include/SingleCameraOpt.h src/SingleCameraOpt.cpp include/DoubleCameraOpt.h src/DoubleCameraOpt.cpp)

# OpenCV
FIND_PACKAGE(OpenCV)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(calibration ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(test ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(takePhotoSingle ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(takePhotoDouble ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(takePhotoDoubleAfter ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(stereoCalibration ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(recordVideo ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(recordProcessing ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(testCpp ${OpenCV_LIBS})

# virtualCamera
TARGET_LINK_LIBRARIES(test libloopback.so)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(test Threads::Threads)
