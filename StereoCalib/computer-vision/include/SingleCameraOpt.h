//
// Created by zixia on 2021/3/10.
//

#ifndef COMPUTERVISION_SINGLECAMERAOPT_H
#define COMPUTERVISION_SINGLECAMERAOPT_H

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <math.h>
#include "loopback.h"

#define ORIGIN_WIDTH 1280
#define ORIGIN_HEIGHT 720

using namespace cv;
using namespace std;

class SingleCameraOpt {
private:
    /// 用于构造函数中复用的初始化摄像头函数
    void cameraGeneralInit();

    bool usingCamPath;

public:
    int CAMERA_INDEX{};
    string CAMERA_PATH;
    string CAMERA_NAME;

    /// 从摄像头创建SingleCameraOpt对象
    /// \param index 摄像头id
    /// \param camName 摄像头名称，用来匹配参数文件
    /// \param imgSize 分辨率
    SingleCameraOpt(int index, string camName, const Size &imgSize);

    /// 从摄像头创建SingleCameraOpt对象
    /// \param camPath 摄像头路径
    /// \param camName 摄像头名称，用来匹配参数文件
    /// \param imgSize 分辨率
    SingleCameraOpt(string camPath, string camName, const Size& imgSize);

    /// 利用摄像头id从摄像头创建SingleCameraOpt对象，并在系统中创建相同参数的虚拟摄像头
    /// \param index 同上
    /// \param camName 同上
    /// \param imgSize 同上
    /// \param virIndex 创建的虚拟摄像头的id
    SingleCameraOpt(int index, string camName, const Size &imgSize, int virIndex);

    /// 利用摄像头路径从摄像头创建SingleCameraOpt对象，并在系统中创建相同参数的虚拟摄像头
    /// \param index 同上
    /// \param camName 同上
    /// \param imgSize 同上
    /// \param virIndex 创建的虚拟摄像头的id
    SingleCameraOpt(string camPath, string camName, const Size &imgSize, int virIndex);

    /// 从图片序列文件创建SingleCameraOpt对象
    /// \param filePath 图片序列文件夹的路径
    /// \param fileNameLen 图片文件名的长度
    /// \param camName 同上
    SingleCameraOpt(const string& filePath, int fileNameLen, string camName);

    SingleCameraOpt();

    ~SingleCameraOpt();

    Size &getFrameSize();

    bool getLastFrame(Mat& frame, bool undistort, bool gray);
    bool getNewFrame(Mat& frame, bool undistort, bool gray);
    bool isOpened() const;

    const string &getPath() const;

private:
    VideoCapture cam;
    VideoCaptureLoopback *cam2{};
    Size frameSize;
    bool open = false;

    void init();

    // 工作模式：1=摄像头，2=摄像头+虚拟摄像头，3=图片序列
    int mode;

    // 图片文件夹路径
    string filePath;

    // 图片文件序号、序号长度、文件夹中第一个图片的序号
    int lastFileIndex{}, fileNameLength{}, firstFileIndex{};

    // 相机内参，从文件读取
    Mat cameraMatrix, distCoeffs;

    // 单目矫正时用到的map，通过内参计算
    Mat map1, map2, lastOriginFrame;

    bool readXML();
};

#endif //COMPUTERVISION_SINGLECAMERAOPT_H