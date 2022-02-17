//
// Created by zixia on 2021/3/10.
//

#include "../include/SingleCameraOpt.h"

#include <utility>

SingleCameraOpt::SingleCameraOpt(int index, string camName, const Size& imgSize) :
CAMERA_INDEX(index), CAMERA_NAME(std::move(camName)), usingCamPath(false) {
    cout << endl << "----------" << CAMERA_NAME << "----------" << endl;

    mode = 1;
    cam = VideoCapture(CAMERA_INDEX);
    if (!cam.isOpened()) {
        open = false;
        std::cout << "can not open camera!" << std::endl;
        return;
    }
    open = true;

    cam.set(CAP_PROP_FRAME_WIDTH, imgSize.width);
    cam.set(CAP_PROP_FRAME_HEIGHT, imgSize.height);
    cam.set(CAP_PROP_FPS, 25);
    cam.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));


    Mat tf;
    cam >> tf;
    lastOriginFrame = tf;
    // todo 正确设置帧大小，并相应调整相机内参
    frameSize = tf.size();

    init();

    cout << "----------" << CAMERA_NAME << " OK----------" << endl;
}

SingleCameraOpt::SingleCameraOpt(int index, string camName, const Size &imgSize, int virIndex) :
CAMERA_INDEX(index), CAMERA_NAME(std::move(camName)), usingCamPath(false) {

    cout << endl << "----------" << CAMERA_NAME << "----------" << endl;

    cam2 = new VideoCaptureLoopback(index, virIndex, ORIGIN_WIDTH, ORIGIN_HEIGHT, 25);
    if (!cam2->isOpened()) {
        open = false;
        std::cout << "can not open camera!" << std::endl;
        return;
    }
    cameraGeneralInit();
}

SingleCameraOpt::SingleCameraOpt(string camPath, string camName, const Size &imgSize, int virIndex) :
        CAMERA_PATH(camPath), CAMERA_NAME(std::move(camName)), usingCamPath(true) {

    cout << endl << "----------" << CAMERA_NAME << "----------" << endl;

    cam2 = new VideoCaptureLoopback(camPath, virIndex, imgSize.width, imgSize.height, 25);
    if (!cam2->isOpened()) {
        open = false;
        std::cout << "can not open camera!" << std::endl;
        return;
    }
    cameraGeneralInit();
}

SingleCameraOpt::SingleCameraOpt(string camPath, string camName, const Size& imgSize) :
        CAMERA_PATH(camPath), CAMERA_NAME(std::move(camName)), usingCamPath(false) {
    cout << endl << "----------" << CAMERA_NAME << "----------" << endl;

    mode = 1;
    cam = VideoCapture(CAMERA_PATH);
    if (!cam.isOpened()) {
        open = false;
        std::cout << "can not open camera!" << std::endl;
        return;
    }
    open = true;

    cam.set(CAP_PROP_FRAME_WIDTH, imgSize.width);
    cam.set(CAP_PROP_FRAME_HEIGHT, imgSize.height);
    cam.set(CAP_PROP_FPS, 25);
    cam.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));


    Mat tf;
    cam >> tf;
    lastOriginFrame = tf;
    // todo 正确设置帧大小，并相应调整相机内参
    frameSize = tf.size();

    init();

    cout << "----------" << CAMERA_NAME << " OK----------" << endl;
}

SingleCameraOpt::SingleCameraOpt(const string& filePath, int fileNameLen, string camName) :
        filePath(filePath), fileNameLength(fileNameLen),
        CAMERA_NAME(std::move(camName)), usingCamPath(false), firstFileIndex(0) {
    cout << endl << "----------read " << CAMERA_NAME << " from file----------" << endl;
    mode = 3;

    // 寻找第一个图片的index
    lastFileIndex = -1;
    Mat tf;
    bool findImg = false;
    string imgID;
    for (int i = 0; i < pow(10, fileNameLength) - 1; ++i) {
        imgID = to_string(++lastFileIndex);
        tf = imread(filePath + string(fileNameLength - imgID.length(), '0') + imgID + ".jpg");
        if (!tf.empty())  {
            // 找到图片
            findImg = true;
            break;
        }
    }

    if (findImg) {
        // 第一个图片index为lastFileIndex
        firstFileIndex = lastFileIndex;
        lastFileIndex--;
    } else {
        // 未找到图片
        cout << "ERROR! can't read " << CAMERA_NAME << " from: " << filePath << endl;
        return;
    }

    // 初始化参数
    frameSize = tf.size();
    init();

    cout << "----------" << CAMERA_NAME << " OK----------" << endl;
}

void SingleCameraOpt::cameraGeneralInit() {
    open = true;
    mode = 2;

    Mat tf;
    *cam2 >> tf;
    lastOriginFrame = tf;

    // todo 正确设置帧大小，并相应调整相机内参
    frameSize = tf.size();

    init();

    cout << "----------" << CAMERA_NAME << " OK----------" << endl;
}

void SingleCameraOpt::init(){
    if (!readXML()) {
        cout << "can not find camera calibration file!" << endl;
        return;
    }
    cout << endl << "cameraMatrix" << endl << cameraMatrix << endl;
    cout << endl << "distCoeffs" << endl << distCoeffs << endl;

//    initUndistortRectifyMap(
//                cameraMatrix, distCoeffs, Mat(),
//                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, frameSize, 1, frameSize, nullptr),
//                frameSize,CV_16SC2, map1, map2);

    initUndistortRectifyMap(
                cameraMatrix, distCoeffs, Mat(),
                cameraMatrix,
                frameSize,CV_16SC2, map1, map2);
}

bool SingleCameraOpt::readXML() {
    FileStorage fs("../res/camera_parameters/" + CAMERA_NAME + ".xml", FileStorage::READ);
    if (fs.isOpened()) {
        // 成功打开文件，开始读数据
        fs["cameraMatrix"] >> cameraMatrix;
        fs["distCoeffs"] >> distCoeffs;
        return true;
    } else {
        cout << "Error: can't open file \"" << "../res/camera_parameters/" + CAMERA_NAME + ".xml" << "\"" << endl;
        return false;
    }
}

Size &SingleCameraOpt::getFrameSize() {
    return frameSize;
}

bool SingleCameraOpt::getLastFrame(Mat& frame, bool undistort, bool gray) {
    Mat f(lastOriginFrame), f2;
    if (gray)
        cvtColor(f, f, COLOR_BGR2GRAY);
    if (undistort) {
        remap(f, f2, map1, map2, INTER_LINEAR);
        f = f2;
    }
    frame = f;
    return !frame.empty();
}

bool SingleCameraOpt::getNewFrame(Mat& frame, bool undistort, bool gray) {
    switch (mode) {
        case 1:
            // VideoCapture
            cam >> lastOriginFrame;
            break;
        case 2:
            // VideoCaptureLoopback
//            *cam2 >> lastOriginFrame;
            cam2->read_frame(lastOriginFrame);
            break;
        case 3:
            // 图片序列文件
            string imgID = to_string(++lastFileIndex);
            lastOriginFrame = imread(filePath + string(fileNameLength - imgID.length(), '0') + imgID + ".jpg");
            if (lastOriginFrame.empty()) {
                // 已读到最后一张图片，回到第一张
                lastFileIndex = firstFileIndex - 1;
                imgID = to_string(++lastFileIndex);
                lastOriginFrame = imread(filePath + string(fileNameLength - imgID.length(), '0') + imgID + ".jpg");
            }
            break;
    }
    return getLastFrame(frame, undistort, gray);
}

bool SingleCameraOpt::isOpened() const {
    return open;
}

const string &SingleCameraOpt::getPath() const {
    return filePath;
}

SingleCameraOpt::~SingleCameraOpt() {
    delete cam2;
}

SingleCameraOpt::SingleCameraOpt() = default;
