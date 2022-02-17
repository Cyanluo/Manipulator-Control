#ifndef COMPUTERVISION_DOUBLECAMERAOPT_H
#define COMPUTERVISION_DOUBLECAMERAOPT_H

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/cudastereo.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <thread>
#include <future>
#include "SingleCameraOpt.h"

using namespace cv;
using namespace std;

class DoubleCameraOpt {
private:
    /// 用于构造函数中复用的初始化摄像头函数
    void cameraGeneralInit(const string camName[2], Size frameSize, const int virCamDispIndex_[2], int virCamSpliceIndex_);

    bool usingCamPath;

public:
    int CAMERA_INDEX[2]{};
    string CAMERA_PATH[2];
    string CAMERA_NAME[2];
    string groupName;
    SingleCameraOpt* singleCamera[2]{};
    Size frameSize;

    // 打印函数getLastDispFrame的耗时
    bool printTime = false;

    // 自动计算深度信息。设置为true后，每次getNewFrame就自动更新视差图；否则需要手动更新
    bool autoCalcDisp = false;

    // 自动计算拼接图
    bool autoCalcSplice = false;

    // 启用GPU加速立体匹配
    bool GPU = false;

    // 启用视差图空洞填充
    bool insertDisp = false;

    /// 从相机读取图像的构造函数
    /// \param camIndex 摄像头索引数组
    /// \param camName 摄像头名称数组
    /// \param frameSize 帧分辨率
    DoubleCameraOpt(const int camIndex[2], const string camName[2], Size frameSize);

    /// 利用摄像头id从摄像头读取图像，并创建每个摄像头对应的原始画面、矫正画面、视差图画面的虚拟摄像头
    /// \param camIndex 同上
    /// \param camName 同上
    /// \param frameSize 同上
    /// \param virCamIndex 虚拟原始摄像头id数组，-1表示不创建该虚拟摄像头
    /// \param virCamDispIndex 虚拟深度摄像头id数组，-1表示不创建该虚拟摄像头
    /// \param virCamSpliceIndex 虚拟拼接摄像头id，-1表示不创建该虚拟摄像头
    DoubleCameraOpt(const int camIndex[2], const string camName[2], Size frameSize,
                    const int virCamIndex[2], const int virCamDispIndex[2], int virCamSpliceIndex);

    /// 利用摄像头路径从摄像头读取图像，并创建每个摄像头对应的原始画面、矫正画面、视差图画面的虚拟摄像头
    /// \param camIndex 同上
    /// \param camName 同上
    /// \param frameSize 同上
    /// \param virCamIndex 同上
    /// \param virCamDispIndex 同上
    /// \param virCamSpliceIndex 同上
    DoubleCameraOpt(const string camPath[2], const string camName[2], Size frameSize,
                    const int virCamIndex[2], const int virCamDispIndex[2], int virCamSpliceIndex);

    /// 从文件读取图像的构造函数
    /// \param path 双目图片路径，如："../capture_data/record2/camera1&camera2/"
    /// \param fileNameLen 图片文件名长度
    /// \param camName 同上
    /// \param frameSize 同上
    DoubleCameraOpt(const string& path, int fileNameLen, const string camName[2], Size frameSize);

    /// 从文件读取图像，并创建虚拟摄像头的构造函数
    /// \param path 双目图片路径，如："../capture_data/record2/camera1&camera2/"
    /// \param fileNameLen 图片文件名长度
    /// \param camName 同上
    /// \param frameSize 同上
    /// \param virCamDispIndex 同上
    /// \param virCamSpliceIndex 同上
    DoubleCameraOpt(const string& path, int fileNameLen, const string camName[2], Size frameSize,
                    const int virCamDispIndex[2], int virCamSpliceIndex);

    ~DoubleCameraOpt();

    bool convertFrame(const Mat frameIn[2], Mat frameOut[2]);
    bool getNewFrame(Mat frame[2], bool gray);
    bool getLastFrame(Mat frame[2], bool gray);

    // 计算视差图并更新lastDisp。autoCalcDisp=true，则自动更新；否则在getNewFrame后，需要手动执行更新
    void countDisp();

    // 获取双目矫正的预览图片Mat
    bool getRectifiedView(Mat &rec);

    // 获取上一帧左、右视差图的展示图片
    bool getLastDispShow(Mat *frameOut);

    // 计算矩形area在左或右视差图（由参数dispIndex指定）的平均距离
    double getAreaDistance(const Rect &area, int frameIndex);

    // 双目拼接
    void countSplice();
    void getSpliceFrame(Mat &frameOut);

    bool isOpened() const;

    const string &getFilePath() const;
    void setFilePath(const string &filePath);

private:
    // 是否写入虚拟摄像头
    bool writeVirCam = false;

    // 深度图虚拟摄像头
    VirtualCamera virCamDisp[2];
    int virCamDispIndex[2];

    // 拼接图虚拟摄像头
    VirtualCamera virCamSplice;
    int virCamSpliceIndex;

    Size SingleCameraFrameSize;
    bool open = false;
    string filePath;

    // 是否从文件读取图像
    bool readFromPath = false;

    // 上一次获取的帧、视差图、点云、拼接图，其尺寸始终为frameSize
    Mat lastFrame[2], lastDisp[2], lastDispShow[2], lastPointCloud[2], lastDepth[2], lastSplice;

    // SGBM立体匹配实例（CPU计算时使用）
    Ptr<StereoSGBM> sgbm[2];

    // SGM立体匹配实例（GPU计算时使用）
    Ptr<cuda::StereoSGM> sgm[2];

    // SGBM使用的参数，后期考虑去除
    int nmDisparities = 128;

    // 相机的内参和外参，从文件读取
    Mat cameraMatrix[2], distCoeffs[2], R1[2], P1[2];

    // 画面可用区域
    Rect validRoi[2];

    // 双目矫正时用到的map，通过相机参数计算
    Mat rmap[2][2];

    // 从视差图转为坐标时使用，通过文件读取
    Mat Q;

    // 双目拼接时，左、右画面裁切列的位置（比例）
    double leftCutPos = 0.5, rightCutPos = 0.49;

    // 双目拼接时，左、右画面的裁切列的索引（左不保留该列，右保留该列）
    int leftCutIndex, rightCutIndex;

    // 双目拼接后的图片宽度
    int spliceWidth;

    void init(const string *camName);
    bool readXML();
    void initSGBM(const Mat& frame);

    // 生成左右视差图，通过子线程调用
    void leftStereoMatch();
    void rightStereoMatch();

    // Mat转Vector
    template<typename Tp>
    vector<Tp> convertMat2Vector(const Mat &mat){
        Mat t;
        mat.copyTo(t);
        // 通道数不变，按行转为一行
	    return (vector<Tp>)(t.reshape(0, 1));
    }

    // 视差图空洞填充
    static void insertDepth32f(Mat &depth);
};

#endif //COMPUTERVISION_DOUBLECAMERAOPT_H