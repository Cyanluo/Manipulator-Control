
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <numeric>
#include <functional>
#include <fstream>
#include <numeric>
#include <math.h>
#include <iomanip>
using namespace std;
using namespace cv;

int main()
{
    /***************************open Camera**************************/
    Mat img_left, img_right;
    cv::VideoCapture camera(2);
    if (!camera.isOpened()) return 1;
    camera.set(CAP_PROP_FOURCC, CAP_OPENCV_MJPEG);//设置为MJPG格式
    camera.set(CAP_PROP_FRAME_WIDTH, 1280);
    camera.set(CAP_PROP_FRAME_HEIGHT, 480);
    Size ImgSize(640, 480);
    cout << "CV_CAP_PROP_FPS: " << camera.get(CAP_PROP_FPS) << endl;

    /**************************read Parameter*********************************/
    FileStorage fs("../intrinsics.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        printf("Failed to open file %s\n", "intrinsics.yml");
        return -1;
    }
    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;
    fs.open("../extrinsics.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        printf("Failed to open file %s\n", "extrinsics.yml");
        return -1;
    }
    Mat R, T, R1, P1, R2, P2, Q;
    fs["R"] >> R;
    fs["T"] >> T;

    /***************************calculate Rectify Matrix**********************************/
    stereoRectify(M1, D1, M2, D2, ImgSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, ImgSize);
    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(M1, D1, R1, P1, ImgSize, CV_16SC2, map11, map12);//立体校正，即进行畸变校正和极线校正，使得y方向没有视差
    initUndistortRectifyMap(M2, D2, R2, P2, ImgSize, CV_16SC2, map21, map22);



    char c = ' ';
    while (true)
    {
        cv::Mat frame;
        camera >> frame;
        cv::Mat frame_left,frame_right;
        cv::Mat Left_remap, Right_remap;
        frame_left = frame(Rect(0, 0, 640, 480));
        frame_right = frame(Rect(640, 0, 640, 480));
        remap(frame_left, Left_remap, map11, map12, INTER_LINEAR);
        remap(frame_right, Right_remap, map21, map22, INTER_LINEAR);
        cv::imshow("frame_left", frame_left);
        cv::imshow("frame_right", frame_right);
        cv::imshow("Left_remap", Left_remap);
        cv::imshow("Right_remap", Right_remap);
        c = waitKey(1);
        if ('q' == char(c)) break;
    }
    return 0;
}
