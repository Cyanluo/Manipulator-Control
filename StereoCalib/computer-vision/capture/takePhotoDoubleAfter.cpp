/*
 * 打开两个摄像头采集图像，当均识别到棋盘格时，按'c'将当前两帧保存到相应文件夹中
 * 在运行之前，请正确设置 CAMERA_NAME_LIST、CAMERA_INDEX_LIST、boardSize、imgSize
 */

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include "../include/SingleCameraOpt.h"

using namespace std;
using namespace cv;

const String CAMERA_NAME_LIST[2] = {"camera1", "camera2"};
const int CAMERA_INDEX_LIST[2] = {0, 1};
Size boardSize(11, 8);
Size imgSize(1280, 720);

const char ESC_KEY = 27;

int main(){
    // 初始化相机
    vector<SingleCameraOpt> capture_list(2);
    for (int i = 0; i < 2; ++i) {
        capture_list[i] = SingleCameraOpt(CAMERA_INDEX_LIST[i], CAMERA_NAME_LIST[i], imgSize);
        if (!capture_list[i].isOpened()) {
            cout << "open camera error: " << CAMERA_NAME_LIST[i];
            return 0;
        }
    }

    vector<Point2f> corners[2];
    bool patternFound[2];
    int imgNum = 0;
    Mat frame[2], frameGray[2], tframe[2];

    cout << endl << "----------start capture----------" << endl;

    while (true) {
        bool skip = false;
        for (int i = 0; i < 2; ++i) {
            capture_list[i].getNewFrame(frame[i], true, false);
            if (frame[i].empty()) {
                cout << "capture empty frame" << endl;
                skip = true;
                break;
            }
            tframe[i] = frame[i].clone();
            resize(tframe[i], tframe[i], Size(640, 360), 0, 0, INTER_LINEAR);
            cvtColor(tframe[i], frameGray[i], COLOR_BGR2GRAY);
            patternFound[i] = findChessboardCorners(frameGray[i],
                                                 boardSize,
                                                 corners[i],
                                                 CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK
            );
            drawChessboardCorners(tframe[i], boardSize, Mat(corners[i]), patternFound[i]);
            imshow(CAMERA_NAME_LIST[i], tframe[i]);
        }

        if (skip) {
            continue;
        }

        int state;
        int key = waitKey(1);
        switch (key) {
            case ESC_KEY:
                // 退出采集模式
                state = -1;
                break;

            case 'c':
                if (patternFound[0] && patternFound[1]) {
                    // 当前两帧都识别到标定板，保存
                    imgNum++;
                    for (int i = 0; i < 2; ++i) {
                        // 分别保存两张照片
                        imwrite("../capture_data/imgs/" + CAMERA_NAME_LIST[0] + "&" + CAMERA_NAME_LIST[1] + "_undistort/" +
                                CAMERA_NAME_LIST[i] + "_" + to_string(imgNum) + ".jpg", frame[i]);
                    }
                    cout << "Save image. imgNum=" << imgNum << endl;
                } else {
                    cout << "no calibration plate found!" << endl;
                }
                break;

            default:
                break;
        }

        if (state == -1) {
            break;
        }
    }

    cout << "----------finish capture----------" << endl << imgNum << " images were captured totally";
    return 0;
}