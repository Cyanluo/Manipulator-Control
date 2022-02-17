#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "include/SingleCameraOpt.h"
#include "include/DoubleCameraOpt.h"
#include <thread>

using namespace std;
using namespace cv;

int CAMERA_INDEX[2] = {0, 2};
//string filePath = "/home/zixiao_bios/CLionProjects/detect-outside/res/capture_data/record2/camera1&camera2/";
string filePath = "/home/zixiao_bios/res/back2/camera1&camera3/";
string CAMERA_NAME[2] = {"camera1", "camera3"};
Size frameSize(640, 360);
int VIR_CAMERA_INDEX[2] = {10, 11};

Size clickSize(20, 20);

// 选取的矩形区域，确保不超出Mat范围
Rect area;

DoubleCameraOpt stereoCom(filePath, 7, CAMERA_NAME, frameSize);

void setRectangle(int x, int y){
    // 设置矩形四个边界
    int left = x - clickSize.width / 2;
    int top = y - clickSize.height / 2;
    int right = x + clickSize.width / 2;
    int bottom = y + clickSize.height / 2;

    // 处理边界
    if (left < 0) {
        left = 0;
    }
    if (left >= frameSize.width) {
        left = frameSize.width - 1;
    }
    if (top < 0) {
        top = 0;
    }
    if (top >= frameSize.height) {
        top = frameSize.height - 1;
    }

    if (right < 0) {
        right = 0;
    }
    if (right >= frameSize.width) {
        right = frameSize.width - 1;
    }
    if (bottom < 0) {
        bottom = 0;
    }
    if (bottom >= frameSize.height) {
        bottom = frameSize.height - 1;
    }

    // 创建矩形
    area = Rect(left, top, right - left, bottom - top);

//    cout << "lu" << leftUp << "rb" << rightBottom << endl;
}

void clickMouse(int event, int x, int y, int flags, void* p1){
    if (event == EVENT_LBUTTONUP){
        setRectangle(x, y);
        stereoCom.getAreaDistance(area, 0);
//        cout << stereoCom.getAreaDistance(area, 0) << endl;
    }
}

int main() {
    stereoCom.printTime = false;
    stereoCom.autoCalcDisp = false;
    stereoCom.GPU = true;
    stereoCom.insertDisp = false;

    Mat dispShow[2], m, frame[2], spliceFrame;

    // 鼠标监听
    namedWindow("dispL", WINDOW_AUTOSIZE);
    setMouseCallback("dispL", clickMouse, nullptr);

    while (true) {
        stereoCom.getNewFrame(frame, false);
//        stereoCom.getLastDispShow(dispShow);
        stereoCom.getRectifiedView(m);

        // 画选定的矩形
//        rectangle(dispShow[0], area.tl(), area.br(), Scalar(255, 0, 0), -1, LINE_8, 0);

//        imshow("dispL", dispShow[0]);
//        imshow("dispR", dispShow[1]);
//        imshow("left", frame[0]);
        imshow("m", m);

        // 画面拼接
        stereoCom.getSpliceFrame(spliceFrame);
        imshow("splice", spliceFrame);

        int key = waitKey(1);
        if (key == 27) {
            break;
        } else if (key == 'p') {
            waitKey(-1);
        }
    }

    return 0;
}