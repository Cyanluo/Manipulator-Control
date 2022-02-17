/*
 * 打开单个摄像头采集图像，当识别到棋盘格时，按'c'将当前帧保存到相应文件夹中
 * 在运行之前，请正确设置 CAMERA_NAME、CAMERA_INDEX、boardSize、imgSize
 */

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

using namespace std;
using namespace cv;

const String CAMERA_NAME = "camera2";
const int CAMERA_INDEX = 2;
Size boardSize(11, 8);
Size imgSize(1280, 720);
int startIndex = 59;

const char ESC_KEY = 27;

int main() {
    VideoCapture capture(CAMERA_INDEX);

    if (!capture.isOpened()) {
        cout << "open camera error!" << endl;
        return -1;
    }

    capture.set(CAP_PROP_FRAME_WIDTH, imgSize.width);
    capture.set(CAP_PROP_FRAME_HEIGHT, imgSize.height);

    vector<Point2f> corners;
    bool patternFound;
    int imgNum = 0;
    Mat frame, frameGray, tframe;

    cout << endl << "----------start capture----------" << endl;

    while (true) {
        capture >> frame;
        if (frame.empty()) {
            cout << "capture empty frame" << endl;
            continue;
        }
        tframe = frame.clone();
        cvtColor(frame, frameGray, COLOR_BGR2GRAY);

        patternFound = findChessboardCorners(frameGray,
                                             boardSize,
                                             corners,
                                             CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK
        );
        drawChessboardCorners(tframe, boardSize, Mat(corners), patternFound);
        imshow(CAMERA_NAME, tframe);

        int state;
        int key = waitKey(1);
        switch (key) {
            case ESC_KEY:
                // 退出采集模式
                state = -1;
                break;
            case 'c':
                // 如果当前帧识别到标定板，则保存当前帧作为标定图片
                if (patternFound) {
                    imgNum++;
                    imwrite("../capture_data/imgs/" + CAMERA_NAME + "/" + to_string(imgNum + startIndex) + ".jpg", frame);
                    cout << "Save image. totImgNum=" << imgNum + startIndex << endl;
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