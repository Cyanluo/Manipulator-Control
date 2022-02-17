#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "include/SingleCameraOpt.h"
#include "include/DoubleCameraOpt.h"
#include <thread>

using namespace std;
using namespace cv;

string CAMERA_NAME[2] = {"camera1", "camera2"};
string path = "../capture_data/record2/camera1&camera2/";

int main(){
    DoubleCameraOpt stereoCom(path, CAMERA_NAME);
    stereoCom.autoCalcDisp = true;

    Mat frame[2], disp[2], rect;
    int key;

    while (true) {
        stereoCom.getNewFrame(frame, false);
        stereoCom.getLastDispShow(disp);
        stereoCom.getRectifiedView(rect);

        imshow("dispLeft", disp[0]);
        imshow("dispRight", disp[1]);
        imshow("show", rect);
//        imshow("left", frame[0]);
//        imshow("right", frame[1]);

        key = waitKey(1);
        if (key == 27) {
            break;
        }
    }

    return 0;
}