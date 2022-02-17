#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

int IMG_NUM = 19;

const String CAMERA_NAME_LIST[2] = {"camera1", "camera2"};

const Size boardSize = Size(11, 8);

const bool fixIntrinsic = false;

//标定板黑白格子的大小(mm)
const float squareSize = 15;

const char ESC_KEY = 27;

void saveRes(const Mat cameraMatrix[2], const Mat distCoeffs[2], const Mat R1[2], const Mat P1[2], const Rect validRoi[2], const Mat& Q) {
    FileStorage fs("../" + CAMERA_NAME_LIST[0] + "&" + CAMERA_NAME_LIST[1] + ".xml", FileStorage::WRITE);
    if( fs.isOpened() ){
        for (int i = 0; i < 2; ++i) {
            fs << CAMERA_NAME_LIST[i] + "_cameraMatrix" << cameraMatrix[i]
               << CAMERA_NAME_LIST[i] + "_distCoeffs" << distCoeffs[i]
               << CAMERA_NAME_LIST[i] + "_R1" << R1[i]
               << CAMERA_NAME_LIST[i] + "_P1" << P1[i]
               << CAMERA_NAME_LIST[i] + "_validRoi" << validRoi[i];
        }
        fs << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not write the file.\n";
}

void showRes(Mat frameAfter[2], Rect validRoi[2], Size imageSize){
    /*
    把校正结果显示出来
    把左右两幅图像显示到同一个画面上
    这里只显示了最后一副图像的校正结果。并没有把所有的图像都显示出来
    */
    Mat canvas;
    double sf;
    int w, h;
    sf = 600. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);
    /*左图像画到画布上*/
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分
    resize(frameAfter[0], canvasPart, canvasPart.size(), 0, 0, INTER_AREA);        //把图像缩放到跟canvasPart一样大小
    Rect vroiL(cvRound(validRoi[0].x * sf), cvRound(validRoi[0].y * sf),                //获得被截取的区域
               cvRound(validRoi[0].width * sf), cvRound(validRoi[0].height * sf));
    rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形
    cout << "Painted ImageL" << endl;
    /*右图像画到画布上*/
    canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分
    resize(frameAfter[1], canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
    Rect vroiR(cvRound(validRoi[1].x * sf), cvRound(validRoi[1].y * sf),
               cvRound(validRoi[1].width * sf), cvRound(validRoi[1].height * sf));
    rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);
    cout << "Painted ImageR" << endl;
    /*画上对应的线条*/
    for (int i = 0; i < canvas.rows; i += 16)
        line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
    imshow("rectified", canvas);
}

int main(){
    String fileDir;
    if (fixIntrinsic) {
        // 读取没有去畸变的原始图像
        fileDir = "../imgs/" + CAMERA_NAME_LIST[0] + "&" + CAMERA_NAME_LIST[1] + "_original/";
    } else {
        fileDir = "../imgs/" + CAMERA_NAME_LIST[0] + "&" + CAMERA_NAME_LIST[1] + "_undistort/";
    }

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;
    Mat frame[2], frameGray[2];

    for (int i = 1; i <= IMG_NUM; ++i) {
        bool findBoth = true;

        // 临时存放两张图片的角点
        vector<Point2f> cornersTemp[2];

        for (int j = 0; j < 2; ++j) {
            String imgName = fileDir + CAMERA_NAME_LIST[j] + "_" + to_string(i) + ".jpg";

            frame[j] = imread(imgName);
            if (frame[j].empty()) {
                cout << "Can't open: " + imgName << "!" << endl;
            }
            cvtColor(frame[j], frameGray[j], COLOR_BGR2GRAY);

            if (!findChessboardCorners(frameGray[j], boardSize, cornersTemp[j],
                                       CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE)) {
                // 当前图片没有找到角点
                cout << "Can't find pattern: " + imgName << "!" << endl;
                findBoth = false;
                break;
            }

            cornerSubPix(frameGray[j],
                         cornersTemp[j],
                         Size(11, 11),
                         Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001)
            );
        }

        if (findBoth) {
            // 左右两幅图片的角点都找到
            for (int j = 0; j < 2; ++j) {
                imagePoints[j].push_back(cornersTemp[j]);
            }
        }
    }

    // 检查找到的角点列表
    int imgPairNum = imagePoints[0].size();
    if (imgPairNum==0) {
        cout << "no patterns found!" << endl;
        return 0;
    } else {
        cout << imgPairNum << " pairs have been successfully detected." << endl;
    }

    imageSize = frame[0].size();

    // 计算世界坐标系中的角点列表
    objectPoints.resize(imgPairNum);
    for(int i = 0; i < imgPairNum; i++ ){
        for(int j = 0; j < boardSize.height; j++ )
            for(int k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(float(k)*squareSize, float(j)*squareSize, 0));
    }

    // 初始化相机矩阵
    Mat cameraMatrix[2], distCoeffs[2];
    int flags;
    if (fixIntrinsic) {
        // 从文件读取相机矩阵和失真系数矩阵
        for (int i = 0; i < 2; ++i) {
            FileStorage fs("../" + CAMERA_NAME_LIST[i] + ".xml", FileStorage::READ);
            if (fs.isOpened()) {
                // 成功打开文件，开始读数据
                fs["cameraMatrix"] >> cameraMatrix[i];
                fs["distCoeffs"] >> distCoeffs[i];
            } else {
                cout << "Error: can't open file \"" << "../" + CAMERA_NAME_LIST[i] + ".xml" << "\"" << endl;
            }
            cout << cameraMatrix[i] << endl << distCoeffs[i] << endl << endl;
        }


        // 设置flag
        flags = CALIB_USE_INTRINSIC_GUESS;

//        flags = CALIB_FIX_INTRINSIC +
//                CALIB_FIX_ASPECT_RATIO +
//                CALIB_ZERO_TANGENT_DIST +
//                CALIB_USE_INTRINSIC_GUESS +
//                CALIB_RATIONAL_MODEL +
//                CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5;
    } else {
        cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
        cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);
        flags =
//                CALIB_FIX_ASPECT_RATIO +
//                CALIB_ZERO_TANGENT_DIST +
                CALIB_USE_INTRINSIC_GUESS +
//                CALIB_SAME_FOCAL_LENGTH +
                CALIB_RATIONAL_MODEL +
                CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5;
    }

    Mat R, T, E, F;
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T, E, F, flags,
                                 TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
    cout << "done with RMS error=" << rms << endl;
    cout << "T=" << T << endl;

    Mat R1[2], P1[2], Q;
    Rect validRoi[2];
    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1[0], R1[1], P1[0], P1[1], Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    Mat rmap[2][2];
    Mat frameAfter[2];
    for (int i = 0; i < 2; ++i) {
        initUndistortRectifyMap(cameraMatrix[i], distCoeffs[i], R1[i], P1[i],
                                imageSize, CV_16SC2, rmap[i][0], rmap[i][1]);
        remap(frame[i], frameAfter[i], rmap[i][0], rmap[i][1], INTER_LINEAR);
        imshow(CAMERA_NAME_LIST[i], frameAfter[i]);
    }

    showRes(frameAfter, validRoi, imageSize);

    // 等待用户操作
    int key = waitKey(-1);
    switch (key) {
        case ESC_KEY:
            // 退出程序，不保存结果
            return 0;
        case 's':
            // 保存结果
            saveRes(cameraMatrix, distCoeffs, R1, P1, validRoi, Q);
            break;
        default:
            break;
    }

    return 0;
}