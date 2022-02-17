/*
 * 根据已采集的图片集，计算单目相机的参数和失真系数，并保存至xml文件
 * 在运行之前，请正确设置 CAMERA_NAME、IMG_NUM
 */
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

const String CAMERA_NAME = "camera2";
const int IMG_NUM = 60;

const char ESC_KEY = 27;

// 将相机参数保存为xml文件
void saveRes(const Mat& cameraMatrix,const Mat& distCoeffs) {
    FileStorage fs("../" + CAMERA_NAME + ".xml", FileStorage::WRITE);
    if( fs.isOpened() ){
        fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";
}

int main() {
    // 设置棋盘格数量
    Size boardSize(11, 8);

    // 设置单个棋盘方格的尺寸
    // todo 如何理解单位？
    float squareSize = 1;

    // 最左侧角点到最右侧角点的水平距离
    float grid_width = squareSize * (float)(boardSize.width - 1);

    Mat frame, frameGray;
    std::vector<Point2f> corners;
    Size imageSize;
    bool patternFound;
    int imgNum = 0;

    // 标定图片的角点列表
    std::vector<std::vector<Point2f> > imagePoints;

    while (imgNum < IMG_NUM) {
        frame = imread("../imgs/" + CAMERA_NAME + "/" + to_string(++imgNum) + ".jpg", IMREAD_COLOR);
        if (frame.empty()) {
            std::cout << "open file error!" << std::endl;
            continue;
        }

        cvtColor(frame, frameGray, COLOR_BGR2GRAY);
        patternFound = findChessboardCorners(frameGray,
                                             boardSize,
                                             corners,
                                             CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK
                                             );

        if (patternFound) {
            cornerSubPix(frameGray,
                         corners,
                         Size(11, 11),
                         Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001)
            );

            imagePoints.push_back(corners);
        } else {
            cout << "no calibration plate found in " + to_string(imgNum) + ".jpg !" << endl;
        }
    }
    imageSize = frame.size();

    if (imagePoints.empty()) {
        cout << "no calibration plate found in all images!" << endl;
        return 0;
    }

    cout << "Number of effective images = " << imagePoints.size() << endl;

    // 创建与标定图片数量相同的 标定板的点列表
    vector<vector<Point3f> > objectPoints(1);
    objectPoints[0].clear();
    for( int i = 0; i < boardSize.height; ++i )
        for (int j = 0; j < boardSize.width; ++j)
            objectPoints[0].push_back(Point3f(float(j) * squareSize, float(i) * squareSize, 0));

    objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    // 相机矩阵、失真系数矩阵
    Mat cameraMatrix, distCoeffs;
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    distCoeffs = Mat::zeros(8, 1, CV_64F);

    // 每个图片的旋转向量、平移向量
    std::vector<Mat> rvecs, tvecs;

    std::vector<Point3f> newObjPoints;
    newObjPoints = objectPoints[0];

    // 计算相机矩阵、失真系数矩阵
    calibrateCameraRO(objectPoints, imagePoints, imageSize, -1, cameraMatrix, distCoeffs, rvecs, tvecs,
                      newObjPoints, 0);

    cout << endl << "cameraMatrix" << endl << cameraMatrix << endl;
    cout << endl << "distCoeffs" << endl << distCoeffs << endl;

    // 显示矫正后的标定图片
    Mat map1, map2;

    initUndistortRectifyMap(
                cameraMatrix, distCoeffs, Mat(),
                cameraMatrix, imageSize,
                CV_16SC2, map1, map2);

//    initUndistortRectifyMap(
//                cameraMatrix, distCoeffs, Mat(),
//                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize, 0), imageSize,
//                CV_16SC2, map1, map2);

    Mat frameAfter, frameAfter2;
    remap(frame, frameAfter, map1, map2, INTER_LINEAR);
    imshow("before", frame);
    imshow("after", frameAfter);

    // 等待用户操作
    int key = waitKey(-1);
    switch (key) {
        case ESC_KEY:
            // 退出程序，不保存结果
            return 0;
        case 's':
            // 保存结果
            saveRes(cameraMatrix, distCoeffs);
            break;
        default:
            break;
    }

    return 0;
}