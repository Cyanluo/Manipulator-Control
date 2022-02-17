/*
 * �����Ѳɼ���ͼƬ�������㵥Ŀ����Ĳ�����ʧ��ϵ������������xml�ļ�
 * ������֮ǰ������ȷ���� CAMERA_NAME��IMG_NUM
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

// �������������Ϊxml�ļ�
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
    // �������̸�����
    Size boardSize(11, 8);

    // ���õ������̷���ĳߴ�
    // todo �����ⵥλ��
    float squareSize = 1;

    // �����ǵ㵽���Ҳ�ǵ��ˮƽ����
    float grid_width = squareSize * (float)(boardSize.width - 1);

    Mat frame, frameGray;
    std::vector<Point2f> corners;
    Size imageSize;
    bool patternFound;
    int imgNum = 0;

    // �궨ͼƬ�Ľǵ��б�
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

    // ������궨ͼƬ������ͬ�� �궨��ĵ��б�
    vector<vector<Point3f> > objectPoints(1);
    objectPoints[0].clear();
    for( int i = 0; i < boardSize.height; ++i )
        for (int j = 0; j < boardSize.width; ++j)
            objectPoints[0].push_back(Point3f(float(j) * squareSize, float(i) * squareSize, 0));

    objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    // �������ʧ��ϵ������
    Mat cameraMatrix, distCoeffs;
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    distCoeffs = Mat::zeros(8, 1, CV_64F);

    // ÿ��ͼƬ����ת������ƽ������
    std::vector<Mat> rvecs, tvecs;

    std::vector<Point3f> newObjPoints;
    newObjPoints = objectPoints[0];

    // �����������ʧ��ϵ������
    calibrateCameraRO(objectPoints, imagePoints, imageSize, -1, cameraMatrix, distCoeffs, rvecs, tvecs,
                      newObjPoints, 0);

    cout << endl << "cameraMatrix" << endl << cameraMatrix << endl;
    cout << endl << "distCoeffs" << endl << distCoeffs << endl;

    // ��ʾ������ı궨ͼƬ
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

    // �ȴ��û�����
    int key = waitKey(-1);
    switch (key) {
        case ESC_KEY:
            // �˳����򣬲�������
            return 0;
        case 's':
            // ������
            saveRes(cameraMatrix, distCoeffs);
            break;
        default:
            break;
    }

    return 0;
}