
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
static int print_help()
{
    cout <<
         " Given a list of chessboard images, the number of corners (nx, ny)\n"
         " on the chessboards, and a flag: useCalibrated for \n"
         "   calibrated (0) or\n"
         "   uncalibrated \n"
         "     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
         "         matrix separately) stereo. \n"
         " Calibrate the cameras and display the\n"
         " rectified results along with the computed disparity images.   \n" << endl;
    cout << "Usage:\n ./stereo_calib -w=<board_width default=9> -h=<board_height default=6> -s=<square_size default=1.0> <image list XML/YML file default=../data/stereo_calib.xml>\n" << endl;
    return 0;
}
/************相机标定程序***************/
static void
StereoCalib(const vector<string>& imagelist, Size boardSize, float squareSize, bool displayCorners = false, bool useCalibrated = true, bool showRectified = true)
{
    if (imagelist.size() % 2 != 0)		//判断图片是否成对
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    const int maxScale = 2;							//设定寻找角点的图像尺寸，若scale未找到，则将图像放大寻找角点
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > imagePoints[2];		//分别为左右图像的角点
    vector<vector<Point3f> > objectPoints;			//物体
    Size imageSize;									//

    int i, j, k, nimages = (int)imagelist.size() / 2;		//nimages为左或右图像的个数

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;

    for (i = j = 0; i < nimages; i++)					//依次寻找13对图片
    {
        for (k = 0; k < 2; k++)						//依次寻找左右图片
        {
            const string& filename = imagelist[i * 2 + k];
            Mat img = imread(filename, 0);				//载入灰度图 0代表灰度图
            if (img.empty())
                break;
            if (imageSize == Size())					//判断图像尺寸是否达到预先设置的要求
                imageSize = img.size();
            else if (img.size() != imageSize)
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                waitKey(10000);
                break;
            }
            bool found = false;
            //设置图像矩阵的引用(指针)，此时指向左右视图的矩阵首地址
            vector<Point2f>& corners = imagePoints[k][j];		//赋值角点个数及位置
            for (int scale = 1; scale <= maxScale; scale++)
            {
                Mat timg;
                if (scale == 1)
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale, INTER_LINEAR_EXACT);
                found = findChessboardCorners(timg, boardSize, corners,				//找角点函数，得到内角点坐标
                                              CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

                //				drawChessboardCorners(timg, boardSize, corners, found);				//画出角点，自己调试时使用

                if (found)
                {
                    if (scale > 1)
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1. / scale;
                    }
                    break;
                }
            }
            if (displayCorners)
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640. / MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf, INTER_LINEAR_EXACT);
                imshow("corners", cimg1);
                char c = (char)waitKey(500);
                if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if (!found)
                break;
            cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
                                      30, 0.01));
        }
        if (k == 2)
        {
            goodImageList.push_back(imagelist[i * 2]);
            goodImageList.push_back(imagelist[i * 2 + 1]);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";		//命令行打印检测到的图片
    nimages = j;													//nimages为左右图像个数
    if (nimages < 2)
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);			//左相机 角点位置
    imagePoints[1].resize(nimages);			//右相机 角点位置
    objectPoints.resize(nimages);

    for (i = 0; i < nimages; i++)
    {
        for (j = 0; j < boardSize.height; j++)
            for (k = 0; k < boardSize.width; k++)
                objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
    }

    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];			//相机参数  畸变矩阵
    cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);
    Mat R, T, E, F;		//R旋转矩阵 T平移矩阵 E本征矩阵 F输出基本矩阵
    //最关键的地方，求解校正后的相机参数
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T, E, F,
                                 CALIB_FIX_ASPECT_RATIO +
                                 CALIB_ZERO_TANGENT_DIST +
                                 CALIB_USE_INTRINSIC_GUESS +
                                 CALIB_SAME_FOCAL_LENGTH +
                                 CALIB_RATIONAL_MODEL +
                                 CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                                 TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
    cout << "done with RMS error=" << rms << endl;

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;								//计算投影标定误差
    int npoints = 0;
    vector<Vec3f> lines[2];						//极线
    for (i = 0; i < nimages; i++)				//水平校正
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for (k = 0; k < 2; k++)
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
        }
        for (j = 0; j < npt; j++)
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average epipolar err = " << err / npoints << endl;

    // save intrinsic parameters
    FileStorage fs("intrinsics.yml", FileStorage::WRITE);		//存储内参
    if (fs.isOpened())
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
           "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    //外参数
    // R--右相机相对左相机的旋转矩阵
    // T--右相机相对左相机的平移矩阵
    // R1,R2--左右相机校准变换（旋转）矩阵  3×3
    // P1,P2--左右相机在校准后坐标系中的投影矩阵 3×4
    // Q--视差-深度映射矩阵，我利用它来计算单个目标点的三维坐标

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];		//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
    //进行立体校正
    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    /*
    * R T 左相机到右相机的旋转平移矩阵  R 3*3      T  3*1  T中第一个Tx为 基线长度
    立体校正的时候需要两幅图像共面并且行对准 以使得立体匹配更加的可靠
    使得两幅图像共面的方法就是把两个摄像头的图像投影到一个公共成像面上，这样每幅图像从本图像平面投影到公共图像平面都需要一个旋转矩阵R
    stereoRectify 这个函数计算的就是从图像平面投影都公共成像平面的旋转矩阵Rl,Rr。 Rl,Rr即为左右相机平面行对准的校正旋转矩阵。
    左相机经过Rl旋转，右相机经过Rr旋转之后，两幅图像就已经共面并且行对准了。
    其中Pl,Pr为两个相机的投影矩阵，其作用是将3D点的坐标转换到图像的2D点的坐标:P*[X Y Z 1]' =[x y w]
    Q矩阵为重投影矩阵，即矩阵Q可以把2维平面(图像平面)上的点投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的视差
    */

    fs.open("extrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the extrinsic parameters\n";
    //opencv可以辨认相机的物理位置
    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

    // COMPUTE AND DISPLAY RECTIFICATION	计算并显示校正信息
    if (!showRectified)
        return;

    Mat rmap[2][2];
    // IF BY CALIBRATED (BOUGUET'S METHOD)
    if (useCalibrated)
    {
        // we already computed everything
    }
        // OR ELSE HARTLEY'S METHOD
    else
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for (k = 0; k < 2; k++)
        {
            for (i = 0; i < nimages; i++)
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    /*
    根据stereoRectify 计算出来的R 和 P 来计算图像的映射表 mapx,mapy
    mapx,mapy这两个映射表接下来可以给remap()函数调用，来校正图像，使得两幅图像共面并且行对准
    ininUndistortRectifyMap()的参数newCameraMatrix就是校正后的摄像机矩阵。在openCV里面，校正后的计算机矩阵Mrect是跟投影矩阵P一起返回的。
    所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
    */

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    /*
    显示校正结果
    把左右两幅图像显示到同一个画面上
    这里只显示了最后一副图像的校正结果。并没有把所有的图像都显示出来
    */

    Mat canvas;
    double sf;
    int w, h;
    if (!isVerticalStereo)
    {
        sf = 600. / MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w * 2, CV_8UC3);
    }
    else
    {
        sf = 300. / MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h * 2, w, CV_8UC3);
    }

    for (i = 0; i < nimages; i++)
    {
        for (k = 0; k < 2; k++)
        {
            Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);		//对准左右图像，
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            if (useCalibrated)
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
            }
        }

        if (!isVerticalStereo)		 //画上对应的线条
            for (j = 0; j < canvas.rows; j += 16)
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);		 //画平行线
        else
            for (j = 0; j < canvas.cols; j += 16)
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);				//输出立体校正后的图片
        char c = (char)waitKey();
        if (c == 27 || c == 'q' || c == 'Q')
            break;
    }
}
static bool readStringList(const string& filename, vector<string>& l)
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
        l.push_back((string)*it);
    return true;
}
static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for (int y = 0; y < mat.rows; y++)
    {
        for (int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f  %f  %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}
int main(int argc, char** argv)
{
    string path_intrinsics("../intrinsics.yml"),
            path_extrinsics("../extrinsics.yml");
    Size ImgSize(640, 480);
    string point_cloud_filename = "point_11.txt";
    Mat imgL = imread("../pic/left/3.jpg"), imgR = imread("../pic/right/3.jpg");//24不行，23右图70二值化
    // 读取相机内外参数
    FileStorage fs(path_intrinsics, FileStorage::READ);
    if (!fs.isOpened()) {
        printf("Failed to open file %s\n", "intrinsics.yml");
        return -1;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    fs.open(path_extrinsics, FileStorage::READ);
    if (!fs.isOpened()) {
        printf("Failed to open file %s\n", "extrinsics.yml");
        return -1;
    }

    Mat R, T, R1, P1, R2, P2, Q;
    fs["R"] >> R;
    fs["T"] >> T;
    //图像矫正
    /*
    1 cameraMatrix1-第一个摄像机的摄像机矩阵
    2 distCoeffs1-第一个摄像机的畸变向量
    3 cameraMatrix2-第二个摄像机的摄像机矩阵
    4 distCoeffs1-第二个摄像机的畸变向量
    5 imageSize-图像大小
    6 R- stereoCalibrate() 求得的R矩阵
    7 T- stereoCalibrate() 求得的T矩阵
    8 R1-输出矩阵，第一个摄像机的校正变换矩阵（旋转变换）
    9 R2-输出矩阵，第二个摄像机的校正变换矩阵（旋转矩阵）
    10 P1-输出矩阵，第一个摄像机在新坐标系下的投影矩阵
    11 P2-输出矩阵，第二个摄像机在想坐标系下的投影矩阵
    12 Q-4*4的深度差异映射矩阵
    13 flags-可选的标志有两种零或者 CV_CALIB_ZERO_DISPARITY ,如果设置 CV_CALIB_ZERO_DISPARITY 的话，该函数会让两幅校正后的图像的主点有相同的像素坐标。否则该函数会水平或垂直的移动图像，以使得其有用的范围最大
    14 alpha-拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
    15 newImageSize-校正后的图像分辨率，默认为原分辨率大小。
    16 validPixROI1-可选的输出参数，Rect型数据。其内部的所有像素都有效
    17 validPixROI2-可选的输出参数，Rect型数据。其内部的所有像素都有效
     */
    stereoRectify(M1, D1, M2, D2, ImgSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, ImgSize);
    Mat map11, map12, map21, map22;

    initUndistortRectifyMap(M1, D1, R1, P1, ImgSize, CV_16SC2, map11, map12);//立体校正，即进行畸变校正和极线校正，使得y方向没有视差
    initUndistortRectifyMap(M2, D2, R2, P2, ImgSize, CV_16SC2, map21, map22);
    //Mat t;
    //initUndistortRectifyMap(M1, D1, t, M1, img_size, CV_32FC1, map11, map12);//只进行畸变校正，不进行极限校正
    //initUndistortRectifyMap(M2, D2, t, M2, img_size, CV_32FC1, map21, map22);
    Mat Left_remap, Right_remap;
    remap(imgL, Left_remap, map11, map12, INTER_LINEAR);
    remap(imgR, Right_remap, map21, map22, INTER_LINEAR);

    //画出进行极线校正后的效果图，方便极限校正查看效果
    if (1) {
        Mat canvas;
        double sf;
        int w, h;
        sf = 1920. / MAX(ImgSize.width, ImgSize.height);
        w = cvRound(ImgSize.width * sf);
        h = cvRound(ImgSize.height * sf);
        canvas.create(h, w * 2, CV_8UC3);
        Mat canvasPart = canvas(Rect(0, 0, w, h));
        resize(Left_remap, canvasPart, canvasPart.size(), 0, 0,  INTER_AREA);

        canvasPart = canvas(Rect(w, 0, w, h));
        resize(Right_remap, canvasPart, canvasPart.size(), 0, 0,  INTER_AREA);
        for (int j = 0; j < canvas.rows; j += 40) {
            line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(255, 0, 0), 2, 8);
        }
        namedWindow("img", WINDOW_NORMAL);
        imshow("img", canvas);
        waitKey(1);
        destroyWindow("img");
    }

    vector<Point2f> points_L;
    vector<Point2f> points_R;

    //cvtColor(Left_remap, Left_remap,  BGR2GRAY);
    //cvtColor(Right_remap, Right_remap,  BGR2GRAY);
    //Right_remap = Right_remap > 70;
    //Left_remap = Left_remap > 70;
    findChessboardCorners(Left_remap, Size(11, 8), points_L,  CALIB_CB_ADAPTIVE_THRESH);
    //cornerSubPix(Left_remap, points_L, Size(11, 8), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER +  TERMCRIT_EPS, 30, 0.01));
    drawChessboardCorners(Left_remap, Size(11, 8), points_L, true);
    findChessboardCorners(Right_remap, Size(11, 8), points_R,  CALIB_CB_NORMALIZE_IMAGE |  CALIB_CB_ADAPTIVE_THRESH);
    //cornerSubPix(Right_remap, points_R, Size(11, 8), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
    drawChessboardCorners(Right_remap, Size(11, 8), points_R, true);
    namedWindow("Left",  WINDOW_NORMAL);
    imshow("Left", Left_remap);
    namedWindow("Right",  WINDOW_NORMAL);
    imshow("Right", Right_remap);
    waitKey(5);

    /****************************************
    *--------Step3.获取点云数据-------------*
    ****************************************/

    if (1) {
        //vector<Point3f> xyz3D = XYZ(M1, M2, R, T, points_L, points_R);
        //获取视差图
        Mat dis(imgR.rows, imgR.cols, CV_32FC1, Scalar(0));
        for (int i = 0; i < points_L.size(); i++)
        {
            int y1 = points_L[i].y;
            int x1 = points_L[i].x;
            int x1r = points_R[i].x;
            dis.at<float>(y1, x1) = x1 - x1r;
        }
        //由视差图获取深度图
        Mat imageXYZ, syXYZ;
        reprojectImageTo3D(dis, imageXYZ, Q, false, -1);
        reprojectImageTo3D(dis, syXYZ, Q, false, -1);
        vector<float> data;
        float x1=0,y1=0,z1=0,x2=0,y2=0,z2=0;
        for (int i = 1; i < points_L.size(); i++)
        {
            //printf_s("左%d图：x=%2f,y=%2f\n", i, points_L.at(i).x, points_L.at(i).y);
            //printf_s("右%d图：x=%2f,y=%2f,d=%2fmm\n", i, points_R.at(i).x, points_R.at(i).y, imageXYZ.at<float>((int)points_L.at(i).y, 3 * (int)points_L.at(i).x + 2));
            x1 = x2;
            y1 = y2;
            z1 = z2;
            x2 = imageXYZ.at<float>((int) points_L.at(i).y, 3 * (int) points_L.at(i).x);
            y2 = imageXYZ.at<float>((int) points_L.at(i).y, 3 * (int) points_L.at(i).x + 1);
            z2 = imageXYZ.at<float>((int) points_L.at(i).y, 3 * (int) points_L.at(i).x + 2);
            float dist = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
            printf("第%d个三维点坐标X=%2f Y=%2f Z=%2f dist = %3f\n", i+1,x2,y2,z2,dist);
            //cout << x2-x1 << "\t" << y2-y1 << "\t" << z2-z1 << endl << endl;
            data.push_back(imageXYZ.at<float>((int) points_L.at(i).y, 3 * (int) points_L.at(i).x + 2));
        }
        double sum = accumulate(std::begin(data), std::end(data), 0.0);
        double mean = sum / data.size(); //均值

        double accum = 0.0;
        std::for_each(std::begin(data), std::end(data), [&](const double d) {
            accum += (d - mean) * (d - mean);
        });

        double stdev = sqrt(accum / (data.size() - 1)); //方差
//        for (int i = 0; i < points_L.size(); i++)
//        {
//        	circle(Left_remap, Point(points_L.at(i).x, points_L.at(i).y), 3 * i, Scalar(0, 0, 255));
//        	circle(Right_remap, Point(points_R.at(i).x, points_R.at(i).y), 3 * i, Scalar(0, 0, 255));
//        	putText(Left_remap, to_string(i), Point(points_L.at(i).x, points_L.at(i).y), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0));
//        	putText(Right_remap, to_string(i), Point(points_R.at(i).x, points_R.at(i).y), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0));
//        }
        saveXYZ(point_cloud_filename.c_str(), imageXYZ);
        cout << stdev << endl;
        waitKey(0);
        return 0;
    }
}