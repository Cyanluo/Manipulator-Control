#ifndef BINOCULARCAMERA_H_
#define BINOCULARCAMERA_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

class binocularCamera
{
	const int imageWidth = 640;                             //摄像头的分辨率
	const int imageHeight = 480;
	Size imageSize = Size(imageWidth, imageHeight);

	Mat map11, map12, map21, map22;

	Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
	Rect validROIR;
	Size imgSize;

	Mat R, T, R1, P1, R2, P2, Q;
	Mat M1, D1, M2, D2;

	Ptr<StereoSGBM> sgbm;

	VideoCapture* cap;
	bool loadCaParams();
	void GenerateFalseMap(cv::Mat &src, cv::Mat &disp);
	Point2d pixel2cam(const Point& p, const Mat& K);
public:
	binocularCamera();
	
	bool open(int cameraID);
	Mat getImage(bool& ok);
	bool getImage(Mat& dstL, Mat& dstR);
	bool getCorrectedImg(Mat& dstL, Mat& dstR);
	Mat mergingDisplay(Mat& srcL, Mat& srcR, bool drawLine=false);
	Mat stereoMatch(Mat& rectifyImageL, Mat& rectifyImageR, Mat& deepImg);
	Point3d triangulation(Point2d lp, Point2d rp);

	~binocularCamera();
};

#endif //BINOCULARCAMERA_H_
