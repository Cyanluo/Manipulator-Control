#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

//归一化平面坐标
Point2d pixel2cam(const Point& p, const Mat& K)
{
	return Point2d
	(
		(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
		(p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
	);
 
}

void triangulation(
	const vector< KeyPoint >& keypoint_1,
	const vector< KeyPoint >& keypoint_2,
	const std::vector< DMatch >& matches,
	const Mat& R, const Mat& t,
	vector< Point3d >& points)
{
	Mat T1 = (Mat_<double>(3, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	Mat T2 = (Mat_<double>(3, 4) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
		);
 
	Mat K = (Mat_<double>(3, 3) << 2654.085430584877, 0, 818.2876194006783,
		0, 2653.357741817838, 1644.824103626496,
		0, 0, 1);
	vector<Point2d> pts_1, pts_2;
	for (DMatch m : matches)
	{
		// 将像素坐标转换至相机坐标
		pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
		pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
	}
 
	Mat pts_4d;
	cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
	cout << pts_4d;
	// 转换成非齐次坐标
	for (int i = 0; i < pts_4d.cols; i++)
	{
		double D = pts_4d.at<double>(3, i);
		//Mat x = pts_4d.col(i);
		//x /= x.at<float>(3, 0); // 归一化
		Point3d p(
			pts_4d.at<double>(0, i)/D,
			pts_4d.at<double>(1, i)/D,
			pts_4d.at<double>(2, i)/D
		);
		points.push_back(p);
	}
}

int main() {
	//读取两张图像
	Mat image01 = imread("test1.jpg",1);
	Mat image02 = imread("test2.jpg",1);
	cout << "图像读取完成" << endl;
 
	Mat cameraMatrix = (Mat_<double>(3, 3) << 2654.085430584877, 0, 818.2876194006783,
		0, 2653.357741817838, 1644.824103626496,
		0, 0, 1); // 内部参数
	Mat cameraDistCoeffs = (Mat_<double>(1, 5) << 0.1285584874010567, -0.465498801205853,
		0.002259177724517946, -0.0004828871797040054, 0); //畸变参数
 
	//矫正图像
	Mat image1 = image01.clone();
	Mat image2 = image02.clone();
	undistort(image01, image1, cameraMatrix, cameraDistCoeffs);
	undistort(image02, image2, cameraMatrix, cameraDistCoeffs);
	cout << "图像矫正完成" << endl;
	//imshow("image1", image1);
	//imshow("image2", image2);
	//waitKey(2000);
	// 关键点和描数子的容器
	std::vector<cv::KeyPoint> keypoints1;
	std::vector<cv::KeyPoint> keypoints2;
	cv::Mat descriptors1, descriptors2;
	// 创建 SIFT 特征检测器
	cv::Ptr<cv::Feature2D> ptrFeature2D =
		cv::xfeatures2d::SIFT::create(500);
	// 检测 SIFT 特征和相关的描述子
	ptrFeature2D->detectAndCompute(image1, cv::noArray(),
		keypoints1, descriptors1);
	ptrFeature2D->detectAndCompute(image2, cv::noArray(),
		keypoints2, descriptors2);
 
 
	// 匹配两幅图像的描述子
	// 创建匹配器并交叉检查
	cv::BFMatcher matcher(cv::NORM_L2, true);
	std::vector<cv::DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);
	cout << "图像匹配完成" << endl;
	std::nth_element(matches.begin(), matches.begin() + 25, matches.end());
	matches.erase(matches.begin() + 25, matches.end());
 
	cv::Mat imageMatches;
	cv::drawMatches(image1, keypoints1,  // 1st image and its keypoints
		image2, keypoints2,  // 2nd image and its keypoints
		matches,			// the matches
		imageMatches,		// the image produced
		cv::Scalar(255, 255, 255),  // color of the lines
		cv::Scalar(255, 255, 255),  // color of the keypoints
		std::vector<char>(),
		2);
	cv::pyrDown(imageMatches, imageMatches);
	cv::pyrDown(imageMatches, imageMatches);
	cv::imshow("Matches", imageMatches);
 
 
 
	// 将关键点转换成 Point2f 类型
	std::vector<cv::Point2f> points1, points2;
	for (std::vector<cv::DMatch>::const_iterator it =
		matches.begin(); it != matches.end(); ++it) {
		// 获取左侧关键点的位置
		points1.push_back(keypoints1[it->queryIdx].pt);
		// 获取右侧关键点的位置
		points2.push_back(keypoints2[it->queryIdx].pt);
	}
	cout << "二维点完成" << endl;
	// 找出 image1 和 image2 之间的本质矩阵
	cv::Mat inliers;
	cv::Mat essential = cv::findEssentialMat(points1, points2,
		cameraMatrix, // 内部参数
		cv::RANSAC,
		0.9, 1.0, // RANSAC 方法
		inliers); // 提取到的内点
	cout << "求出本质矩阵" << endl;
	// 根据本质矩阵还原相机的相对姿态
	cv::Mat rotation, translation;
	cv::recoverPose(essential, // 本质矩阵
		points1, points2, // 匹配的关键点
		cameraMatrix, // 内部矩阵
		rotation, translation, // 计算的移动值
		inliers); // 内点匹配项
 
	cout << "求出T" << endl;
 
	// 三角剖分
	Mat pts_4d;
	vector<Point3d> points; //三维重建坐标
 
	triangulation(keypoints1, keypoints2, matches, rotation, translation, points);
 
	cout << "三维重建完成" << endl;
 
 
	//-- 验证三角化点与特征点的重投影关系
	Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
	for (int i = 0; i < matches.size(); i++)
	{
		Point2d pt1_cam = pixel2cam(keypoints1[matches[i].queryIdx].pt, K);
		Point2d pt1_cam_3d(
			points[i].x / points[i].z,
			points[i].y / points[i].z
		);
 
		cout << "point in the first camera frame: " << pt1_cam << endl;
		cout << "point projected from 3D " << pt1_cam_3d << ", d=" << points[i].z << endl;
 
		// 第二个图
		Point2f pt2_cam = pixel2cam(keypoints2[matches[i].trainIdx].pt, K);
		Mat pt2_trans = rotation * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + translation;
		pt2_trans /= pt2_trans.at<double>(2, 0);
		cout << "point in the second camera frame: " << pt2_cam << endl;
		cout << "point reprojected from second frame: " << pt2_trans.t() << endl;
		cout << endl;
	}
 
	waitKey(0);
	return 0;
}
