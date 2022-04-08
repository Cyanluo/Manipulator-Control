#include "binocularCamera.h"

binocularCamera::binocularCamera()
{
	cap = new VideoCapture();

	imgSize = Size(640, 480);

	if(!loadCaParams())
	{
		cout << "Error:can not load camera parameters" << endl;
	}

	sgbm = StereoSGBM::create(0, 16, 3);
}

binocularCamera::~binocularCamera()
{
	delete cap;
}

bool binocularCamera::loadCaParams()
{
	bool ret = true;
	
	string path_intrinsics("../intrinsics.yml"),
	path_extrinsics("../extrinsics.yml");

	FileStorage fs(path_intrinsics, FileStorage::READ);
	if (!fs.isOpened()) 
	{
		cout << "Failed to open file intrinsics.yml" << endl;
		return false;
	}

	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	fs.open(path_extrinsics, FileStorage::READ);
	if (!fs.isOpened()) 
	{
		cout << "Failed to open file extrinsics.yml" << endl;
		return false;
	}

	fs["R"] >> R;
	fs["T"] >> T;

	stereoRectify(M1, D1, M2, D2, imgSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imgSize);
	initUndistortRectifyMap(M1, D1, R1, P1, imgSize, CV_16SC2, map11, map12);//立体校正，即进行畸变校正和极线校正，使得y方向没有视差
    initUndistortRectifyMap(M2, D2, R2, P2, imgSize, CV_16SC2, map21, map22);
	
	return true;
}

bool binocularCamera::open(int cameraID)
{
	this->cap->open(cameraID);

	if (!this->cap->isOpened()) 
	{
		cout << "can not open camera" << endl;
		
		return false;
	}
	
	this->cap->set(CAP_PROP_FOURCC, CAP_OPENCV_MJPEG);//设置为MJPG格式
    this->cap->set(CAP_PROP_FRAME_WIDTH, (imgSize.width)*2);
    this->cap->set(CAP_PROP_FRAME_HEIGHT, imgSize.height);
    cout << "CV_CAP_PROP_FPS: " << this->cap->get(CAP_PROP_FPS) << endl;

	return true;
}

Mat binocularCamera::getImage(bool& ok)
{
	ok = true;
	Mat ret;

	if(this->cap->isOpened())
	{
		*(this->cap) >> ret;
	}
	else
	{
		ok = false;
		
		cout << "camera is close!!!" << endl;
	}

	return ret;
}

bool binocularCamera::getImage(Mat& dstL, Mat& dstR)
{
	bool ret = true;

	Mat img = getImage(ret);

	if(ret)
	{
		dstL = img(Rect(0, 0, imageSize.width, imageSize.height));
        dstR = img(Rect(imageSize.width, 0, imageSize.width, imageSize.height));
	}

	return ret;
}


bool binocularCamera::getCorrectedImg(Mat& dstL, Mat& dstR)
{
	Mat rgbImageL, rgbImageR;
	bool ret = false;

	if(getImage(rgbImageL, rgbImageR))
	{
		remap(rgbImageL, dstL, map11, map12, INTER_LINEAR);
		remap(rgbImageR, dstR, map21, map22, INTER_LINEAR);

		ret = true;
	}

	return ret;	
}

Mat binocularCamera::mergingDisplay(Mat& srcL, Mat& srcR, bool drawLine)
{
	Mat canvas;
	double sf;
	int w, h;
	sf = 700. / MAX(imageSize.width, imageSize.height);
	w = cvRound(imageSize.width * sf);
	h = cvRound(imageSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);   //注意通道

	//左图像画到画布上
	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分
	resize(srcL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //把图像缩放到跟canvasPart一样大小
	Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //获得被截取的区域
		cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));

	//右图像画到画布上
	canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分
	resize(srcR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));

	if(drawLine)
	{
		//画上对应的线条
		for (int i = 0; i < canvas.rows; i += 16)
			line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
	}

	return canvas;
}

/*给深度图上色*/
void binocularCamera::GenerateFalseMap(cv::Mat &src, cv::Mat &disp)
{
    // color map
    float max_val = 255.0f;
    float map[8][4] = { { 0,0,0,114 },{ 0,0,1,185 },{ 1,0,0,114 },{ 1,0,1,174 },
    { 0,1,0,114 },{ 0,1,1,185 },{ 1,1,0,114 },{ 1,1,1,0 } };
    float sum = 0;
    for (int i = 0; i<8; i++)
        sum += map[i][3];

    float weights[8]; // relative   weights
    float cumsum[8];  // cumulative weights
    cumsum[0] = 0;
    for (int i = 0; i<7; i++) 
	{
        weights[i] = sum / map[i][3];
        cumsum[i + 1] = cumsum[i] + map[i][3] / sum;
    }

    int height_ = src.rows;
    int width_ = src.cols;
    // for all pixels do
    for (int v = 0; v<height_; v++) 
	{
        for (int u = 0; u<width_; u++) 
		{

            // get normalized value
            float val = std::min(std::max(src.data[v*width_ + u] / max_val, 0.0f), 1.0f);

            // find bin
            int i;
            for (i = 0; i<7; i++)
                if (val<cumsum[i + 1])
                    break;

            // compute red/green/blue values
            float   w = 1.0 - (val - cumsum[i])*weights[i];
            uchar r = (uchar)((w*map[i][0] + (1.0 - w)*map[i + 1][0]) * 255.0);
            uchar g = (uchar)((w*map[i][1] + (1.0 - w)*map[i + 1][1]) * 255.0);
            uchar b = (uchar)((w*map[i][2] + (1.0 - w)*map[i + 1][2]) * 255.0);
            //rgb内存连续存放
            disp.data[v*width_ * 3 + 3 * u + 0] = b;
            disp.data[v*width_ * 3 + 3 * u + 1] = g;
            disp.data[v*width_ * 3 + 3 * u + 2] = r;
        }
    }
}

Mat binocularCamera::stereoMatch(Mat& rectifyImageL, Mat& rectifyImageR, Mat& deepImg)
{
	Mat xyz;              //三维坐标
	
	sgbm->setPreFilterCap(63);
	int sgbmWinSize =  4;//根据实际情况自己设定
	int NumDisparities = 16*20;//根据实际情况自己设定
	int UniquenessRatio = 6;//根据实际情况自己设定
	sgbm->setBlockSize(sgbmWinSize);
	int cn = rectifyImageL.channels();

	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(NumDisparities);
	sgbm->setUniquenessRatio(UniquenessRatio);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(10);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setMode(StereoSGBM::MODE_SGBM);
	Mat disp, dispf, disp8;
	sgbm->compute(rectifyImageL, rectifyImageR, disp);
	//去黑边
	Mat img1p, img2p;
	copyMakeBorder(rectifyImageL, img1p, 0, 0, NumDisparities, 0, BORDER_REPLICATE);
	copyMakeBorder(rectifyImageR, img2p, 0, 0, NumDisparities, 0, BORDER_REPLICATE);
	dispf = disp.colRange(NumDisparities, img2p.cols- NumDisparities);

	reprojectImageTo3D(dispf, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
	xyz = xyz * 16;
	dispf.convertTo(disp8, CV_8U, 255 / (NumDisparities *16.));
	Mat color(dispf.size(), CV_8UC3);		
	GenerateFalseMap(disp8, color);//转成彩图
	deepImg = color;
	
	return xyz;	
}

Point2d binocularCamera::pixel2cam(const Point& p, const Mat& K)
{
	return Point2d
	(
		(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
		(p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
	);
}

Point3d binocularCamera::triangulation(Point2d lp, Point2d rp)
{
	vector<Point2d> pts_1, pts_2;

	pts_1.push_back(pixel2cam(lp, M1));
	pts_2.push_back(pixel2cam(rp, M2));

	Mat T2 = (Mat_<double>(3, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);

	Mat T1 = (Mat_<double>(3, 4) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), T.at<double>(0, 0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), T.at<double>(1, 0),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), T.at<double>(2, 0)
		);

	Mat pts_4d;

	cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

	cout << pts_4d;

	Point3d p;

	double D = pts_4d.at<double>(3, 0);
	//Mat x = pts_4d.col(i);
	//x /= x.at<float>(3, 0); // 归一化
	
	p.x = pts_4d.at<double>(0, 0)/D;
	p.y = pts_4d.at<double>(1, 0)/D;
	p.z = pts_4d.at<double>(2, 0)/D;		

	return p;
}
