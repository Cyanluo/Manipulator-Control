#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

const int imageWidth = 640;                             //摄像头的分辨率
const int imageHeight = 480;
Size imageSize = Size(imageWidth, imageHeight);

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
Rect validROIR;

// Mat mapLx, mapLy, mapRx, mapRy;     //映射表
// Mat Rl, Rr, Pl, Pr, Q;            //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
Mat xyz;              //三维坐标
Mat R, T, R1, P1, R2, P2, Q;

Point origin;         //鼠标按下的起始点
Rect selection;      //定义矩形选框
bool selectObject = false;    //是否选择对象


Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);


static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 16.0e4;
    FILE* fp = fopen(filename, "wt");
    printf("%d %d \n", mat.rows, mat.cols);
    for (int y = 0; y < mat.rows; y++)
    {
        for (int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);

        }
    }
    fclose(fp);
}

/*给深度图上色*/
void GenerateFalseMap(cv::Mat &src, cv::Mat &disp)
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
    for (int i = 0; i<7; i++) {
        weights[i] = sum / map[i][3];
        cumsum[i + 1] = cumsum[i] + map[i][3] / sum;
    }

    int height_ = src.rows;
    int width_ = src.cols;
    // for all pixels do
    for (int v = 0; v<height_; v++) {
        for (int u = 0; u<width_; u++) {

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

      /*****立体匹配*****/
void stereo_match(int, void*)
{
    sgbm->setPreFilterCap(63);
    int sgbmWinSize =  2;//根据实际情况自己设定
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

    dispf.convertTo(disp8, CV_8U, 255 / (NumDisparities *16.));
    reprojectImageTo3D(dispf, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
    imshow("disparity", disp8);
    Mat color(dispf.size(), CV_8UC3);
    GenerateFalseMap(disp8, color);//转成彩图
    imshow("disparity", color);
    saveXYZ("xyz.xls", xyz);
}



/*****描述：鼠标操作回调*****/
static void onMouse(int event, int x, int y, int, void*)
{
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }

    switch (event)
    {
    case EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
        cout << origin << "in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
        break;
    case EVENT_LBUTTONUP:    //鼠标左按钮释放的事件
        selectObject = false;
        if (selection.width > 0 && selection.height > 0)
            break;
    }
}


/*****主函数*****/
int main()
{
	cv::VideoCapture camera0(2);
	if (!camera0.isOpened()) return 1;
    camera0.set(CAP_PROP_FOURCC, CAP_OPENCV_MJPEG);//设置为MJPG格式
    camera0.set(CAP_PROP_FRAME_WIDTH, 1280);
    camera0.set(CAP_PROP_FRAME_HEIGHT, 480);
    cout << "CV_CAP_PROP_FPS: " << camera0.get(CAP_PROP_FPS) << endl;

	while (true)
    {
		cv::Mat frame;
        camera0 >> frame;

        rgbImageL = frame(Rect(0, 0, 640, 480));
        rgbImageR = frame(Rect(640, 0, 640, 480));
		
		cv::imshow("sLeft", rgbImageL);
        cv::imshow("sRight", rgbImageR);

		int c = waitKey(1);
        if (' ' == char(c))
        {
			string path_intrinsics("../intrinsics.yml"),
					path_extrinsics("../extrinsics.yml");
			Size ImgSize(640, 480);
			//rgbImageL = imread("../pic/left/1.jpg");
			//rgbImageR = imread("../pic/right/1.jpg");//24不行，23右图70二值化
			// 读取相机内外参数
			FileStorage fs(path_intrinsics, FileStorage::READ);
			if (!fs.isOpened()) 
			{
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
			remap(rgbImageL, rectifyImageL, map11, map12, INTER_LINEAR);
			remap(rgbImageR, rectifyImageR, map21, map22, INTER_LINEAR);

			/*  把校正结果显示出来*/

			//显示在同一张图上
			Mat canvas;
			double sf;
			int w, h;
			sf = 700. / MAX(imageSize.width, imageSize.height);
			w = cvRound(imageSize.width * sf);
			h = cvRound(imageSize.height * sf);
			canvas.create(h, w * 2, CV_8UC3);   //注意通道

												//左图像画到画布上
			Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分
			resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //把图像缩放到跟canvasPart一样大小
			Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //获得被截取的区域
				cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
			//rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形
			cout << "Painted ImageL" << endl;

			//右图像画到画布上
			canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分
			resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
			Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
				cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
			//rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
			cout << "Painted ImageR" << endl;

			//画上对应的线条
			for (int i = 0; i < canvas.rows; i += 16)
				line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
			imshow("rectified", canvas);

			/*  立体匹配    */
			namedWindow("disparity", WINDOW_NORMAL);
			//鼠标响应函数setMouseCallback(窗口名称, 鼠标回调函数, 传给回调函数的参数，一般取0)
			setMouseCallback("disparity", onMouse, 0);//disparity
			stereo_match(0, 0);

			waitKey(1);
		}
		else if ('q' == char(c))
		{
			break;
		}

	}

    return 0;
}