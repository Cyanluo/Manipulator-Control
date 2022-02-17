#include "../include/DoubleCameraOpt.h"
#include <utility>
#include <iomanip>
#include <numeric>

DoubleCameraOpt::DoubleCameraOpt(const int *camIndex, const string *camName, Size frameSize)
        : frameSize(std::move(frameSize)), usingCamPath(false) {

    readFromPath = false;

    // ��ʼ��SingleCameraOpt����
    for (int i = 0; i < 2; ++i) {
        CAMERA_INDEX[i] = camIndex[i];
        CAMERA_NAME[i] = camName[i];
        singleCamera[i] = new SingleCameraOpt(CAMERA_INDEX[i], CAMERA_NAME[i], Size(1280, 720));
        if (!singleCamera[i]->isOpened()) {
            cout << "open camera error: " << CAMERA_NAME[i];
            open = false;
        }
    }
    open = true;
    init(camName);
}

DoubleCameraOpt::DoubleCameraOpt(const int *camIndex, const string *camName, Size frameSize,
                                 const int virCamIndex[2], const int virCamDispIndex[2], int virCamSpliceIndex)
        : frameSize(std::move(frameSize)), usingCamPath(false) {

    // ��ʼ��SingleCameraOpt����
    for (int i = 0; i < 2; ++i) {
        CAMERA_INDEX[i] = camIndex[i];
        CAMERA_NAME[i] = camName[i];

        if (virCamIndex[i] == -1) {
            singleCamera[i] = new SingleCameraOpt(CAMERA_INDEX[i], CAMERA_NAME[i], Size(1280, 720));
        } else {
            singleCamera[i] = new SingleCameraOpt(CAMERA_INDEX[i], CAMERA_NAME[i], Size(1280, 720), virCamIndex[i]);
        }

        if (!singleCamera[i]->isOpened()) {
            cout << "open camera error: " << CAMERA_NAME[i];
            open = false;
        }
    }
    cameraGeneralInit(camName, frameSize, virCamDispIndex, virCamSpliceIndex);
}

DoubleCameraOpt::DoubleCameraOpt(const string *camPath, const string *camName, Size frameSize,
                                 const int virCamIndex[2], const int virCamDispIndex[2], int virCamSpliceIndex)
        : frameSize(std::move(frameSize)), usingCamPath(true) {

    // ��ʼ��SingleCameraOpt����
    for (int i = 0; i < 2; ++i) {
        CAMERA_PATH[i] = camPath[i];
        CAMERA_NAME[i] = camName[i];

        if (virCamIndex[i] == -1) {
            singleCamera[i] = new SingleCameraOpt(CAMERA_PATH[i], CAMERA_NAME[i], Size(1280, 720));
        } else {
            singleCamera[i] = new SingleCameraOpt(CAMERA_PATH[i], CAMERA_NAME[i], Size(1280, 720), virCamIndex[i]);
        }

        if (!singleCamera[i]->isOpened()) {
            cout << "open camera error: " << CAMERA_NAME[i];
            open = false;
        }
    }

    cameraGeneralInit(camName, frameSize, virCamDispIndex, virCamSpliceIndex);
}

DoubleCameraOpt::DoubleCameraOpt(const string &path, int fileNameLen, const string *camName, Size frameSize)
: frameSize(std::move(frameSize)) {

    readFromPath = true;
    setFilePath(path);

    // ��ʼ��SingleCameraOpt����
    for (int i = 0; i < 2; ++i) {
        CAMERA_NAME[i] = camName[i];
        singleCamera[i] = new SingleCameraOpt(path + CAMERA_NAME[i] + "/", fileNameLen, CAMERA_NAME[i]);
    }

    init(camName);
}

DoubleCameraOpt::DoubleCameraOpt(const string &path, int fileNameLen, const string *camName, Size frameSize,
                                 const int virCamDispIndex[2], int virCamSpliceIndex)
        : frameSize(std::move(frameSize)) {

    readFromPath = true;
    setFilePath(path);

    // ��ʼ��SingleCameraOpt����
    for (int i = 0; i < 2; ++i) {
        CAMERA_NAME[i] = camName[i];
        singleCamera[i] = new SingleCameraOpt(path + CAMERA_NAME[i] + "/", fileNameLen, CAMERA_NAME[i]);
    }

    cameraGeneralInit(camName, frameSize, virCamDispIndex, virCamSpliceIndex);
}

void DoubleCameraOpt::cameraGeneralInit(const string *camName, Size frameSize, const int virCamDispIndex_[2], int virCamSpliceIndex_){

    open = true;
    init(camName);

    // ���������������ͷ
    for (int i = 0; i < 2; ++i) {
        virCamDispIndex[i] = virCamDispIndex_[i];
        if (virCamDispIndex[i] == -1) {
            continue;
        }
        virCamDisp[i] = VirtualCamera(virCamDispIndex_[i], frameSize.width, frameSize.height);
    }

    // ��������ƴ������ͷ
    virCamSpliceIndex = virCamSpliceIndex_;
    if (virCamSpliceIndex != -1) {
        virCamSplice = VirtualCamera(virCamSpliceIndex, spliceWidth, frameSize.height);
    }

    writeVirCam = true;
}

void DoubleCameraOpt::init(const string *camName){
    SingleCameraFrameSize = singleCamera[0]->getFrameSize();
    groupName = camName[0] + "&" + camName[1];
    cout << endl << "----------" << groupName << "----------" << endl;

    // ��ȡ˫Ŀ�����ļ�
    if (!readXML()) {
        cout << "can not find stereo camera calibration file!" << endl;
        return;
    }
    cout << "load stereo camera parameters." << endl;

    // ��ʼ��˫Ŀ��������
    for (int i = 0; i < 2; ++i) {
        initUndistortRectifyMap(cameraMatrix[i], distCoeffs[i], R1[i], P1[i],
                                SingleCameraFrameSize, CV_16SC2, rmap[i][0], rmap[i][1]);
    }

    // ��ʼ������ƥ���㷨
    Mat tf[2];
    getNewFrame(tf, true);
    initSGBM(tf[0]);

    // ��ʼ��˫Ŀƴ�Ӳ���
    leftCutIndex = (int) round(frameSize.width * leftCutPos);
    rightCutIndex = (int) round(frameSize.width * rightCutPos);
    spliceWidth = leftCutIndex + frameSize.width - rightCutIndex;

    cout << "----------" << groupName << " OK----------" << endl;
}

bool DoubleCameraOpt::readXML() {
    FileStorage fs("../res/camera_parameters/" + groupName + ".xml", FileStorage::READ);
    if (fs.isOpened()) {
        // �ɹ����ļ�����ʼ������
        for (int i = 0; i < 2; ++i) {
            fs[CAMERA_NAME[i] + "_cameraMatrix"] >> cameraMatrix[i];
            fs[CAMERA_NAME[i] + "_distCoeffs"] >> distCoeffs[i];
            fs[CAMERA_NAME[i] + "_R1"] >> R1[i];
            fs[CAMERA_NAME[i] + "_P1"] >> P1[i];
            fs[CAMERA_NAME[i] + "_validRoi"] >> validRoi[i];
        }
        fs["Q"] >> Q;
        return true;
    } else {
        cout << "Error: can't open file \"" << "../res/camera_parameters/" + groupName + ".xml" << "\"" << endl;
        return false;
    }
}

bool DoubleCameraOpt::isOpened() const {
    return open;
}

bool DoubleCameraOpt::getNewFrame(Mat *frame, bool gray) {
    Mat tf;
    for (int i = 0; i < 2; ++i) {
        singleCamera[i]->getNewFrame(tf, true, false);
        remap(tf, tf, rmap[i][0], rmap[i][1], INTER_LINEAR);
        resize(tf, tf, frameSize, 0, 0, INTER_LINEAR);
        lastFrame[i] = Mat(tf);
    }

    // �����Ӳ�ͼ
    if (autoCalcDisp) {
        countDisp();
    }

    // ����ƴ��ͼ
    if (autoCalcSplice) {
        countSplice();
    }

    if (frame != nullptr){
        return getLastFrame(frame, gray);}
    else{
        return true;}
}

bool DoubleCameraOpt::getLastFrame(Mat *frame, bool gray) {
    if (lastFrame[0].empty() || lastFrame[1].empty()) {
        return false;
    }

    Mat tf;
    for (int i = 0; i < 2; ++i) {
        tf = lastFrame[i];
        if (gray)
            cvtColor(tf, tf, COLOR_BGR2GRAY);

        frame[i] = Mat(tf);
    }

    if (frame[0].empty() || frame[1].empty()) {
        return false;
    } else {
        return true;
    }
}

bool DoubleCameraOpt::getRectifiedView(Mat &rec) {
    if (lastFrame[0].empty() || lastFrame[1].empty()) {
        return false;
    }
    Mat canvas;
    double sf;
    int w, h;
    sf = 600. / MAX(frameSize.width, frameSize.height);
    w = cvRound(frameSize.width * sf);
    h = cvRound(frameSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);

    /*��ͼ�񻭵�������*/
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //�õ�������һ����
    resize(lastFrame[0], canvasPart, canvasPart.size(), 0, 0, INTER_AREA);        //��ͼ�����ŵ���canvasPartһ����С
//    Rect vroiL(cvRound(validRoi[0].x * sf), cvRound(validRoi[0].y * sf),                //��ñ���ȡ������
//               cvRound(validRoi[0].width * sf), cvRound(validRoi[0].height * sf));
//    rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //����һ������

    /*��ͼ�񻭵�������*/
    canvasPart = canvas(Rect(w, 0, w, h));                                      //��û�������һ����
    resize(lastFrame[1], canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
//    Rect vroiR(cvRound(validRoi[1].x * sf), cvRound(validRoi[1].y * sf),
//               cvRound(validRoi[1].width * sf), cvRound(validRoi[1].height * sf));
//    rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

    /*���϶�Ӧ������*/
    for (int i = 0; i < canvas.rows; i += 16)
        line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

    rec = Mat(canvas);
    return !rec.empty();
}

bool DoubleCameraOpt::convertFrame(const Mat *frameIn, Mat *frameOut) {
    Mat tf;
    for (int i = 0; i < 2; ++i) {
        tf = frameIn[i];
        remap(tf, tf, rmap[i][0], rmap[i][1], INTER_LINEAR);
        resize(tf, tf, frameSize, 0, 0, INTER_LINEAR);
        lastFrame[i] = Mat(tf);
    }
    return getLastFrame(frameOut, false);
}

void DoubleCameraOpt::initSGBM(const Mat& frame){
    // ��ʼ��CPU����ʵ��
    int winSize = 5;
    // ��ȡ����ͼͨ����
    int pngChannels = frame.channels();

    for (int i = 0; i < 2; ++i) {
        sgbm[i] = StereoSGBM::create();

        // Ԥ�����˲����ض�ֵ����Ӱ��
        sgbm[i]->setPreFilterCap(31);

        // SAD���ڴ�С
        sgbm[i]->setBlockSize(winSize);

        // �����Ӳ�ƽ���ȵ�һ������ϵ��һ��Ϊ8
        sgbm[i]->setP1(8 * pngChannels * winSize * winSize);

        // �����Ӳ�ƽ���ȵڶ�������ϵ��һ��Ϊ32
        sgbm[i]->setP2(32 * pngChannels * winSize * winSize);

        // ��С�Ӳһ��Ϊ0
        sgbm[i]->setMinDisparity(0);

        // �Ӳ�������Χ�������ú�
        sgbm[i]->setNumDisparities(nmDisparities);

        // �Ӳ�Ψһ�԰ٷֱȣ���ʱ����Ӱ�죬�����¿ն����࣬��С���ֲ����ڵĽ���������
        sgbm[i]->setUniquenessRatio(5);

        // ����Ӳ���ͨ����仯�ȵĴ��ڴ�С��Ӱ��Ч������һ��������ͬ
        sgbm[i]->setSpeckleWindowSize(100);

        // �Ӳ�仯��ֵ����ʱ����Ӱ�죬��Сʱ�ն����࣬�ϴ�������Ӱ��
        sgbm[i]->setSpeckleRange(32);

        // �����Ӳ�ͼ���������죬��ʱ����Ӱ�죬С��ն����࣬������ƥ�����࣬Ĭ��Ϊ1���Բ��ݶ�20
        sgbm[i]->setDisp12MaxDiff(20);

        // ����ȫ�ߴ�˫ͨ����̬����㷨
        //        sgbm[i]->setMode(StereoSGBM::MODE_SGBM);
        sgbm[i]->setMode(StereoSGBM::MODE_SGBM_3WAY);
        //        sgbm[i]->setMode(StereoSGBM::MODE_HH);
        //        sgbm[i]->setMode(StereoSGBM::MODE_HH4);
    }


    // ��ʼ��GPU����ʵ��
    for (int i = 0; i < 2; ++i) {
        sgm[i] = cuda::createStereoSGM();
        sgm[i]->setMode(StereoSGBM::MODE_HH);
    }
}

void DoubleCameraOpt::leftStereoMatch(){
    // ��ʱ��ʼ
    auto up = chrono::steady_clock::now();

    Mat disp, dispShow, img[2];
    getLastFrame(img, true);

    // �������Ӳ�ͼ
    if (!GPU) {
        sgbm[0]->compute(img[0], img[1], disp);
    } else {
        cuda::GpuMat d_img[2], d_disp;
        for (int i = 0; i < 2; ++i) {
            d_img[i].upload(img[i]);
        }
        sgm[0]->compute(d_img[0], d_img[1], d_disp);
        d_disp.download(disp);
    }

    // �ն����
    if (insertDisp) {
        Mat temp;
        disp.convertTo(temp, CV_32F);
        insertDepth32f(temp);
        // ת8λ
        temp.convertTo(dispShow, CV_8U, 255 / (nmDisparities * 16.));
    } else {
        disp.convertTo(dispShow, CV_8U, 255 / (nmDisparities * 16.));
    }

    // չʾ�Ӳ�ͼת��ɫ
    dispShow *= 2.5;
    applyColorMap(dispShow, dispShow, COLORMAP_JET);

    // ��ֵ
    lastDisp[0] = Mat(disp);
    lastDispShow[0] = Mat(dispShow);

    // �����������
    reprojectImageTo3D(lastDisp[0], lastPointCloud[0], Q, false);
    lastPointCloud[0] *= 16;

    // ��ʱ����
    if (printTime){
        auto down = chrono::steady_clock::now();
        auto d = chrono::duration_cast<chrono::milliseconds>(down - up);
        cout << "\ttimeLeft = " << d.count();
    }
}

void DoubleCameraOpt::rightStereoMatch(){
    // ��ʱ��ʼ
    auto up = chrono::steady_clock::now();

    Mat disp, dispShow, img[2], newImg[2];
    getLastFrame(img, true);

    // ˮƽ��ת
    for (int i = 0; i < 2; ++i) {
        flip(img[i], newImg[i], 1);
    }

    // �������Ӳ�ͼ
    if (!GPU) {
        sgbm[1]->compute(newImg[1], newImg[0], disp);
    } else {
        cuda::GpuMat d_newImg[2], d_disp;
        for (int i = 0; i < 2; ++i) {
            d_newImg[i].upload(newImg[i]);
        }
        sgm[1]->compute(d_newImg[1], d_newImg[0], d_disp);
        d_disp.download(disp);
    }

    // ��ת����
    flip(disp, disp, 1);

    // �ն����
    if (insertDisp) {
        Mat temp;
        disp.convertTo(temp, CV_32F);
        insertDepth32f(temp);
        // ת8λ
        temp.convertTo(dispShow, CV_8U, 255 / (nmDisparities * 16.));
    } else {
        disp.convertTo(dispShow, CV_8U, 255 / (nmDisparities * 16.));
    }

    // չʾ�Ӳ�ͼת��ɫ
    dispShow *= 2.5;
    applyColorMap(dispShow, dispShow, COLORMAP_JET);

    // ��ֵ
    lastDisp[1] = Mat(disp);
    lastDispShow[1] = Mat(dispShow);

    // �����������
    reprojectImageTo3D(lastDisp[1], lastPointCloud[1], Q, false);
    lastPointCloud[1] *= 16;

    // ��ʱ����
    if (printTime){
        auto down = chrono::steady_clock::now();
        auto d = chrono::duration_cast<chrono::milliseconds>(down - up);
        cout << "\ttimeRight = " << d.count();
    }
}

void DoubleCameraOpt::countDisp() {
    // ��ʱ��ʼ
    auto up = chrono::steady_clock::now();

    Mat img[2];
    getLastFrame(img, true);

    if (img[0].empty() || img[1].empty()) {
        return;
    }

    // ���̼߳��������Ӳ�ͼ
    thread left(&DoubleCameraOpt::leftStereoMatch , this);
    thread right(&DoubleCameraOpt::rightStereoMatch, this);
    left.join();
    right.join();

    // ��ȡ���ͨ��
    for (int i = 0; i < 2; ++i) {
        vector<Mat> t(lastPointCloud[0].channels());
        split(lastPointCloud[i], t);
        lastDepth[i] = Mat(t[2]);
    }

    // ��ʱ����
    if (printTime){
        auto down = chrono::steady_clock::now();
        auto d = chrono::duration_cast<chrono::milliseconds>(down - up);
        cout << "\ttimeTotal = " << d.count() << endl;
    }

    // д����������ͷ
    if (writeVirCam) {
        for (int i = 0; i < 2; ++i) {
            if (virCamDispIndex[i] == -1) {
                continue;
            }
            virCamDisp[i].write_frame(lastDispShow[i]);
        }
    }
}

bool DoubleCameraOpt::getLastDispShow(Mat *frameOut) {
    for (int i = 0; i < 2; ++i) {
        frameOut[i] = Mat(lastDispShow[i]);
    }

    if (frameOut[0].empty() || frameOut[1].empty()) {
        return false;
    } else {
        return true;
    }
}

double DoubleCameraOpt::getAreaDistance(const Rect &area, int frameIndex) {
    if (area.empty()) {
        return -1;
    }

    vector<float> vec = convertMat2Vector<float>(lastDepth[frameIndex](area));

    // ԭʼ������
    int originNum = vec.size();

    if (originNum <= 0) {
        return -1;
    }

    // ����ԭʼƽ������
    double sum = accumulate(begin(vec), end(vec), 0.0);
	double originMean = sum / vec.size();
    originMean /= 10;

    // ��������
    sort(vec.begin(), vec.end());

    // ���㷽��
    sum = accumulate(begin(vec), end(vec), 0.0);
	double mean = sum / vec.size();
    double variance  = 0.0;
    for (float i:vec) {
        variance += (i - mean) * (i - mean);
    }
    variance /= vec.size();

    // ȥ���ն���������ȣ������ȫΪ�ն����������һ���ն�ֵ������ȥ�����пն�ֵ
    for (int i = 0; i < vec.size(); ++i) {
        if (vec[i] > 0 || i == vec.size() - 1) {
            // ����i���ն�����0~i-1���ǿն�
            vec.assign(vec.begin() + i, vec.end());
            break;
        }
    }

    // �����ͷΪ������˵������ȫΪ�ն�
    if (vec[0] <= 0)
        return -1;

    // ����ն�����
    double hole = (double) (originNum - vec.size()) / (double) originNum;

    // ȥ������ֵ
    int deleteNum = int(vec.size() * 0.15);
    vec.assign(vec.begin() + deleteNum, vec.end() - deleteNum);

    // ����ƽ��ֵ
    sum = accumulate(begin(vec), end(vec), 0.0);
	mean = sum / vec.size();
    mean /= 10;

//    cout << endl << "mean=" << mean << endl << "origin_mean=" << originMean << endl << "variance=" << variance << endl
//         << "hole=" << hole << endl << endl;
    double ret = mean / 10e2;
    if (ret > 100) ret = 100;

    return ret;
//    return lastPointCloud[frameIndex].at<Vec3f>(area.tl())[2] / 1000;
}

void DoubleCameraOpt::countSplice() {

    Mat tf = Mat(Size(spliceWidth, frameSize.height), CV_8UC3);
    tf.setTo(0);
    lastFrame[0](Rect(0, 0, leftCutIndex, frameSize.height))
        .copyTo(tf(Rect(0, 0, leftCutIndex, frameSize.height)));
    lastFrame[1](Rect(rightCutIndex, 0, frameSize.width - rightCutIndex, frameSize.height))
            .copyTo(tf(Rect(leftCutIndex, 0, frameSize.width - rightCutIndex, frameSize.height)));

    // д������ƴ������ͷ
    if (writeVirCam and virCamSpliceIndex != -1) {
        virCamSplice.write_frame(tf);
    }

    lastSplice = Mat(tf);
}

void DoubleCameraOpt::getSpliceFrame(Mat &frameOut) {
    if (autoCalcSplice && !lastSplice.empty()) {
        frameOut = Mat(lastSplice);
    } else {
        countSplice();
        frameOut = Mat(lastSplice);
    }
}

const string &DoubleCameraOpt::getFilePath() const {
    return filePath;
}

void DoubleCameraOpt::setFilePath(const string &filePath) {
    DoubleCameraOpt::filePath = filePath;
}

void DoubleCameraOpt::insertDepth32f(cv::Mat& depth)
{
    const int width = depth.cols;
    const int height = depth.rows;
    float* data = (float*)depth.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    // ��������
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = max(0, left);
                right = min(right, width - 1);
                top = max(0, top);
                bot = min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
    }
}

DoubleCameraOpt::~DoubleCameraOpt() {
    delete singleCamera[0];
    delete singleCamera[1];
}
