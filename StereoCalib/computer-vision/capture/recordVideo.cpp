// 拍摄图片序列，自动承接上次编号
// todo 帧率与分辨率问题：720p时大约只有15fps，360p可以达到约30fps

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <queue>
#include <thread>

using namespace std;
using namespace cv;

const String CAMERA_NAME_LIST[2] = {"camera1", "camera2"};
const int CAMERA_INDEX_LIST[2] = {1, 2};
Size imgSize(1280, 720);
//Size imgSize(640, 360);
string path = "../capture_data/record2/" + CAMERA_NAME_LIST[0] + "&" + CAMERA_NAME_LIST[1] + "/";

bool record = false;
const char ESC_KEY = 27;
int imgNum;
bool stop = false;
queue<Mat> imgLeftCache;
queue<Mat> imgRightCache;

bool getRectifiedView(Mat frame[2], Mat &rec) {
    if (frame[0].empty() || frame[1].empty()) {
        return false;
    }
    Mat canvas;
    double sf;
    int w, h;
    sf = 600. / MAX(imgSize.width, imgSize.height);
    w = cvRound(imgSize.width * sf);
    h = cvRound(imgSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);

    /*左图像画到画布上*/
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分
    resize(frame[0], canvasPart, canvasPart.size(), 0, 0, INTER_AREA);        //把图像缩放到跟canvasPart一样大小

    /*右图像画到画布上*/
    canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分
    resize(frame[1], canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);

    // 显示文字
    if (record) {
        putText(canvas, "recording...", Point(530, 30), FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 255), 2,8,0);
    }

    rec = Mat(canvas);
    return !rec.empty();
}

void saveImg(){
    string imgID;
    while (!stop || !imgLeftCache.empty() || !imgRightCache.empty()){
        if (imgRightCache.empty())
            continue;
        imgID = to_string(imgNum);
        imwrite(path + CAMERA_NAME_LIST[0] + "/" + string(7 - imgID.length(), '0') + imgID + ".jpg",
                Mat(imgLeftCache.front()));
        imwrite(path + CAMERA_NAME_LIST[1] + "/" + string(7 - imgID.length(), '0') + imgID + ".jpg",
                Mat(imgRightCache.front()));

        imgLeftCache.pop();
        imgRightCache.pop();
        imgNum++;
    }
}

void readImgNum(){
    FileStorage fs(path + "imgNum.xml", FileStorage::READ);
    if( fs.isOpened() ){
        fs["imgNum"] >> imgNum;
    }
    else
        cout << "Error: can not open the file.\n";
}

void writeImgNum(){
    FileStorage fs(path + "imgNum.xml", FileStorage::WRITE);
    if( fs.isOpened() ){
        fs << "imgNum" << imgNum;
        fs.release();
    }
    else
        cout << "Error: can not write the file.\n";
}

int main(){
    // 初始化相机
    vector<VideoCapture> capture_list(2);
    for (int i = 0; i < 2; ++i) {
        capture_list[i] = VideoCapture(CAMERA_INDEX_LIST[i]);
        if (!capture_list[i].isOpened()) {
            cout << "open camera error: " << CAMERA_NAME_LIST[i];
            return 0;
        }
        capture_list[i].set(CAP_PROP_FRAME_WIDTH, imgSize.width);
        capture_list[i].set(CAP_PROP_FRAME_HEIGHT, imgSize.height);
        capture_list[i].set(CAP_PROP_FPS, 25);
        capture_list[i].set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G')) ;
    }

    int key;
    Mat frame[2], frameShow;
    readImgNum();

    // 运行保存图片线程
    thread saveImgThread(saveImg);

    // 计时开始
    auto up = chrono::steady_clock::now();
    int captureFrameNum = 0;

    while (true) {
        for (int i = 0; i < 2; ++i) {
            capture_list[i] >> frame[i];
            if (frame[i].empty()) {
                cout << "capture empty frame" << endl;
                continue;
            }
        }
        captureFrameNum++;
        getRectifiedView(frame, frameShow);
        imshow("camera", frameShow);
        if (record) {
            imgLeftCache.push(frame[0]);
            imgRightCache.push(frame[1]);
        }

        key = waitKey(1);
        switch (key) {
            case ESC_KEY:
                record = !record;
                stop = true;
                break;
            case 'c':
                record = !record;
                break;
            default:
                break;
        }

        if (stop) {
            break;
        }
    }

    // 计时结束
    auto down = chrono::steady_clock::now();
    auto d = chrono::duration_cast<chrono::milliseconds>(down - up);

    cout << "time = " << d.count() << endl << "frameNum = " << captureFrameNum << endl << "fps = "
         << (double)captureFrameNum / (double)d.count() * 1000 << endl;

    cout << "Storing files...... Don't exit programme!" << endl;
    saveImgThread.join();
    writeImgNum();
    cout << "Done" << endl;

    return 0;
}