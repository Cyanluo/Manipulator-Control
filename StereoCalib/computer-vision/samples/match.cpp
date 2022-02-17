#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <cmath>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

//****这里设置三种颜色来让程序运行的更醒目****
#define WHITE "\033[37m"
#define BOLDRED "\033[1m\033[31m"
#define BOLDGREEN "\033[1m\033[32m"

//****相机内参****
struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx,cy,fx,fy,baseline,scale;
};

struct FILE_FORM
{
    cv::Mat left,right;//存放左右视图数据
    string dispname,depthname,colorname;//存放三种输出数据的文件名
};

//****读取配置文件****
class ParameterReader
{
public:
    ParameterReader(string filename = "./parameters.txt")//配置文件目录
    {
        ifstream fin(filename.c_str());
        if(!fin)
        {
            cerr<<BOLDRED"can't find parameters file!"<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline(fin,str);
            if(str[0] == '#')//遇到’#‘视为注释
            {
                continue;
            }

            int pos = str.find("=");//遇到’=‘将其左右值分别赋给key和alue
            if (pos == -1)
            {
                continue;
            }
            string key = str.substr(0,pos);
            string value = str.substr(pos+1,str.length());
            data[key] = value;

            if (!fin.good())
            {
                break;
            }
        }
    }
    string getData(string key)//获取配置文件参数值
    {
        map<string,string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<BOLDRED"can't find:"<<key<<" parameters!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string,string> data;
};

FILE_FORM readForm(int index,ParameterReader pd);//存入当前序列左右视图数据和三种输出结果文件名
void stereoSGBM(cv::Mat lpng,cv::Mat rpng,string filename,cv::Mat &disp);//SGBM方法获取视差图
void stereoBM(cv::Mat lpng,cv::Mat rpng,string filename,cv::Mat &disp);//BM方法获取视差图
void disp2Depth(cv::Mat disp,cv::Mat &depth,CAMERA_INTRINSIC_PARAMETERS camera);//由视察图计算深度图

int main(int argc, char const *argv[])
{
	ParameterReader pd;//读取配置文件
	CAMERA_INTRINSIC_PARAMETERS camera;//相机内参结构赋值
	camera.fx = atof(pd.getData("camera.fx").c_str());
    camera.fy = atof(pd.getData("camera.fy").c_str());
    camera.cx = atof(pd.getData("camera.cx").c_str());
    camera.cy = atof(pd.getData("camera.cy").c_str());
    camera.baseline = atof(pd.getData("camera.baseline").c_str());
    camera.scale = atof(pd.getData("camera.scale").c_str());

    int startIndex = atoi(pd.getData("start_index").c_str());//起始序列
    int endIndex = atoi(pd.getData("end_index").c_str());//截止序列

    bool is_color = pd.getData("is_color") == string("yes");//判断是否要输出彩色深度图
    cout<<BOLDRED"......START......"<<endl;
    for (int currIndex = startIndex;currIndex < endIndex;currIndex++)//从起始序列循环至截止序列
    {
    	cout<<BOLDGREEN"Reading file： "<<currIndex<<endl;
    	FILE_FORM form = readForm(currIndex,pd);//获取当前序列的左右视图以及输出结果文件名
     	cv::Mat disp,depth,color;

        //****判断使用何种算法计算视差图****
    	if (pd.getData("algorithm") == string("SGBM"))
    	{
    		stereoSGBM(form.left,form.right,form.dispname,disp);
    	}
    	else if (pd.getData("algorithm") == string("BM"))
    	{
    		stereoBM(form.left,form.right,form.dispname,disp);
    	}
    	else
    	{
    		cout<<BOLDRED"Algorithm is wrong!"<<endl;
    		return 0;
    	}

	    disp2Depth(disp,depth,camera);//输出深度图
	    cv::imwrite(form.depthname,depth);
	    cout<<WHITE"Depth saved!"<<endl;

        //****判断是否输出彩色深度图****
	    if (is_color)
	    {
	    	cv::applyColorMap(depth,color,cv::COLORMAP_JET);//转彩色图
	    	cv::imwrite(form.colorname,color);
	    	cout<<WHITE"Color saved!"<<endl;
	    }
    }
    cout<<BOLDRED"......END......"<<endl;

	return 0;
}

FILE_FORM readForm(int index,ParameterReader pd)
{
    FILE_FORM f;
    string lpngDir = pd.getData("left_dir");//获取左视图输入目录名
    string rpngDir = pd.getData("right_dir");//获取右视图输入目录名
    string dispDir = pd.getData("disp_dir");//获取视差图输出目录名
    string depthDir = pd.getData("depth_dir");//获取深度图输出目录名
    string colorDir = pd.getData("color_dir");//获取彩色深度图输出目录名
    string rgbExt = pd.getData("rgb_extension");//获取图片数据格式后缀名

    //输出当前文件序号（使用的TUM数据集，其双目视图命名从000000至004540，详情参看博文末尾ps）
    string numzero;
    if ( index >= 0 && index <= 9 )
    {
        numzero = "00000";
    }
    else if ( index >= 10 && index <= 99 )
    {
        numzero = "0000";
    }
    else if ( index >= 100 && index <= 999 )
    {
        numzero = "000";
    }
    else if ( index >= 1000 && index <= 9999 )
    {
        numzero = "00";
    }
    else if ( index >= 10000 && index <= 99999 )
    {
        numzero = "0";
    }

    //获取左视图文件名
    stringstream ss;
    ss<<lpngDir<<numzero<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.left = cv::imread(filename,0);//这里要获取单通道数据

    //获取右视图文件名
    ss.clear();
    filename.clear();
    ss<<rpngDir<<numzero<<index<<rgbExt;
    ss>>filename;
    f.right = cv::imread(filename,0);//这里要获取单通道数据

    //获取深度图输出文件名
    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<rgbExt;
    ss>>filename;
    f.depthname = filename;

    //获取视差图输出文件名
    ss.clear();
    filename.clear();
    ss<<dispDir<<index<<rgbExt;
    ss>>filename;
    f.dispname = filename;

    //获取彩色深度图输出文件名
    ss.clear();
    filename.clear();
    ss<<colorDir<<index<<rgbExt;
    ss>>filename;
    f.colorname = filename;

    return f;
}

void stereoSGBM(cv::Mat lpng,cv::Mat rpng,string filename,cv::Mat &disp)
{
    disp.create(lpng.rows,lpng.cols,CV_16S);
    cv::Mat disp1 = cv::Mat(lpng.rows,lpng.cols,CV_8UC1);
    cv::Size imgSize = lpng.size();
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create();

    int nmDisparities = ((imgSize.width / 8) + 15) & -16;//视差搜索范围
    int pngChannels = lpng.channels();//获取左视图通道数
    int winSize = 5;

    sgbm->setPreFilterCap(31);//预处理滤波器截断值
    sgbm->setBlockSize(winSize);//SAD窗口大小
    sgbm->setP1(8*pngChannels*winSize*winSize);//控制视差平滑度第一参数
    sgbm->setP2(32*pngChannels*winSize*winSize);//控制视差平滑度第二参数
    sgbm->setMinDisparity(0);//最小视差
    sgbm->setNumDisparities(nmDisparities);//视差搜索范围
    sgbm->setUniquenessRatio(5);//视差唯一性百分比
    sgbm->setSpeckleWindowSize(100);//检查视差连通区域变化度的窗口大小
    sgbm->setSpeckleRange(32);//视差变化阈值
    sgbm->setDisp12MaxDiff(1);//左右视差图最大容许差异
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);//采用全尺寸双通道动态编程算法
    sgbm->compute(lpng,rpng,disp);

    disp.convertTo(disp1,CV_8U,255 / (nmDisparities*16.));//转8位

    cv::imwrite(filename,disp1);
    cout<<WHITE"Disp saved!"<<endl;
}

void stereoBM(cv::Mat lpng,cv::Mat rpng,string filename,cv::Mat &disp)
{
    disp.create(lpng.rows,lpng.cols,CV_16S);
    cv::Mat disp1 = cv::Mat(lpng.rows,lpng.cols,CV_8UC1);
    cv::Size imgSize = lpng.size();
    cv::Rect roi1,roi2;
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16,9);

    int nmDisparities = ((imgSize.width / 8) + 15) & -16;//视差搜索范围

    bm->setPreFilterType(CV_STEREO_BM_NORMALIZED_RESPONSE);//预处理滤波器类型
    bm->setPreFilterSize(9);//预处理滤波器窗口大小
    bm->setPreFilterCap(31);//预处理滤波器截断值
    bm->setBlockSize(9);//SAD窗口大小
    bm->setMinDisparity(0);//最小视差
    bm->setNumDisparities(nmDisparities);//视差搜索范围
    bm->setTextureThreshold(10);//低纹理区域的判断阈值
    bm->setUniquenessRatio(5);//视差唯一性百分比
    bm->setSpeckleWindowSize(100);//检查视差连通区域变化度窗口大小
    bm->setSpeckleRange(32);//视差变化阈值
    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setDisp12MaxDiff(1);//左右视差图最大容许差异
    bm->compute(lpng,rpng,disp);

    disp.convertTo(disp1,CV_8U,255 / (nmDisparities*16.));

    cv::imwrite(filename,disp1);
    cout<<WHITE"Disp saved!"<<endl;
}

void disp2Depth(cv::Mat disp,cv::Mat &depth,CAMERA_INTRINSIC_PARAMETERS camera)
{
        depth.create(disp.rows,disp.cols,CV_8UC1);
        cv::Mat depth1 = cv::Mat(disp.rows,disp.cols,CV_16S);
        for (int i = 0;i < disp.rows;i++)
        {
            for (int j = 0;j < disp.cols;j++)
            {
                if (!disp.ptr<ushort>(i)[j])//防止除0中断
                    continue;
                depth1.ptr<ushort>(i)[j] = camera.scale * camera.fx * camera.baseline / disp.ptr<ushort>(i)[j];
            }
        }
        depth1.convertTo(depth,CV_8U,1./256);//转8位
}
