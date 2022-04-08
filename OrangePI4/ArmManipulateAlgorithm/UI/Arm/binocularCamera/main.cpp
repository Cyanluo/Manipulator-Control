#include "binocularCamera.h"

Point origin;         //鼠标按下的起始点
Rect selection;      //定义矩形选框
bool selectObject = false;    //是否选择对象
Mat xyz;

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

int main()
{
	binocularCamera cap;

	if(!cap.open(2))
	{
		return -1;
	}

	while(true)
	{
		int c = waitKey(1);

		if(char(c) == 'q')
			break;
		Mat srcL, srcR;
		cap.getImage(srcL, srcR);

		Mat deepImg;
		xyz = cap.stereoMatch(srcL, srcR, deepImg);

		Mat merge;
		merge = cap.mergingDisplay(srcL, srcR);

		namedWindow("deepImg", WINDOW_NORMAL);
		//鼠标响应函数setMouseCallback(窗口名称, 鼠标回调函数, 传给回调函数的参数，一般取0)
		setMouseCallback("deepImg", onMouse, 0);//disparity

		imshow("merge", merge);
		imshow("deepImg", deepImg);
	}

	return 0;
}
