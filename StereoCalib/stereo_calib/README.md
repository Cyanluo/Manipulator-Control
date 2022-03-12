# 实验室双目相机标定及使用

胡子晗 2020.11.30

## 平台 
Ubuntu
## 依赖
opencv3
## 编译
```
工程目录下
mkdir build
cd build
cmake ..
make
```

编译后生成get_Images calib testAccuracy和use四个可执行文件

## 运行

build目录下 

### get_Images

```
./getImages
```

调用双目相机，按<kbd>space</kbd>拍摄，各角度拍摄12*9 20mm的棋盘格标定板约20-30张后按<kbd>q</kbd>退出

左右图会分别保存在`pic/left/`和`pic/right/`下，同时`pic`下生成文件列表`stereo_calib.txt`

### calib

```
./calib
```

按文件列表`stereo_calib.txt`，从`pic/left/`和`pic/right/`下读取图片标定，观察均方根误差RMS，内外参分别写入`intrinsics.yaml`和`	extrinsics.yaml`

双目矫正：
stereoRectify():https://blog.csdn.net/zfjBIT/article/details/94436644

### testAccuracy

```
./testAccuracy
```

利用标定的内外参和畸变矩阵计算一张棋盘格的十字格的三维坐标（相机系下），并输出它们的距离，理想情况应为棋盘格宽度（mm）

### use

```
./use
```

调用双目相机，得到左右目图片矫正后显示

