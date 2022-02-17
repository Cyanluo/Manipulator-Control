## [StereoVision ](https://gitee.com/zixiao_bios/computer-vision)

### 单目标定方法

1. 使用 `takePhotoSingle` 拍摄标定数据集，`takePhotoSingle.cpp` 开头描述了使用方法
   + 参数设置同下文 `takePhotoDoubleAfter` 
     + `startIndex` 为第一张照片文件名的序号，用以多次拍摄同一摄像头标定图片时追加图片，而不是覆盖之前的图片
   + 建议至少拍摄 40 张照片，方法见下文
   + 应为每个物理摄像头分配唯一的 `CAMERA_NAME`，以方便区分和复用摄像头内参文件
   + 按 ESC 退出本次拍摄
2. 使用 `calibration` 计算摄像头参数，对应 `calibration.cpp` 文件
   + 参数设置同下文 `stereoCalibration`
   + `takePhotoSingle.cpp:68` 与 `calibration.cpp:51` 要对应到同一目录下
   + 计算完成后，会使用本次计算的参数校正最后一张标定图片并显示，应观察畸变校正效果是否理想，对应操作同下文



---

### 双目标定方法

1. 确认两相机光轴平行，且左边为 `camera1`，右边为 `camera2`

2. 使用 `takePhotoDoubleAfter` 拍摄标定数据集，`takePhotoDoubleAfter.cpp` 开头描述了使用方法

   + 参数说明
     + `CAMERA_NAME_LIST` 保持不变
     + `CAMERA_INDEX_LIST` 为系统当前与 `CAMERA_NAME_LIST` 对应的 id，即 `CAMERA_INDEX_LIST[0]` 为左相机 id，`CAMERA_INDEX_LIST[1]` 为右相机 id
     + `boardSize` 为标定板内角点数量，即长、宽的方格数各减 1，按习惯长边写在前
     + `imgSize` 为拍摄图像尺寸，为了精确，使用最大分辨率

   + 按下 c 键后，该方法仅在两相机同时识别到符合 `boardSize` 的标定板时拍摄并保存照片

   + 建议至少拍摄 40 张照片（两相机各 40 张，下同），标定板尽可能遍历所有可能的位置和角度，且在图片中所占比例大于 1/4

3. 使用 `stereoCalibration` 计算双目相机参数，对应 `stereoCalibration.cpp` 文件

   + 参数说明
     + `IMG_NUM` 与上一步拍摄图片数相同
     + `CAMERA_NAME_LIST` 和 `boardSize` 取值与 `takePhotoDoubleAfter` 中相同
     + `squareSize` 自行测量，之后可根据测距的偏差修改
     + 其它保持不变
   + `stereoCalibration.cpp:81` 与 `takePhotoDoubleAfter.cpp:79` 要对应到同一目录下（当前好像有问题，建议都修改到 res 文件夹下）
   + 计算完成后，会使用本次计算的参数校正最后一张标定图片并显示，应观察畸变校正和极线约束效果是否理想
     + 若效果不理想，按 ESC 键舍弃结果并退出，此时应考虑：重拍标定图片、增大标定图片数量、改进参数计算方法
     + 若效果理想，按 S 键保存结果并退出，双目参数文件 `camera1&camera2.xml` 会被覆写，双目标定完成

