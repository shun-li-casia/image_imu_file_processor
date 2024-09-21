/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: click_read_image.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 19/09/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// 全局变量
Mat image, displayImage;
double scale = 1.0;          // 图像缩放比例
int mouseParams[] = {0, 0};  // 存储鼠标点击的位置

void mouseCallback(int event, int x, int y, int flags, void* param) {
  // 检查鼠标滚轮事件
  if ((flags & EVENT_FLAG_CTRLKEY) && (flags & EVENT_FLAG_ALTKEY)) {
    // 滚轮上下滚动，调整图像的缩放比例
    if (event == EVENT_MOUSEWHEEL) {
      double factor = 1.1;
      if (flags > 0)  // 向上滚动
      {
        scale *= factor;
      } else if (flags < 0)  // 向下滚动
      {
        scale /= factor;
      }
      // 缩放图像
      resize(image, displayImage, Size(), scale, scale);
      // 显示更新后的图像
      imshow("Image", displayImage);
    }
  } else if (event == EVENT_LBUTTONDOWN) {
    // 计算原始图像中的坐标位置
    int origX = x / scale;
    int origY = y / scale;

    // 在点击的位置画一个红色的圆点
    Mat tempRGB = displayImage.clone();
    circle(tempRGB, Point(x, y), 5, Scalar(255, 0, 0),
           -1);  // 注意: 在RGB中红色是(255, 0, 0)

    // 在圆点旁边添加像素值的文字提示
    Vec3b pixelValue = image.at<Vec3b>(origY, origX);
    putText(
        tempRGB,
        format("R:%d G:%d B:%d", pixelValue[2], pixelValue[1], pixelValue[0]),
        Point(x + 10, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);

    // 更新全局变量
    mouseParams[0] = origX;
    mouseParams[1] = origY;

    // 显示更新后的图像
    imshow("Image", tempRGB);

    // 打印到控制台
    cout << "Clicked at (" << origX << ", " << origY
         << "), Pixel Value: R:" << (int)pixelValue[2]
         << " G:" << (int)pixelValue[1] << " B:" << (int)pixelValue[0] << endl;
  }
}

int main(int argc, char** argv) {
  // 检查命令行参数
  if (argc != 2) {
    cout << "Usage: " << argv[0] << " <image path>" << endl;
    return -1;
  }

  // 读取图像（以原生格式）
  image = imread(argv[1], IMREAD_UNCHANGED);
  if (!image.data) {
    cout << "Could not open or find the image" << endl;
    return -1;
  }

  // 创建一个用于显示的图像副本，并转换为BGR格式（OpenCV默认显示格式）
  displayImage = image.clone();

  // 创建窗口
  namedWindow("Image", WINDOW_NORMAL);
  cv::imshow("Image", displayImage);
  // 设置鼠标回调函数
  setMouseCallback("Image", mouseCallback, NULL);

  // 等待按键退出
  while (true) {
    if (waitKey(0) >= 0) break;
  }

  // 清理
  destroyAllWindows();
  return 0;
}
