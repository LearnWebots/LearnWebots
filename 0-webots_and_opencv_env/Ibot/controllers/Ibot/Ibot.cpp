// File:          Ibot.cpp
// Date:          2024/6/1
// Blog:          https://github.com/LearnWebots/LearnWebots.github.io
// Description:   Camera图像灰度化在Display中显示
// Author:        cpnote
// Modifications:

#include <opencv2/opencv.hpp>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/display.h>

#define TIME_STEP 32

static WbDeviceTag camera, display;
cv::Mat color, gray;
int width, height;

int main(int argc, char **argv) {
  // robot初始化
  wb_robot_init();
  
  // 获取camera和display设备
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  display = wb_robot_get_device("display");

  // 获取camera的分辨率：宽和高
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);

  // 主循环，robot单步运行
  while (wb_robot_step(TIME_STEP) != -1) {
    // 获取摄像头数据并转为OpenCV的Mat格式
    color = cv::Mat(height, width, CV_8UC4);
    const unsigned char *data = wb_camera_get_image(camera);
    memcpy(color.data, data, width * height * 4);
    
    // 图像灰度化和转为RGBA用于显示
    cv::cvtColor(color, gray, cv::COLOR_RGBA2GRAY);
    cv::cvtColor(gray, gray, cv::COLOR_GRAY2RGBA);
    
    // 将OpenCV的Mat转为display可以显示的格式并显示
    WbImageRef imr = wb_display_image_new(display, gray.cols, gray.rows, gray.data, WB_IMAGE_RGBA);
    wb_display_image_paste(display, imr, 0, 0, false);
    wb_display_image_delete(display, imr);
  };

  // robot释放
  wb_robot_cleanup();
  return 0;
}
