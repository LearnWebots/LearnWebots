#include "LineFollower.h"

LineFollower::LineFollower()
{

}

LineFollower::~LineFollower()
{

}

void LineFollower::Process(const cv::Mat& image)
{
  // 图像灰度化和大津阈值分割
  cv::Mat thresh;
  cv::cvtColor(image, mGray, cv::COLOR_BGRA2GRAY);
  cv::threshold(mGray, thresh, 127, 255, cv::THRESH_OTSU);
  
  // 灰度重心法提取线条中心
  float sum = 0, weight = 0;
  for (int j = 400; j < 480; j++) {
    for (int i = 0; i < thresh.cols; i++) {
      if (thresh.at<uchar>(j, i) == 0) {
        float gray = 255 - mGray.at<uchar>(j, i);
        sum += gray * i;
        weight += gray;
      }
    }
  }
  float u = sum / weight;
  
  mPoint = cv::Point(u, (400 + 480) / 2);
  mShow = thresh.clone();
}

float LineFollower::GetPosition()
{
  // 返回线条归一化中心[-1, 1]
  return mPoint.x / (mGray.cols / 2.0f) - 1.0f;
}

cv::Mat LineFollower::GetShow()
{
  // 绘制线条截面灰度分布曲线
  cv::Mat show;
  cv::cvtColor(mShow, show, cv::COLOR_GRAY2BGRA);
  int j = (400 + 480) / 2;
  for (int i = 0; i < mGray.cols; i++) {
    cv::circle(show, cv::Point(i, j - 255 + mGray.at<uchar>(j, i)), 2, cv::Scalar(0, 255, 0, 0), 4);
  }
  cv::circle(show, cv::Point(mPoint), 5, cv::Scalar(0, 255, 0, 0), 10);
  return show;
}
