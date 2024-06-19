#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <opencv2/opencv.hpp>

class LineFollower
{
public:
  LineFollower();
  ~LineFollower();

  void Process(const cv::Mat& image);
  float GetPosition();
  cv::Mat GetShow();

private:
  cv::Mat mGray;
  cv::Mat mShow;
  cv::Point2f mPoint;
};

#endif // LINE_FOLLOWER_H