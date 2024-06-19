#ifndef WHEEL_ODOMETRY_H
#define WHEEL_ODOMETRY_H

#include <opencv2/opencv.hpp>

struct WheelPosition
{
  float pos = 0.0f;         // 当前位置
  float lastPos = 0.0f;     // 上一次的位置
  float distance = 0.0f;    // 换算距离
};

class WheelOdometry
{
public:
  WheelOdometry(float wheelRadius, float wheelDistance, cv::Vec3f initPose);
  ~WheelOdometry();
  
  void Process(float leftPosition, float rightPosition);
  cv::Vec3f GetPose() { return mPose; }

private:
  float mWheelRadius;
  float mWheelDistance;
  WheelPosition mPosition[2];
  cv::Vec3f mPose;
  bool mFirst;
};

#endif // WHEEL_ODOMETRY_H