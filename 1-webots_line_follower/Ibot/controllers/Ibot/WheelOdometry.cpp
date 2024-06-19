#include "WheelOdometry.h"

WheelOdometry::WheelOdometry(float wheelRadius, float wheelDistance, cv::Vec3f initPose)
{
  mWheelRadius = wheelRadius;
  mWheelDistance = wheelDistance;
  mPose = initPose;
  mFirst = true;
}

WheelOdometry::~WheelOdometry()
{

}

void WheelOdometry::Process(float leftPosition, float rightPosition)
{
  if (mFirst) {
    mPosition[0].lastPos = leftPosition;
    mPosition[1].lastPos = rightPosition;
    mFirst = false;
    return;
  }

  mPosition[0].pos = leftPosition;
  mPosition[1].pos = rightPosition;
  
  // 获得编码器前后帧差值
  for (int i = 0; i < 2; i++) {
    float diff = mPosition[i].pos - mPosition[i].lastPos;
    if (fabs(diff) < 0.001) {
      diff = 0;
      mPosition[i].pos = mPosition[i].lastPos;
    }
    mPosition[i].distance = diff * mWheelRadius;
  }
  
  // 计算机器人的线速度和角速度
  float v = (mPosition[0].distance + mPosition[1].distance) / 2.0;
  float w = (mPosition[1].distance - mPosition[0].distance) / mWheelDistance;
  
  // 更新机器人位姿pose
  float dt = 1.0f;
  mPose[2] += (w * dt);
    
  float vx = v * cos(mPose[2]);
  float vy = v * sin(mPose[2]);
    
  mPose[0] += (vx * dt);
  mPose[1] += (vy * dt);
  
  mPosition[0].lastPos = mPosition[0].pos;
  mPosition[1].lastPos = mPosition[1].pos;
}
