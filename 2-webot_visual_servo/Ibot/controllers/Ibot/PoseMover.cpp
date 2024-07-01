#include "PoseMover.h"

PoseMover::PoseMover(float kRho, float kAlpha, float kBeta)
{
  mKRho = kRho;
  mKAlpha = kAlpha;
  mKBeta = kBeta;
  mReached = false;
}

PoseMover::~PoseMover()
{

}

void PoseMover::Process(cv::Vec3f pose)
{
  // 获取当前位姿与目标位姿间的极坐标
  float xDiff = pose[0] - mGoal[0];
  float yDiff = pose[1] - mGoal[1];
   
  float rho = hypot(xDiff, yDiff);
  float theta = fabs(pose[2] - mGoal[2]);
  
  // 判别是否到达目标位置
  if (rho < DISTANCE_TOLERANCE && theta < ANGLE_TOLERANCE) {
    mV = 0;
    mW = 0;
    mReached = true;
    return;
  } else {
    mReached = false;
  }
  
  // 依据当前位姿和目标位姿间的的差异计算控制律，参考书《Robotics, Vision and Control》书第4.2.4节Moving to a Pose
  float alpha = std::fmod(std::atan2(yDiff, xDiff) - pose[2] + M_PI + (xDiff < 0 ? 1 : 0) * M_PI, 2 * M_PI) - M_PI;
  float beta = std::fmod(mGoal[2] - pose[2] - alpha + M_PI, 2 * M_PI) - M_PI; 
  
  mV = mKRho * rho;
  mW = mKAlpha * alpha - mKBeta * beta;
  
  // 修正线速度方向，大小和角速度大小
  if (xDiff < 0) {
    mV = -mV;
  }
  
  if (fabs(mV) > MAX_LINEAR_SPEED) {
    mV = sign(mV) * MAX_LINEAR_SPEED;
  }
  
  if (fabs(mW) > MAX_ANGULAR_SPEED) {
    mW = sign(mW) * MAX_ANGULAR_SPEED;
  }
}
