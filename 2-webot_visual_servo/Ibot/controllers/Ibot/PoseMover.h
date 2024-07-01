#ifndef POSE_MOVER_H
#define POSE_MOVER_H

#include <opencv2/opencv.hpp>

#define MAX_LINEAR_SPEED 9
#define MAX_ANGULAR_SPEED 6
#define DISTANCE_TOLERANCE 0.01
#define ANGLE_TOLERANCE 0.1

class PoseMover
{
public:
  PoseMover(float kRho, float kAlpha, float kBeta);
  ~PoseMover();
  
  void Process(cv::Vec3f pose);
  void SetGoal(cv::Vec3f goal) { mGoal = goal; mReached = false; }
  bool GetReached() { return mReached; }
  float GetLinearSpeed() { return mV; }
  float GetAngularSpeed() { return mW; }
  
private:
  float sign(float value) { return value > 0 ? 1.0f : (value < 0 ? -1.0f : 0.0f); }

private:
  float mKRho, mKAlpha, mKBeta;
  float mV, mW;
  cv::Vec3f mGoal;
  bool mReached;
};

#endif // POSE_MOVER_H