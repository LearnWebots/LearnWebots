#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class PoseEstimator
{
public:
  PoseEstimator(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, float markerLength);
  ~PoseEstimator();
  
  void Process(const cv::Mat& image);
  cv::Mat GetShow();
  bool GetPose(int id, cv::Vec3f& pose);
  cv::Point2f GetCenter(int id);

private:
  cv::Vec3d RotationMatrixToEulerAngles(cv::Mat& pose);

private:
  cv::aruco::ArucoDetector mArucoDetector;
  cv::aruco::Dictionary mDictionary;
  cv::aruco::DetectorParameters mParameters;
  
  cv::Mat mCameraMatrix;
  cv::Mat mDistCoeffs;
  float mMarkerLength;
  
  std::vector<int> mIDs;
  std::vector<std::vector<cv::Point2f>> mCorners;
  std::vector<cv::Vec3d> mRvecs;
  std::vector<cv::Vec3d> mTvecs;  
  std::vector<cv::Vec3d> mEulers;
  std::vector<cv::Vec3d> mPositions;
  
  cv::Mat mShow;
};

#endif // POSE_ESTIMATOR_H