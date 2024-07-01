#include "PoseEstimator.h"

PoseEstimator::PoseEstimator(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, float markerLength)
{
  mDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  mParameters = cv::aruco::DetectorParameters();
  mArucoDetector = cv::aruco::ArucoDetector(mDictionary, mParameters);
  
  mCameraMatrix = cameraMatrix;
  mDistCoeffs = distCoeffs;
  mMarkerLength = markerLength;
}

PoseEstimator::~PoseEstimator()
{

}

void PoseEstimator::Process(const cv::Mat& image)
{
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);

  mIDs.clear();
  mCorners.clear();
  // 检测aruco的角点，识别ID，计算位姿
  mArucoDetector.detectMarkers(gray, mCorners, mIDs);
  cv::aruco::estimatePoseSingleMarkers(mCorners, mMarkerLength, mCameraMatrix, mDistCoeffs, mRvecs, mTvecs);

  cv::cvtColor(image, mShow, cv::COLOR_BGRA2BGR);

  mEulers.clear();
  mPositions.clear();

  if (mIDs.size() > 0) {
    mEulers.resize(mIDs.size());
    mPositions.resize(mIDs.size());
    
    for (unsigned int i = 0; i < mIDs.size(); i++) {
      // 求解位姿矩阵
      cv::Mat rot;
      cv::Rodrigues(mRvecs[i], rot);
    
      cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);
      for (unsigned int row = 0; row < 3; ++row) {
        for (unsigned int col = 0; col < 3; ++col) {
          pose.at<double>(row, col) = rot.at<double>(row, col);
        }
        pose.at<double>(row, 3) = mTvecs[0][row];
      }
      
      // 求解位姿矩阵的逆
      cv::Mat invertPose;
      cv::invert(pose, invertPose, cv::DECOMP_SVD);
      
      mEulers[i] = RotationMatrixToEulerAngles(invertPose);
      mPositions[i][0] = invertPose.at<double>(0, 3);
      mPositions[i][1] = invertPose.at<double>(1, 3);
      mPositions[i][2] = invertPose.at<double>(2, 3);
      
      std::cout << "detect aruco code, id is " << mIDs[i] << ", euler: " << mEulers[i] << ", pos: " << mPositions[i] << std::endl;    
    }    
  }
}

bool PoseEstimator::GetPose(int id, cv::Vec3f& pose)
{
  // 获取对应ID的aruco位姿
  for (unsigned int i = 0; i < mIDs.size(); i++) {
    if (id == mIDs[i]) {
      pose[0] = mPositions[i][2];
      pose[1] = mPositions[i][0];
      pose[2] = mEulers[i][1];
      return true;
    }
  }
  
  return false;
}

cv::Point2f PoseEstimator::GetCenter(int id) {
  // 获取对应ID的aruco图像中心坐标
  cv::Point2f center(0, 0);
  for (unsigned int i = 0; i < mIDs.size(); i++) {
    if (id == mIDs[i]) {
      for (unsigned int k = 0; k < mCorners[i].size(); k++) {
        center += mCorners[i][k];
      }
      center /= (float)mCorners[i].size();
      break;
    }
  }
  return center;
}

cv::Mat PoseEstimator::GetShow()
{
  cv::Mat show;
  if (mRvecs.size() > 0) {
    for (unsigned int i = 0; i < mRvecs.size(); i++) {
      cv::drawFrameAxes(mShow, mCameraMatrix, mDistCoeffs, mRvecs[i], mTvecs[i], 0.4f, 8);    
    }
  }
  cv::cvtColor(mShow, show, cv::COLOR_BGR2BGRA);
  return show;
}

cv::Vec3d PoseEstimator::RotationMatrixToEulerAngles(cv::Mat& pose)
{
  // 旋转矩阵转为欧拉角
  double sy = sqrt(pose.at<double>(0,0) * pose.at<double>(0,0) +  pose.at<double>(1,0) * pose.at<double>(1,0) );
  bool singular = sy < 1e-6;
  double x, y, z;

  if (!singular) {
    x = atan2(pose.at<double>(2,1) , pose.at<double>(2,2));
    y = atan2(-pose.at<double>(2,0), sy);
    z = atan2(pose.at<double>(1,0), pose.at<double>(0,0));
  } else {
    x = atan2(-pose.at<double>(1,2), pose.at<double>(1,1));
    y = atan2(-pose.at<double>(2,0), sy);
    z = 0;
  }

  return cv::Vec3d(x, y, z);
}
