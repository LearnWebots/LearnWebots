// File:          Ibot.cpp
// Date:          2024/7/1
// Blog:          https://github.com/LearnWebots/LearnWebots.github.io
// Description:   视觉伺服
// Author:        cpnote
// Modifications:

/*********************************************************
* Include List
**********************************************************/
#include <opencv2/opencv.hpp>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/display.h>
#include <webots/supervisor.h>

#include "PoseEstimator.h"
#include "PoseMover.h"

/*********************************************************
* Parameter List
**********************************************************/
#define TIME_STEP 32
#define MARKER_LENGTH (0.5f * 0.8f)
#define MAXIMUM_NUMBER_OF_COORDINATES 2000
#define REFRESH_FACTOR 10
//children->HingeJoint "left wheel"->endPoint->translation
#define WHEEL_RADIUS 0.025f
#define WHEEL_DISTANCE 0.09f
#define POSE_MOVER_KRHO 2.25f
#define POSE_MOVER_KALPHA 5.0f
#define POSE_MOVER_KBETA 1.0f
#define ARUCO_ID 0

/*********************************************************
* Variable List
**********************************************************/
static WbDeviceTag camera, display, leftMotor, rightMotor, leftSensor, rightSensor;
std::shared_ptr<PoseEstimator> poseEstimator;
std::shared_ptr<PoseMover> poseMover;
cv::Mat image, show;
bool inCenter = true;
// 来自Webots的samples中的supervisor_draw_trail
int index_trail = 0;           // This points to the current position to be drawn.
bool trail_first_step = true;  // Only equals to true during the first step.
WbNodeRef trail_target_node;
WbFieldRef trail_point_field, trail_coord_index_field;
// 来自Webots的samples中的supervisor_draw_trail
int goalIndex = 0;
std::vector<cv::Vec2f> goals = {
{1.1f, 0.0f},
{2.1f, 0.1f},
{1.0f, 0.0f},
{2.0f, -0.1f},
{0.9f, 0.0f},
{2.2f, 0.0f},
};

/*********************************************************
* Function List
**********************************************************/
void DeviceAndParaInit();
void DeviceAndParaDeinit();
void create_trail_shape(); // 来自Webots的samples中的supervisor_draw_trail
void CameraProcess();
void MotorProcess();
void DisplayProcess();
void TrailProcess();

/*********************************************************
* Main Function
**********************************************************/

int main(int argc, char **argv) {
  // robot初始化
  wb_robot_init();
  
  // 设备和参数初始化
  DeviceAndParaInit();

  // 主循环，设定目标位置
  while (true) {
    // 设定Robot目标位置
    int index = (goalIndex++) % goals.size();
    float rho = goals[index][0];
    float theta = goals[index][1];
    poseMover->SetGoal(cv::Vec3f(rho * cos(theta), rho * sin(theta), theta));
  
    // 主循环，robot单步运行
    while (wb_robot_step(TIME_STEP) != -1) {
      // 判断Robot是否达到目标位置
      if (poseMover->GetReached()) {
        break;
      }
      
      // Camera处理函数
      CameraProcess();      
    
      // Motor处理函数
      MotorProcess();
    
      // Display处理函数
      DisplayProcess();
    
      // Trail处理函数
      TrailProcess();
    }  
  }
  
  // 设备和参数去初始化
  DeviceAndParaDeinit();

  // robot释放
  wb_robot_cleanup();
  return 0;
}

/*********************************************************
* Function name ：DeviceAndParaInit
* Description   ：Webots设备和参数初始化函数
**********************************************************/

void DeviceAndParaInit()
{
  // 获取camera和display设备
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  display = wb_robot_get_device("display");
  
  // 获取motor设备
  leftMotor = wb_robot_get_device("left wheel motor");
  rightMotor = wb_robot_get_device("right wheel motor");
  
  // 获取Position设备
  leftSensor = wb_robot_get_device("left wheel sensor");
  rightSensor = wb_robot_get_device("right wheel sensor");
  
  // 获取camera的分辨率：宽和高
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  float fov = wb_camera_get_fov(camera);
  image = cv::Mat(height, width, CV_8UC4);
  
  // 获取camera的光心和焦距
  double cx = width / 2.0;
  double cy = height / 2.0;
  double f = (width / 2.0) / tan(fov / 2.0);
  
  // 设定camera的内参数
  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cameraMatrix.at<double>(0, 0) = f;
  cameraMatrix.at<double>(0, 2) = cx;
  cameraMatrix.at<double>(1, 1) = f;
  cameraMatrix.at<double>(1, 2) = cy;
  cv::Mat distCoeffs = cv::Mat::zeros(1, 4, CV_64F);
  
  // 设定Motor初始值
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);
  wb_motor_set_velocity(leftMotor, 0.0);
  wb_motor_set_velocity(rightMotor, 0.0);
  
  // 使能Position设备
  wb_position_sensor_enable(leftSensor, TIME_STEP);
  wb_position_sensor_enable(rightSensor, TIME_STEP);
  
  // 来自Webots的samples中的supervisor_draw_trail
  // Get the target object node, i.e. the TARGET Pose in the E-puck turretSlot field.
  trail_target_node = wb_supervisor_node_get_from_def("SMILE");//TARGET");

  // Create the TRAIL Shape which will contain the green line set.
  create_trail_shape();
  // 来自Webots的samples中的supervisor_draw_trail
  
  // 新建PoseEstimator智能指针
  poseEstimator = std::make_shared<PoseEstimator>(cameraMatrix, distCoeffs, MARKER_LENGTH);
  
  // 新建PoseMover智能指针
  poseMover = std::make_shared<PoseMover>(POSE_MOVER_KRHO, POSE_MOVER_KALPHA, POSE_MOVER_KBETA);
}

/*********************************************************
* Function name ：DeviceAndParaDeinit
* Description   ：Webots设备和参数去初始化函数
**********************************************************/

void DeviceAndParaDeinit()
{
  // 释放PoseEstimator智能指针
  poseEstimator = nullptr;
  
  // 释放PoseMover智能指针
  poseMover = nullptr;  
}

/*********************************************************
* Function name ：CameraProcess
* Description   ：Webots摄像头处理函数
**********************************************************/

void CameraProcess()
{
  if (camera == 0) {
    return;
  }

  if (poseEstimator == nullptr) {
    return;
  }

  // 获取摄像头数据并转为OpenCV的Mat格式
  const unsigned char *data = wb_camera_get_image(camera);
  memcpy(image.data, data, image.cols * image.rows * 4);
  
  // 处理图像
  poseEstimator->Process(image);
  show = poseEstimator->GetShow();
  
  return;
}

/*********************************************************
* Function name ：MotorProcess
* Description   ：Webots电机处理函数
**********************************************************/

void MotorProcess()
{
  if (leftMotor == 0 || rightMotor == 0) {
    return;
  }
  
  if (poseMover == nullptr || poseEstimator == nullptr) {
    return;
  }
  
  cv::Point2f center = poseEstimator->GetCenter(ARUCO_ID);
  float pos = center.x / (image.cols / 2.0f) - 1.0f;
  
  // 判别aruco是否在图像便中间的位置
  if (inCenter) {
    inCenter = fabs(pos) < 0.7f ? true : false;  
  
    // 获取aruco的位姿信息
    cv::Vec3f pose;
    if (poseEstimator->GetPose(ARUCO_ID, pose)) {
      // 根据aruco位姿获取线速度和角速度 
      poseMover->Process(pose);
  
      float v = poseMover->GetLinearSpeed();
      float w = poseMover->GetAngularSpeed();
  
      // 设定左右轮速度
      float leftSpeed = (2 * v - WHEEL_DISTANCE * w);
      float rightSpeed = (2 * v + WHEEL_DISTANCE * w);
  
      wb_motor_set_velocity(leftMotor, leftSpeed);
      wb_motor_set_velocity(rightMotor, rightSpeed);
    }
  } else {
    // aruco偏离图像，则移动其至图像中心
    inCenter = fabs(pos) < 0.05f ? true : false; 
  
    wb_motor_set_velocity(leftMotor, pos * 2.0);
    wb_motor_set_velocity(rightMotor, - pos * 2.0);
  }
  


  return;
}

/*********************************************************
* Function name ：DisplayProcess
* Description   ：Webots显示处理函数
**********************************************************/

void DisplayProcess()
{
  if (display == 0) return;
  
  if (show.empty()) return;

  WbImageRef imr = wb_display_image_new(display, show.cols, show.rows, show.data, WB_IMAGE_BGRA);
  wb_display_image_paste(display, imr, 0, 0, false);
  wb_display_image_delete(display, imr);
}

/*********************************************************
* Function name ：TrailProcess
* Description   ：Webots轨迹处理函数
**********************************************************/

void TrailProcess()
{
  if (trail_target_node == 0) return;

  static int count = 0;
  if (count++ % 10 != 0) return;

  // Get the current target translation.
  const double *target_translation = wb_supervisor_node_get_position(trail_target_node);

  // Add the new target translation in the line set.
  wb_supervisor_field_set_mf_vec3f(trail_point_field, index_trail, target_translation);

  // Update the line set indices.
  if (index_trail > 0) {
    // Link successive indices.
    wb_supervisor_field_set_mf_int32(trail_coord_index_field, 3 * (index_trail - 1), index_trail - 1);
    wb_supervisor_field_set_mf_int32(trail_coord_index_field, 3 * (index_trail - 1) + 1, index_trail);
  } else if (index_trail == 0 && trail_first_step == false) {
    // Link the first and the last indices.
    wb_supervisor_field_set_mf_int32(trail_coord_index_field, 3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1), 0);
    wb_supervisor_field_set_mf_int32(trail_coord_index_field, 3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1) + 1,
                                     MAXIMUM_NUMBER_OF_COORDINATES - 1);
  }
  // Unset the next indices.
  wb_supervisor_field_set_mf_int32(trail_coord_index_field, 3 * index_trail, index_trail);
  wb_supervisor_field_set_mf_int32(trail_coord_index_field, 3 * index_trail + 1, index_trail);

  // Update global variables.
  trail_first_step = false;
  index_trail++;
  index_trail = index_trail % MAXIMUM_NUMBER_OF_COORDINATES;
}

/*********************************************************
* Function name ：create_trail_shape
* Description   ：Webots轨迹绘制函数，来自Webots的samples中的howto下的supervisor_draw_trail
**********************************************************/

// Create the trail shape with the correct number of coordinates.
void create_trail_shape() {
  // If TRAIL exists in the world then silently remove it.
  WbNodeRef existing_trail = wb_supervisor_node_get_from_def("TRAIL");
  if (existing_trail)
    wb_supervisor_node_remove(existing_trail);

  int i;
  char trail_string[0x10000] = "\0";  // Initialize a big string which will contain the TRAIL node.

  // Create the TRAIL Shape.
  strcat(trail_string, "DEF TRAIL Shape {\n");
  strcat(trail_string, "  appearance Appearance {\n");
  strcat(trail_string, "    material Material {\n");
  strcat(trail_string, "      diffuseColor 0 1 0\n");
  strcat(trail_string, "      emissiveColor 0 1 0\n");
  strcat(trail_string, "    }\n");
  strcat(trail_string, "  }\n");
  strcat(trail_string, "  geometry DEF TRAIL_LINE_SET IndexedLineSet {\n");
  strcat(trail_string, "    coord Coordinate {\n");
  strcat(trail_string, "      point [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(trail_string, "      0 0 0\n");
  strcat(trail_string, "      ]\n");
  strcat(trail_string, "    }\n");
  strcat(trail_string, "    coordIndex [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(trail_string, "      0 0 -1\n");
  strcat(trail_string, "    ]\n");
  strcat(trail_string, "  }\n");
  strcat(trail_string, "}\n");

  // Import TRAIL and append it as the world root nodes.
  WbFieldRef root_children_field = wb_supervisor_node_get_field(wb_supervisor_node_get_root(), "children");
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, trail_string);
  
  // Get interesting references to the TRAIL subnodes.
  WbNodeRef trail_line_set_node = wb_supervisor_node_get_from_def("TRAIL_LINE_SET");
  WbNodeRef coordinates_node = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(trail_line_set_node, "coord"));
  trail_point_field = wb_supervisor_node_get_field(coordinates_node, "point");
  trail_coord_index_field = wb_supervisor_node_get_field(trail_line_set_node, "coordIndex");
}
