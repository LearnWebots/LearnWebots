// File:          Ibot.cpp
// Date:          2024/6/18
// Blog:          https://github.com/LearnWebots/LearnWebots.github.io
// Description:   机器人巡线
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
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/display.h>
#include <webots/supervisor.h>

#include "LineFollower.h"
#include "PID.h"
#include "WheelOdometry.h"

/*********************************************************
* Parameter List
**********************************************************/
#define TIME_STEP 32
#define PID_KP 0.8f
#define PID_KI 0.4f
#define PID_KD 0.04f
#define BASE_MOVE_SPEED 8.0f
#define BASE_ROT_SPEED 8.0f
#define MAXIMUM_NUMBER_OF_COORDINATES 2000
#define REFRESH_FACTOR 10
//children->HingeJoint "left wheel"->endPoint->translation
#define WHEEL_RADIUS 0.025f
#define WHEEL_DISTANCE 0.09f

/*********************************************************
* Variable List
**********************************************************/
static WbDeviceTag camera, display, leftMotor, rightMotor, leftSensor, rightSensor, compass, gps;
std::shared_ptr<LineFollower> lineFollower;
std::shared_ptr<PID> pid;
std::shared_ptr<WheelOdometry> wheelOdometry;
cv::Mat image, show;
// 来自Webots的samples中的supervisor_draw_trail
int index_trail = 0;           // This points to the current position to be drawn.
bool trail_first_step = true;  // Only equals to true during the first step.
WbNodeRef trail_target_node;
WbFieldRef trail_point_field, trail_coord_index_field;
// 来自Webots的samples中的supervisor_draw_trail
int index_trace = 0;           // This points to the current position to be drawn.
bool trace_first_step = true;  // Only equals to true during the first step.
WbNodeRef trace_target_node;
WbFieldRef trace_point_field, trace_coord_index_field;

/*********************************************************
* Function List
**********************************************************/
void DeviceAndParaInit();
void DeviceAndParaDeinit();
void create_trail_shape(); // 来自Webots的samples中的supervisor_draw_trail
void create_trace_shape(); // 参考Webots的samples中的supervisor_draw_trail
void WheelOdometryProcess();
void CameraProcess();
void MotorProcess();
void DisplayProcess();
void TrailProcess();
void TraceProcess();

/*********************************************************
* Main Function
**********************************************************/

int main(int argc, char **argv) {
  // robot初始化
  wb_robot_init();
  
  // 设备和参数初始化
  DeviceAndParaInit();

  // 主循环，robot单步运行
  while (wb_robot_step(TIME_STEP) != -1) {
    // WheelOdometry处理函数
    WheelOdometryProcess();
    
    // Camera处理函数
    CameraProcess();
    
    // Motor处理函数
    MotorProcess();
    
    // Display处理函数
    DisplayProcess();
    
    // Trail处理函数
    TrailProcess();
    
    // Trace处理函数
    TraceProcess();
  };
  
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
  
  // 获取gps和compass设备
  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  
  // 获取camera的分辨率：宽和高
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  image = cv::Mat(height, width, CV_8UC4);
  
  // 设定Motor初始值
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);
  wb_motor_set_velocity(leftMotor, 0.0);
  wb_motor_set_velocity(rightMotor, 0.0);
  
  // 使能Position设备
  wb_position_sensor_enable(leftSensor, TIME_STEP);
  wb_position_sensor_enable(rightSensor, TIME_STEP);
  
  // 使能gps和compass设备
  wb_gps_enable(gps, TIME_STEP);
  wb_compass_enable(compass, TIME_STEP);
  
  // 来自Webots的samples中的supervisor_draw_trail
  // Get the target object node, i.e. the TARGET Pose in the E-puck turretSlot field.
  trail_target_node = wb_supervisor_node_get_from_def("SMILE");//TARGET");

  // Create the TRAIL Shape which will contain the green line set.
  create_trail_shape();
  
  // Get the target object node, i.e. the TARGET Pose in the E-puck turretSlot field.
  trace_target_node = wb_supervisor_node_get_from_def("SMILE");//TARGET");

  // Create the TRAIL Shape which will contain the green line set.
  create_trace_shape();
  // 来自Webots的samples中的supervisor_draw_trail
  
  // 获取gps和compass值
  wb_robot_step(TIME_STEP);
  const double *gps_values = wb_gps_get_values(gps);
  const double *compass_values = wb_compass_get_values(compass);
  
  float theta = atan2(1.0, 0.0) - atan2(compass_values[1], compass_values[0]);
  cv::Vec3f initPose = cv::Vec3f(gps_values[0], gps_values[1], theta);
  
  // 新建WheelOdometry智能指针
  wheelOdometry = std::make_shared<WheelOdometry>(WHEEL_RADIUS, WHEEL_DISTANCE, initPose);  
  
  // 新建LineFollower智能指针
  lineFollower = std::make_shared<LineFollower>();
  
  // 新建PID智能指针
  pid = std::make_shared<PID>(PID_KP, PID_KI, PID_KD, TIME_STEP);
}

/*********************************************************
* Function name ：DeviceAndParaDeinit
* Description   ：Webots设备和参数去初始化函数
**********************************************************/

void DeviceAndParaDeinit()
{
  // 释放WheelOdometry智能指针
  wheelOdometry = nullptr; 

  // 释放LineFollower智能指针
  lineFollower = nullptr;
  
  // 释放PID智能指针
  pid = nullptr; 
}

/*********************************************************
* Function name ：WheelOdometryProcess
* Description   ：Webots轮式里程计处理函数
**********************************************************/

void WheelOdometryProcess()
{
  if (leftSensor == 0 || rightSensor == 0) {
    return;
  }
  
  float leftPosition = wb_position_sensor_get_value(leftSensor);
  float rightPosition = wb_position_sensor_get_value(rightSensor);
  
  if (wheelOdometry != nullptr) {
    wheelOdometry->Process(leftPosition, rightPosition);
  }
  
  return;
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

  if (lineFollower == nullptr) {
    return;
  }

  // 获取摄像头数据并转为OpenCV的Mat格式
  const unsigned char *data = wb_camera_get_image(camera);
  memcpy(image.data, data, image.cols * image.rows * 4);
  
  // 处理图像
  lineFollower->Process(image);
  show = lineFollower->GetShow();
  
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
  
  if (pid == nullptr || lineFollower == nullptr) {
    return;
  }
  
  // 将归一化线条中心作为输入获得PID控制器结果
  float pos = pid->Process(lineFollower->GetPosition());
  
  // 根据PID控制器结果设定左右轮速度
  wb_motor_set_velocity(leftMotor, (1 - fabs(pos)) * BASE_MOVE_SPEED + pos * BASE_ROT_SPEED);
  wb_motor_set_velocity(rightMotor, (1 - fabs(pos)) * BASE_MOVE_SPEED - pos * BASE_ROT_SPEED);

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

  // 显示图像处理结果至display
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

/*********************************************************
* Function name ：TraceProcess
* Description   ：Webots里程计轨迹处理函数
**********************************************************/

void TraceProcess()
{
  if (trace_target_node == 0) return;

  static int count = 0;
  if (count++ % 10 != 0) return;

  // Get the current target translation.
  //const double *target_translation = wb_supervisor_node_get_position(trace_target_node);
  cv::Vec3f pose = wheelOdometry->GetPose();
  const double target_translation[3] = {pose[0], pose[1], 0.02f};

  // Add the new target translation in the line set.
  wb_supervisor_field_set_mf_vec3f(trace_point_field, index_trace, target_translation);

  // Update the line set indices.
  if (index_trace > 0) {
    // Link successive indices.
    wb_supervisor_field_set_mf_int32(trace_coord_index_field, 3 * (index_trace - 1), index_trace - 1);
    wb_supervisor_field_set_mf_int32(trace_coord_index_field, 3 * (index_trace - 1) + 1, index_trace);
  } else if (index_trace == 0 && trace_first_step == false) {
    // Link the first and the last indices.
    wb_supervisor_field_set_mf_int32(trace_coord_index_field, 3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1), 0);
    wb_supervisor_field_set_mf_int32(trace_coord_index_field, 3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1) + 1,
                                     MAXIMUM_NUMBER_OF_COORDINATES - 1);
  }
  // Unset the next indices.
  wb_supervisor_field_set_mf_int32(trace_coord_index_field, 3 * index_trace, index_trace);
  wb_supervisor_field_set_mf_int32(trace_coord_index_field, 3 * index_trace + 1, index_trace);

  // Update global variables.
  trace_first_step = false;
  index_trace++;
  index_trace = index_trace % MAXIMUM_NUMBER_OF_COORDINATES;
}

/*********************************************************
* Function name ：create_trace_shape
* Description   ：Webots里程计轨迹绘制函数，参考Webots的samples中的howto下的supervisor_draw_trail
**********************************************************/

// Create the trace shape with the correct number of coordinates.
void create_trace_shape() {
  // If TRACE exists in the world then silently remove it.
  WbNodeRef existing_trace = wb_supervisor_node_get_from_def("TRACE");
  if (existing_trace)
    wb_supervisor_node_remove(existing_trace);

  int i;
  char trace_string[0x10000] = "\0";  // Initialize a big string which will contain the TRACE node.

  // Create the TRACE Shape.
  strcat(trace_string, "DEF TRACE Shape {\n");
  strcat(trace_string, "  appearance Appearance {\n");
  strcat(trace_string, "    material Material {\n");
  strcat(trace_string, "      diffuseColor 1 0 0\n");
  strcat(trace_string, "      emissiveColor 1 0 0\n");
  strcat(trace_string, "    }\n");
  strcat(trace_string, "  }\n");
  strcat(trace_string, "  geometry DEF TRACE_LINE_SET IndexedLineSet {\n");
  strcat(trace_string, "    coord Coordinate {\n");
  strcat(trace_string, "      point [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(trace_string, "      0 0 0\n");
  strcat(trace_string, "      ]\n");
  strcat(trace_string, "    }\n");
  strcat(trace_string, "    coordIndex [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(trace_string, "      0 0 -1\n");
  strcat(trace_string, "    ]\n");
  strcat(trace_string, "  }\n");
  strcat(trace_string, "}\n");

  // Import TRACE and append it as the world root nodes.
  WbFieldRef root_children_field = wb_supervisor_node_get_field(wb_supervisor_node_get_root(), "children");
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, trace_string);
  
  // Get interesting references to the TRACE subnodes.
  WbNodeRef trace_line_set_node = wb_supervisor_node_get_from_def("TRACE_LINE_SET");
  WbNodeRef coordinates_node = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(trace_line_set_node, "coord"));
  trace_point_field = wb_supervisor_node_get_field(coordinates_node, "point");
  trace_coord_index_field = wb_supervisor_node_get_field(trace_line_set_node, "coordIndex");
}
