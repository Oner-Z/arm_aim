#ifndef __GLOBALPARAM_HPP__
#define __GLOBALPARAM_HPP__

#include "Eigen/Eigen"
#include "MvCameraControl.h"
#include "deque"
#include "opencv2/core.hpp"
#include <cstdint>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#pragma pack(1)
typedef struct
{

  uint8_t head; // 0x71

  // 电控发送的信息
  float yaw;   // 当前车云台的yaw角，单位为弧度制
  float pitch; // 当前车云台的pitch角，单位为弧度制
  float roll;  // 当前车云台的roll角，单位为弧度制
  uint8_t status;
  uint8_t is_far;     // 状态位，/5==0自己为红色，/5==1自己为蓝色，%5==0为自瞄，%5==1为小符，%5==3为大符
  uint8_t armor_flag; // 目标车编号
  float latency;      // 延迟，单位为毫秒
  float bias;
  float distance;
  float pitch_offset;

  // 自喵信息
  float coo_x;       
  float coo_y;
  float empty;
  uint8_t fire_allowance;
  float target_yaw;   // 目标偏航角，单位为弧度制
  float target_pitch; // 目标俯仰角，单位为弧
  float yaw_vel;      // 偏航角速度，单位为弧度每秒
  float pitch_vel;    // 俯仰角速度，单位为弧

  uint16_t crc;
  uint8_t tail; // 0x4C

} MessData_AutoAim;
#pragma pack()

#pragma pack(1)
typedef struct
{ // 打符结构体

  uint8_t head; // 0x71
  // 电控发送的信息
  float yaw;          // 当前车云台的yaw角，单位为弧度制
  float pitch;        // 当前车云台的pitch角，单位为弧度制
  float roll;         // 当前车云台的roll角，单位为弧度制
  uint8_t status;     // 状态位，/5==0自己为红色，/5==0自己为蓝色，%5==0为自瞄，%5==1为小符，%5==3为大符
  uint8_t is_far;
  uint8_t armor_flag; // 目标车编号
  float latency; // 延迟，单位为毫秒
  float bullet_speed;
  float empty1;     
  float empty2;     
  float empty3;     
  float empty4;      
  float empty5; 
  uint8_t empty6;     
  float yaw_a;   // 目标姿态yaw角，单位为弧度制
  float pitch_a;    // 目标姿态pitch角，单位为弧度制   
  float empty7;
  float empty8;
  uint16_t crc;
  uint8_t tail; // 0x4C
} MessData_WM;
#pragma pack()

typedef union
{
  MessData_AutoAim message;
  MessData_WM messageWM;
  char data[64];
} Translator;

enum class ArmorType
{
  SMALL,
  LARGE,
  INVALID
};
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b)
              { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color;
  cv::Point2f top, bottom;
  double length;
  double width;
  float tilt_angle;
};

struct UnsolvedArmor
{
  UnsolvedArmor() = default;
  UnsolvedArmor(const Light &l1, const Light &l2)
  {
    if (l1.center.x < l2.center.x)
    {
      left_light = l1, right_light = l2;
    }
    else
    {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classfication_result;
};

struct Armor
{
  int color;
  int type;
  cv::Point3f center;
  cv::Point3f angle;
  cv::Point2f apex[4];
  double distance_to_image_center;
  Eigen::Vector3d position;
  Eigen::Vector3d position_ypd;
  Eigen::Vector3d position_gimbal_ypd;
  cv::Mat rVec;
  double yaw;
};
struct Armors
{
  std::deque<Armor> armors;
};
struct ArmorObject
{
  cv::Point2f apex[4];
  cv::Rect_<float> rect;
  int cls;
  int color;
  int area;
  float prob;
  std::vector<cv::Point2f> pts;
};
struct WMBlade
{
  // cv::Point2f apex[4];
  // cv::Rect_<float> rect;
  // int cls;
  int color;
  // int area;
  // float prob;
  std::vector<cv::Point2f> apex;
};
enum COLOR
{
  UNKNOWN_COLOR = -1,
  RED = 0,
  BLUE = 1
};
enum TARGET
{
  SMALL_ARMOR = 0,
  BIG_ARMOR = 1
};

enum ATTACK_MODE
{
  ENERGY = 0,
  ARMOR = 1
};

enum SWITCH
{
  OFF = 0,
  ON = 1
};

enum GETARMORMODE
{
  HIERARCHY = 0,
  FLOODFILL = 1
};

/**
 * @brief 全局参数结构体，用于保存所有可以更改的参数或贯穿全局的状态量，请在对应yaml配置文件内修改其具体值，而不要在此文件内修改
 *
 *
 */
struct GlobalParam
{
  double r_yaw_obs = 0.0001;              // 距离噪声与距离的比例因子
  double r_pitch_obs = 0.00001;           // 基础距离噪
  double r_dist = 0.5;                    // 基础角度噪声
  double r_dist_coeff = 5.0;              // 距离噪声系数
  double r_dist_base = 5.0;               // 基础距离噪声
  double side_r_update_noise = 2000.0;    // 侧面灯条校正半径的测量噪声
  double side_match_threshold_mm = 150.0; // 侧面灯条匹配的3D距离阈值
  double side_match_z_threshold_mm = 60.0; // 侧面灯条匹配的高度阈值（比距离阈值更严格）
  // //==============================滤波参数部分==============================//
  // double r_range_factor = 0.00001; // 距离噪声与距离的比例因子
  // double r_range_base = 0.0001;   // 基础距离噪声
  // double r_angle_base = 0.01;   // 基础角度噪声
  // double r_angle_scale = 0.01;  // 角度噪声随距离变化的尺度
  //==============================全局状态量部分==============================//
  int color = BLUE; // 当前颜色
  // int get_armor_mode = FLOODFILL;      // 当前获取armor中心点方法
  // 调试信息，INFO等级的日志是否输出
  int switch_INFO = ON; // 调试信息，ERROR等级的日志是否输出
  int switch_ERROR = ON;
  bool debug_mode = false; // 默认关闭
  //============================信息管理参数================================//
  int message_hold_threshold = 5;
  float fake_pitch = 0.0F;
  float fake_yaw = 0.0F;
  float fake_bullet_v = 25.0F;
  uint8_t fake_status = 5;
  float fake_now_time = 0;
  float fake_predict_time = 0;

  //==============================相机部分==============================//
  int attack_mode = ARMOR;
  // 当前使用的相机序号，只连接一个相机时为0，多个相机在取流时可通过cam_index选择设备列表中对应顺序的相机
  int cam_index = 0;
  //===曝光时间===//
  MV_CAM_EXPOSURE_AUTO_MODE enable_auto_exp = MV_EXPOSURE_AUTO_MODE_OFF;
  float energy_exp_time = 200.0F; // 能量机关曝光时间
  float armor_exp_time = 290.0F;  // 装甲板曝光时间
  float blue_exp_time = 1000;
  float red_exp_time = 1000;
  float windmill_blue_exp_time = 950.0F;
  float windmill_red_exp_time = 950.0F;
  float height = 1080;
  float width = 1440;
  //===白平衡===/
  // 默认为目标为红色的白平衡
  int r_balance = 1500;   // 红色通道
  int g_balance = 1024;   // 绿色通道
  int b_balance = 4000;   // 蓝色通道
  int e_r_balance = 1500; // 红色通道
  int e_g_balance = 1024; // 绿色通道
  int e_b_balance = 4000; // 蓝色通道
  //===以下参数重点参与实际帧率的调节===//
  unsigned int pixel_format = PixelType_Gvsp_BayerRG8; // 图像格式，设置为Bayer RG8，更多图像格式可前往MVS的SDK中寻找
  // 在经过测试之后，包括在官方SDK中没有调整Acquisition Frame Rate Control Enable的参数，不过在MVS软件中调试时此选项关闭后依然可以达到期望帧率
  //===其他参数===//
  MV_CAM_GAIN_MODE enable_auto_gain = MV_GAIN_MODE_OFF;           // 自动相机增益使能
  float gain = 17.0F;                                             // 相机增益值
  float gamma_value = 0.7F;                                       // 相机伽马值，只有在伽马修正开启后有效
  int trigger_activation = 0;                                     // 触发方式，从0至3依次为触发上升沿、下降沿、高电平、低电平
  float frame_rate = 180.0F;                                      // 设置帧率，仅在不设置外触发时起效
  MV_CAM_TRIGGER_MODE enable_trigger = MV_TRIGGER_MODE_OFF;       // 触发模式，ON为外触发，OFF为内触发
  MV_CAM_TRIGGER_SOURCE trigger_source = MV_TRIGGER_SOURCE_LINE0; // 触发源
  // 内参矩阵参数 (从CameraConfig.yaml加载，不要在此处设置默认值)
  double cx = 0.0; //<! cx - 从yaml加载
  double cy = 0.0; //<! cy - 从yaml加载
  double fx = 0.0; //<! fx - 从yaml加载
  double fy = 0.0; //<! fy - 从yaml加载
  // 畸变矩阵参数 (从CameraConfig.yaml加载，不要在此处设置默认值)
  double k1 = 0.0; //<! k1 - 从yaml加载
  double k2 = 0.0; //<! k2 - 从yaml加载
  double k3 = 0.0; //<! k3 - 从yaml加载
  double p1 = 0.0; //<! p1 - 从yaml加载
  double p2 = 0.0; //<! p2 - 从yaml加载
  // 相机到云台的平移向量 (从CameraConfig.yaml加载)
  double vector_x = 0.0; //<! 从yaml加载
  double vector_y = 0.0; //<! 从yaml加载
  double vector_z = 0.0; //<! 从yaml加载
  double camera_tilt_deg = 0.0; //<! 相机相对枪管向下倾斜角度(度)，从yaml加载
  double camera_roll_deg = 0.0; //<! 相机相对枪管横滚补偿角(度)，从yaml加载


  //============================装甲板识别相关参数============================//
  double min_ratio = 0.02;
  double max_ratio = 0.75;
  double max_angle_l = 60.0;
  double min_light_ratio = 0.5;
  double min_small_center_distance = 0.8;
  double max_small_center_distance = 2.9;
  double min_large_center_distance = 3.2;
  double max_large_center_distance = 6.0;
  double max_angle_a = 60.0;
  double num_threshold = 0.8;

  int blue_threshold = 65;
  int red_threshold = 70;

  int grad_max = 100;
  int grad_min = 50;

  //============================孤儿灯条检测参数============================//
  // 孤儿灯条使用更宽松的参数，以便检测侧面灯条
  double orphan_min_ratio = 0.01;   // 孤儿灯条最小宽高比（比装甲板检测更宽松）
  double orphan_max_ratio = 0.9;    // 孤儿灯条最大宽高比（比装甲板检测更宽松）
  double orphan_max_angle = 75.0;   // 孤儿灯条最大倾斜角度
  double orphan_min_length = 8.0;   // 孤儿灯条最小长度
  double orphan_max_area = 15000.0; // 孤儿灯条最大面积

  //============================自瞄相关参数===============================//
  double threshold_low = 250.0;
  int armorStat = 0;
  bool isBigArmor[12] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // double max_match_distance = 0.5;
  // double max_match_yaw_diff = 0.75;
  double cost_threshold = 1000;
  double max_lost_frame = 10;
  double yaw_percent = 0.5;          // 普通车辆开火角度比例
  double outpost_yaw_percent = 0.5;  // 前哨站开火角度比例
  // 卡尔曼滤波相关参数
  double s2qxyz = 250.0;  // 位置转移噪声
  double s2qvx = 0.0;     // vx随机游走噪声
  double s2qvy = 0.0;     // vy随机游走噪声
  double s2qyaw = 90.0;   // 角度转移噪声
  double s2q_vyaw = 0.0;  // vyaw随机游走噪声
  double s2qr = 250.0;    // 半径转移噪声
  double r_xy_factor = 0.032;
  double r_z = 1e-7;
  double r_yaw = 0.016;
  double r_yaw_static = 5.0;
  double r_yaw_static_vyaw_thresh = 0.2; // 当vyaw
  double s2p0xyr = 1000;
  double s2p0yaw = 1;
  double s2p0vyaw = 1000;
  double r_initial = 300;

  // === 双候选EKF重建参数 ===
  int ekf_observe_frames = 15;  // 观察期帧数，在此期间两个候选EKF并行运行

  double r_yaw_corrected = 1;
  double resize = 1;

  // === 新增：用于转速突变判定和赋值 ===
  double yaw_speed_small = 0.5; // 小阈值
  double yaw_speed_large = 6.0; // 赋予的较大初始值

  double small_armor_a = 67.5; // 小装甲板的半长
  double small_armor_b = 28.5; // 小装甲板的半宽
  double big_armor_a = 115.0;  // 大装甲板的半长
  double big_armor_b = 28.5;   // 大装甲板的半宽

  //===============前哨站参数部分==============//
  // 根据RoboMaster前哨站规格图纸定义
  double outpost_r = 275.0;           // 前哨站转轴半径 (mm)，直径φ550
  double outpost_armor_pitch = 15.0;  // 装甲板俯仰角 (度)，75°向外倾斜 = +15°
  // 三个装甲板的高度偏移量 (相对于中心z的偏移，单位mm)
  // 根据图纸: 装甲板0在中间, 装甲板1较低, 装甲板2较高
  double outpost_z_offset_0 = 0.0;    // 装甲板0 (0°位置) 的z偏移
  double outpost_z_offset_1 = -102.0; // 装甲板1 (120°位置) 的z偏移 (1216-1318=-102)
  double outpost_z_offset_2 = 102.0;  // 装甲板2 (240°位置) 的z偏移 (1318-1216=102)

  // 前哨站EKF专属参数（Q/R与初始协方差）
  double outpost_s2qxy = 5.0e4;        // 车心xy过程噪声强度
  double outpost_s2qz = 2.0e4;         // 三块装甲高度过程噪声强度
  double outpost_s2qyaw = 50.0;        // yaw过程噪声强度
  double outpost_s2qvyaw = 200.0;      // yaw角速度随机游走噪声
  double outpost_s2p0xy = 1000.0;      // 初始协方差：xy
  double outpost_s2p0z = 400.0;        // 初始协方差：z0/z1/z2
  double outpost_s2p0r = 100.0;        // 初始协方差：r1/r2（半径状态）
  double outpost_s2p0yaw = 50.0;       // 初始协方差：yaw
  double outpost_s2p0vyaw = 10.0;      // 初始协方差：vyaw
  double outpost_r_yaw_obs = 1.0e-3;   // 观测噪声：方位角
  double outpost_r_pitch_obs = 1.0e-3; // 观测噪声：俯仰角
  double outpost_r_dist_coeff = 5.0;   // 观测噪声：距离系数
  double outpost_r_dist_base = 50.0;   // 观测噪声：距离基值
  double outpost_r_pose_yaw = 5.0;     // 观测噪声：装甲板朝向角

  // ===== 双EKF竞争重建参数 =====
  int enable_dual_ekf = 1;
  int dual_ekf_observe_frames = 10;
  double dual_ekf_decay = 0.85;
  int dual_ekf_velocity_memory_frames;

  //===============打符识别部分==============//

  //====取图蒙板参数====//
  // 蒙板左上角相对x坐标倍数，范围0～1，TL即Left Top
  float mask_TL_x = 0.125F;
  // 蒙板左上角相对y坐标倍数，范围0～1，TL即Left Top
  float mask_TL_y = 0.0F;
  // 蒙板矩形相对宽度倍数，范围0～1-mask_TL_x
  float mask_width = 0.5F;
  // 蒙板矩形相对高度倍数，范围0～1-mask_TL_y
  float mask_height = 1.0F;
  //====HSV二值化参数====//
  int hmin = 32;  //<! l第一个最小值  84
  int hmax = 255; //<! l第一个最大值   101
  int smin = 0;   //<! s最小值   36
  int smax = 255; //<! s最大值
  int vmin = 0;   //<! v最小值   46
  int vmax = 255; //<! v最大值
  int e_hmin = 0;
  int e_hmax = 20;
  int e_smin = 35;
  int e_smax = 255;
  int e_vmin = 180;
  int e_vmax = 255;
  //====滤波开关====//
  int switch_gaussian_blur = ON;

  //====UI开关====//
  int switch_UI_contours = ON;
  int switch_UI_areas = ON;
  int switch_UI = ON;

  //===============打符识别部分==============//

  int circularityThreshold = 45;
  int medianBlurSize = 3;
  int medianBlurSize_1 = 3;
  int debug = 1;
  int dilationSize = 7;
  int dilationSize_1 = 7;
  int erosionSize = 3;
  int erosionSize_1 = 3;
  int thresholdValue = 108;
  int thresholdValue_1 = 108;
  int thresholdValueBlue = 160;
  int thresholdValueBlue_1 = 160;
  int thresholdValue_for_roi = 150;
  int rect_area_threshold = 2000;
  int circle_area_threshold = 50;

  int target_circle_area_min = 8000;
  int target_circle_area_max = 20000;
  int R_area_min = 800;
  int R_area_max = 2200;

  float length_width_ratio_threshold = 3;
  int minContourArea = 200;

  //===============打符Identify==============//
  int list_size = 280;
  double d_Radius = 100;
  double d_P1P3 = 100;
  double d_RP2 = 100;

  int gap = 0;
  int gap_control = 1;

  double tx_cam2cloud = 0;
  double tx_cam2cloud_1 = 0;
  double ty_cam2cloud = 0;
  double ty_cam2cloud_1 = 0;
  double tz_cam2cloud = 0;
  double tz_cam2cloud_1 = 0;

  double delta_t = 0.10;

  void initGlobalParam(const int color);
  void saveGlobalParam();
};

#endif // __GLOBALPARAM_HPP__