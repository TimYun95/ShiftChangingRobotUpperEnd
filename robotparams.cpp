#include "robotparams.h"

const unsigned int RobotParams::UIAxisNum = 6; // UI界面支持的轴数

const unsigned int RobotParams::UITimerMs = 18; // 基准定时器 18ms
const unsigned int RobotParams::UITimerMultiplier = 3; // 控制指令发送周期 18*3=54m
const unsigned int RobotParams::updateUIFrequency = 6; // 界面更新周期 18*3*6=324ms
const unsigned int RobotParams::waitForGoHomeRound = 30; // 发送了回原指令后30个界面更新周期后失效

const std::string RobotParams::robotType = "dof_6_autodrive"; // 机器人类型
const std::string RobotParams::robotFolder = "AutoDrivingRobot"; // 默认文件夹名称

const double RobotParams::singleAxisBtnRatio = 2.0; // 单轴运动速度比例系数
const double RobotParams::singleAxisBtnRatioS = 1.0; // 单轴运动速度比例系数 挡位
const double RobotParams::singleAxisBtnRatioC = 2.0; // 单轴运动速度比例系数 离合

const int RobotParams::normalMotionAccuracy = 200; // 简单示教运动精度

double RobotParams::angleRealTime[axisNum] = {1,1,1,1,1,1}; // 实时各轴角度
bool RobotParams::ifGoHome; // 是否已经回原
std::string RobotParams::statusStr; // 程序状态字符串
int RobotParams::statusStrIndex; // 程序状态字符串索引
double RobotParams::accOpenValue; // 油门开度
double RobotParams::brakeOpenValue; // 刹车开度
double RobotParams::canCarSpeed; // CAN口读取的车速
double RobotParams::pulseCarSpeed; // PULSE计算的车速
double RobotParams::powerMode = 2; // 上电状态

bool RobotParams::askGoHomeatstart = false; // 是否已经在程序开始时询问了回原信息
unsigned int RobotParams::askGoHomeatstartresult = 0; // 在开始询问回原信息的结果
bool RobotParams::readyToOrigin = false; // 准备执行回原文件

int RobotParams::shiftclutchIndex = 0; // 换挡索引
unsigned int RobotParams::shiftIndex = 0; // 挡位索引
unsigned int RobotParams::clutchIndex = 0; // 离合索引

bool RobotParams::isEmergencyStopNow = false; // 是否急停
bool RobotParams::isEmergencySuccess = false; // 急停是否成功
