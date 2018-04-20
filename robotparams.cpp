#include "robotparams.h"

const unsigned int RobotParams::UIAxisNum = 6; // UI界面支持的轴数

const unsigned int RobotParams::UITimerMs = 18; // 基准定时器 18ms
const unsigned int RobotParams::UITimerMultiplier = 3; // 控制指令发送周期 18*3=54m
const unsigned int RobotParams::updateUIFrequency = 3; // 界面更新周期 18*3*3=112ms

const std::string RobotParams::robotType = "dof_6_autodrive"; // 机器人类型
const std::string RobotParams::robotFolder = "AutoDrivingRobot"; // 默认文件夹名称

const double RobotParams::singleAxisBtnRatio = 1.0; // 单轴运动速度比例系数
const double RobotParams::pedalLiftSpeed = -0.5; // 踏板抬升速度

std::string RobotParams::currentshiftvalue = "/"; // 初始化挡位值
std::string RobotParams::currentclutchvalue = "/"; // 初始化离合值


double RobotParams::angleRealTime[axisNum]; // 实时各轴角度
bool RobotParams::ifGoHome; // 是否已经回原
std::string RobotParams::statusStr; // 程序状态字符串
int RobotParams::statusStrIndex; // 程序状态字符串索引
double RobotParams::accOpenValue; // 油门开度
double RobotParams::brakeOpenValue; // 刹车开度
double RobotParams::canCarSpeed; // CAN口读取的车速
double RobotParams::pulseCarSpeed; // PULSE计算的车速
double RobotParams::powerMode; // 上电状态
int RobotParams::currentshiftindex; // 当前挡位索引
int RobotParams::lastshiftindex; // 前次挡位索引
int RobotParams::aimshiftindex; // 目标挡位索引
int RobotParams::shiftrunpath[10]; // 换挡路径索引 经过的挡位字符串
int RobotParams::shiftrunlength; // 换挡路径长度 经过的挡位数目
int RobotParams::shiftrunpointer; // 换挡路径进行到的状态
int RobotParams::currentclutchindex; // 当前离合索引
