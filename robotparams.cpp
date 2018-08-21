#include "robotparams.h"

const unsigned int RobotParams::UIAxisNum = 6; // UI界面支持的轴数

const unsigned int RobotParams::UITimerMs = 18; // 基准定时器 18ms
const unsigned int RobotParams::UITimerMultiplier = 3; // 控制指令发送周期 18*3=54m
const unsigned int RobotParams::updateUIFrequency = 6; // 界面更新周期 18*3*6=324ms
const unsigned int RobotParams::waitForGoHomeRound = 30; // 发送了回原指令后30个界面更新周期后失效

const std::string RobotParams::robotType = "dof_6_autodrive"; // 机器人类型
const std::string RobotParams::robotFolder = "AutoDrivingRobot"; // 默认文件夹名称

const double RobotParams::singleAxisBtnRatio = 2.0; // 单轴运动速度比例系数
const double RobotParams::singleAxisBtnRatioC = 2.0; // 单轴运动速度比例系数 离合

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
int RobotParams::aimclutchindex; // 目标挡位索引

bool RobotParams::askGoHomeatstart = false; // 是否已经在程序开始时询问了回原信息
unsigned int RobotParams::askGoHomeatstartresult = 0; // 在开始询问回原信息的结果

RobotParams::PairData RobotParams::changeshiftlist; // 换挡时刻表
int RobotParams::checkshiftlist = 0; // 换挡时刻表的初始index

bool RobotParams::changeshiftstart = false; // 开始换挡标志位
bool RobotParams::changeshiftend = false; // 刚完成换挡
unsigned int RobotParams::round = 1; // 换挡过程控制轮数1
unsigned int RobotParams::round2 = 1; // 换挡过程控制轮数2
unsigned int RobotParams::changeshiftprocess = 0; // 换挡过程进度控制
bool RobotParams::startchangeshifttimeflag = false; // 换挡开始时刻记录标志位
timeval RobotParams::starttime; // 换挡开始时刻
timeval RobotParams::stoptime; // 换挡中断时刻

// 手动和自动挡位的值
std::string RobotParams::manulShiftValues[9] = {"N_1&2", "N_3&4", "N_5&6", "1", "2", "3", "4", "5", "6"};
std::string RobotParams::autoShiftValues[3] = {"P", "N", "D"};

double RobotParams::tempVars[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 预留的变量 含义人为赋值

bool RobotParams::ifConfirmSC = false; // 是否确认了挡位离合信息
bool RobotParams::ifConfirmCS = false; // 是否确认了换挡时刻

bool RobotParams::isExaming = false; // 正在测试挡位离合信息

bool RobotParams::switchflag[10] = {false, false, false, false, false, false, false, false, false, false}; // 状态切换
unsigned int RobotParams::NVHcurvestate = 9; //NVH状态下运行曲线标志位
unsigned int RobotParams::NVHcurvestate3state = 0; //NVH状态3下运行状态标志位

double RobotParams::nvh_P1t = 0; // 目标点1的时间
double RobotParams::nvh_P1v = 0; // 目标点1的速度
double RobotParams::nvh_P2t = 0; // 目标点2的时间
double RobotParams::nvh_P2v = 0; // 目标点2的速度

QTime RobotParams::testingTimes[10]; // 测试用时刻计
int RobotParams::testingtimenum[10] = {0,0,0,0,0,0,0,0,0,0}; // 测试用时刻计数

bool RobotParams::readyToOrigin = false; // 准备执行回原文件

bool RobotParams::ifCSACD = false;  // 是否在ACD模式下换挡
bool RobotParams::ifREBA = false; // 是否在ACD换挡模式下恢复踏板
bool RobotParams::iffromNto1 = false; // 是否从空挡换到1挡
