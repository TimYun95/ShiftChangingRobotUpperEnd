#ifndef ROBOTPARAMS_H
#define ROBOTPARAMS_H

#include <QTime>
#include <string>
#include <vector>
#include <sys/time.h>

class RobotParams
{
public:
    typedef std::vector< std::pair<double, int> > PairData; // pair=<time, aimshiftindex>

public:
    static const unsigned int UIAxisNum; // UI界面支持的轴数

    static const unsigned int UITimerMs; // 基准定时器
    static const unsigned int UITimerMultiplier; // 控制指令发送周期
    static const unsigned int updateUIFrequency; // 界面更新周期
    static const unsigned int waitForGoHomeRound; // 发送了回原指令后多少个界面更新周期后失效
    static const unsigned int axisNum = 6; // 共6轴可动 刹车1个 油门1个 挡位2个 离合1个 （方向盘1个）

    static const std::string robotType; // 机器人类型
    static const std::string robotFolder; // 默认文件夹名称

    static const double singleAxisBtnRatio; // 单轴运动速度比例系数
    static const double singleAxisBtnRatioS; // 单轴运动速度比例系数 挡位
    static const double singleAxisBtnRatioC; // 单轴运动速度比例系数 离合

    static const int normalMotionAccuracy; // 简单示教运动精度

    static double angleRealTime[axisNum]; // 实时各轴角度
    static bool ifGoHome; // 是否已经回原
    static std::string statusStr; // 程序状态字符串
    static int statusStrIndex; // 程序状态字符串索引
    static double accOpenValue; // 油门开度
    static double brakeOpenValue; // 刹车开度
    static double canCarSpeed; // CAN口读取的车速
    static double pulseCarSpeed; // PULSE计算的车速
    static double powerMode; // 上电状态

    static bool askGoHomeatstart; // 是否已经在程序开始时询问了回原信息
    static unsigned int askGoHomeatstartresult; // 在开始询问回原信息的结果 0--还没问 1--回原了 100--没回原
    static bool readyToOrigin; // 准备执行回原文件

    static int shiftclutchIndex; // 换挡索引
    static unsigned int shiftIndex; // 挡位索引
    static unsigned int clutchIndex; // 离合索引

    static bool isEmergencyStopNow; // 是否急停
    static bool isEmergencySuccess; // 急停是否成功
};

#endif // ROBOTPARAMS_H
