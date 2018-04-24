#ifndef ROBOTPARAMS_H
#define ROBOTPARAMS_H

#include <string>

class RobotParams
{
public:
    static const unsigned int UIAxisNum; // UI界面支持的轴数

    static const unsigned int UITimerMs; // 基准定时器
    static const unsigned int UITimerMultiplier; // 控制指令发送周期
    static const unsigned int updateUIFrequency; // 界面更新周期
    static const unsigned int axisNum = 6; // 共6轴可动 刹车1个 油门1个 挡位2个 离合1个 （方向盘1个）

    static const std::string robotType; // 机器人类型
    static const std::string robotFolder; // 默认文件夹名称

    static const double singleAxisBtnRatio; // 单轴运动速度比例系数
    static const double singleAxisBtnRatioC; // 单轴运动速度比例系数 离合

    static double angleRealTime[axisNum]; // 实时各轴角度
    static bool ifGoHome; // 是否已经回原
    static std::string statusStr; // 程序状态字符串
    static int statusStrIndex; // 程序状态字符串索引
    static double accOpenValue; // 油门开度
    static double brakeOpenValue; // 刹车开度
    static double canCarSpeed; // CAN口读取的车速
    static double pulseCarSpeed; // PULSE计算的车速
    static double powerMode; // 上电状态

    static int currentshiftindex; // 当前挡位索引
    static std::string currentshiftvalue; // 当前挡位值
    static int lastshiftindex; // 前次挡位索引
    static int aimshiftindex; // 目标挡位索引
    static int shiftrunpath[10]; // 换挡路径索引 经过的挡位字符串
    static int shiftrunlength; // 换挡路径长度 经过的挡位数目
    static int shiftrunpointer; // 换挡路径进行到的状态

    static int currentclutchindex; // 当前离合索引
    static std::string currentclutchvalue; // 当前挡位值
    static int aimclutchindex; // 目标挡位索引

};

#endif // ROBOTPARAMS_H
