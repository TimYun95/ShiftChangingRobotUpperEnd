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

    static bool askGoHomeatstart; // 是否已经在程序开始时询问了回原信息
    static unsigned int askGoHomeatstartresult; // 在开始询问回原信息的结果 0--还没问 1--回原了 100--没回原

    static PairData changeshiftlist; // 换挡时刻表
    static int checkshiftlist; // 换挡时刻表的初始index
    static bool changeshiftstart; // 开始换挡标志位
    static bool changeshiftend; // 刚完成换挡

    static unsigned int round; // 换挡过程控制轮数1
    static unsigned int round2; // 换挡过程控制轮数2
    static unsigned int changeshiftprocess; // 换挡过程进度控制
    static bool startchangeshifttimeflag; // 换挡开始时刻记录标志位
    static timeval starttime; // 换挡开始时刻
    static timeval stoptime; // 换挡中断时刻

    // 手动和自动挡位的值
    static std::string manulShiftValues[9];
    static std::string autoShiftValues[3];

    static double tempVars[10]; // 预留的变量 含义人为赋值

    static bool ifConfirmSC; // 是否确认了挡位离合信息
    static bool ifConfirmCS; // 是否确认了换挡时刻

    static bool isExaming; // 正在测试挡位离合信息

    /**
     * @brief switchflag 切换标志位
     * 0 ---> 切换到曲线运行前的准备状态
     * 1 ---> 准备状态下区分等待和执行
     * 2 ---> NVH状态
     * 3 ---> NVH状态 切换到曲线运行前的准备状态
     * 4 ---> NVH状态 准备状态下区分等待和执行
     * 5 ---> ACD状态
     */
    static bool switchflag[10];

    /**
     * @brief NVHcurvestate NVH状态下运行曲线标志位
     * 0 ---> Moderate optional
     * 1 ---> Moderate required
     * 2 ---> Full Pedal
     * 3 ---> High Gear
     * 4 ---> Ready to Exit
     * 9 ---> No State
     */
    static unsigned int NVHcurvestate;

    static unsigned int NVHcurvestate3state; //NVH状态3下运行状态标志位

    static double nvh_P1t; // 目标点1的时间
    static double nvh_P1v; // 目标点1的速度
    static double nvh_P2t; // 目标点2的时间
    static double nvh_P2v; // 目标点2的速度

    static QTime testingTimes[10]; // 测试用时刻计
    static int testingtimenum[10]; // 测试用时刻计数

    static bool readyToOrigin; // 准备执行回原文件

    static bool ifCSACD; // 是否在ACD模式下换挡
    static bool ifREBA; // 是否在ACD换挡模式下恢复踏板
    static bool iffromNto1; // 是否从空挡换到1挡
};

#endif // ROBOTPARAMS_H
