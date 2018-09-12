#ifndef SYSCONTROLSC_H
#define SYSCONTROLSC_H

#include <vector>
#include <sys/time.h>
#include <math.h>
#include "printf.h"

class syscontrolsc {
public:
    syscontrolsc();
    ~syscontrolsc();

    enum class gearstatus {
        Manual = 0,
        Auto
    };

    enum class clutchstatus {
        Released = 0,
        Pressed,
        SlowlyReleasing,
        SlowlyReleasingAtDeparture
    };

    enum class manualshiftstatus {
        Gear_N = 0,
        Gear_1,
        Gear_2,
        Gear_3,
        Gear_4,
        Gear_5,
        Gear_NLeft,
        Gear_NRight,
        Gear_NBack,
        Gear_6,
        Gear_R
    };

    enum class autoshiftstatus {
        Gear_P = 0,
        Gear_N,
        Gear_D,
        Gear_R
    };

    enum class shiftchangingmode {
        OnlyClutch = 0,
        OnlyShift,
        ThreeAxis,
        FiveAxis,

        AutoShift = 9,
        ManualShift
    };

    enum class shiftchangingstatus {
        Idle = 0,
        ClutchPressing,
        ShiftChanging,
        ClutchReleasing,
        ClutchReleasingSlowly,
        ClutchReleasingSlowlyWithPedalControl,
        ClutchReleasingSlowlyWithPedalRecovery
    };

    enum class clutchreleasingmode {
        Normal = 0,
        Slowly,
        SlowlyWithControl,
        SlowlyWithRecovery
    };

    typedef std::pair<double, double> doublecouple;
    typedef std::vector< doublecouple > doublepair;
    typedef std::vector< std::pair<shiftchangingstatus, int> > changingpair;
    typedef std::vector< manualshiftstatus > manualshiftstatusvector;
    typedef std::vector< double > doublevector;


private:
    gearstatus GearStatus; // 变速箱是手动还是自动
    clutchstatus ClutchStatus; // 离合是踩着还是松开
    manualshiftstatus ManualShiftStatus; // 手动挡挡位
    autoshiftstatus AutoShiftStatus; // 自动挡挡位
    shiftchangingmode ShiftChangingMode; // 换挡模式
    shiftchangingstatus ShiftChangingStatus; // 换挡过程状态
    shiftchangingstatus LastShiftChangingStatus; // 上次换挡过程状态
    clutchreleasingmode ClutchReleasingMode; // 离合上抬方式

    doublepair shiftpositions; // 挡位位置记录
    double clutchpositons[2]; // 离合位置记录
    double angletolerance[3]; // 运动角度公差
    double percenttolerance; // 运动百分比公差
    double curvemotionspeed[3]; // 曲线运动速度
    double slowlyreleasingclutchspeedatdeparture; // 起步换挡的离合缓抬速度
    double slowlyreleasingclutchspeed; // 离合缓抬速度
    double accstartangle; // 起步换挡最终的油门位置
    double pedalrecoverypercent[2]; // 踏板恢复百分比
    double pedalrecoveryrecord[2]; // 踏板恢复记录

    changingpair shiftchanginglist; // 单次换挡表
    int shiftchanginglength; // 单次换挡表长度
    int shiftchangingpointer; // 单次换挡过程进度指针
    int currentshiftindex; // 当前挡位索引
    int aimshiftindex; // 目标（下次）挡位索引
    int currentclutchindex; // 当前离合索引
    int aimclutchindex; // 目标（下次）离合索引
    unsigned int round; // 单次换挡中间控制轮数
    bool isshiftchangingplanned; // 换挡次序已经规划完成

    timeval sectionstarttime; // 换挡中间过程一小段开始时刻
    timeval sectionstoptime; // 换挡中间过程一小段结束时刻
    double sectionsusingtime[5]; // 换挡过程时间记录

public:
    bool settingshiftchanginginfos(bool isManualGear,
            doublepair teachedshiftpositions,
            double teachedclutchpositions[2],
            double givenangletolerance[3],
            double givenpercenttolerance,
            double givencurvemotionspeed[3],
            double givenslowlyreleasingclutchspeed[2],
            double givenaccstartangle,
            double givenpedalrecoverypercent[2]);

    bool resetshiftchangingprocess(bool isManualGear);

    /**
     * 返回vector说明
     * vector[0] --> 表示换挡的进展 0为换挡完成 -1为换挡出错 其他数值表示各种状态
     * vector[1~6] --> 表示各轴的绝对控制量
     * vector[7] --> 表示当前所处的挡位
     * vector[8] --> 表示当前离合的状态
     * vector[9] --> 表示从踩离合到换完挡所用的总时间 单位ms
     * vector[10~14] --> 表示各换挡阶段所用时间 包括抬离合 单位ms
     */
    doublevector getshiftchangingangles(
            double actualangles[6],
            int howtochangeshift,
            double pedallowlimits[2],
            int aimshift = 0,
            int aimclutch = 0,
            int howtoreleaseclutch = 0,
            double* pedalcommand = NULL,
            bool ifpaused = false);



private:
    double sgn(double x);

    //       R ----------- 1 -------- 3 -------- 5 ----------------
    //        |                   |             |               |                           |
    // NBack —— NLeft —— N —— NRight -->节点挡位    | --> 给定挡位
    //                            |             |               |                           |
    //                           2 -------- 4 -------- 6 ----------------
    manualshiftstatus getnodeformanualshift(const manualshiftstatus s); // 获得给定挡位的节点挡位

    bool planshiftchangingsteps(
            int aimshift,
            shiftchangingmode howtochangeshift,
            clutchreleasingmode howtoreleaseclutch,
            clutchstatus aimclutch = clutchstatus::Released);

    manualshiftstatusvector planmanualshiftchangingpartforshifttransfer(manualshiftstatus aimshift);

    bool ifreachedaim(
            bool askclutchornot,
            const int aim,
            double actualangles[6],
            bool isincludepedal = false,
            double* pedalpos = NULL,
            bool ifduringprocess = false);

    double angledistance(doublecouple startangles, doublecouple stopangles);

    doublecouple getinterpshift();

    double getinterpclutch(int modeofreleasing = 0);

    doublevector makefeedbackvector(
            double anglescmd[6],
            bool wrongexist = false);

    double caltimeinterval();










};

#endif // SYSCONTROLSC_H
