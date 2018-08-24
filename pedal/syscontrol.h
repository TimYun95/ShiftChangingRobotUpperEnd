#ifndef SYSCONTROL_H
#define SYSCONTROL_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <utility>

#include "configuration.h"

class SysControl
{
public:
	SysControl();
    ~SysControl();
    double sgn(double x);

    typedef std::vector< std::pair<double, double> > PairData; // pair=<time,speed>
    void Init(const PairData& vp_data, const Configuration* conf, const PairData& vp_data_upperbound, const PairData& vp_data_lowerbound);

    //输入 控制模型
    void calCon(double time, double speedNow, double brakeOpen0, double accOpen0);
    void calConW(double time, double speedNow, double brakeOpen0, double accOpen0);

    //返回 控制的目标开度
    double getconAcc();
    double getconBrake();
    int GetSysControlMethod();

private:
    size_t getIndex(double time, const int whichcurve = 0);
    void getError();
    void getSpeedDv(double time,double speedNow);
    double changeAcc();
    double changeAccW();

    double changeBrake();
    double changeBrakeW();

private:
    // NEDC
    double changepoint1to2 = 0.1; // 加速到匀速拐点补偿
    double changepoint2to3 = 30; // 匀速到刹车拐点补偿
    double borderfix = 2; // 寻优起点
    double accpidfix = 1; // 加速度控制增益
    double startupTime = 1;//5
    double border = 0; // 切换border度
    double brakefix = 0.5; // 总刹车增益

    double accfix = 0.5; // 总油门增益

    // WLTC
    double borderfixW = 6; // 寻优起点
    double accpidfixW = 1; // 加速度增益
    double startupTimeW = 0; // 起步提前量
    double wltc_degree_fix = 1; // 刹车到加速补偿
    double wltc_degree_toLow_fix = 1.5; // 加速到刹车补偿
    double wltc_lowborder_fix = 0.8; // 缓速段边界
    double wltc_time_change = 1; // 时间区间选择

	double pid[2][3];
	double pidA[2][3];

    double conAcc, conBrake;
    double brakeOpen, accOpen;
    double maxAccRecorder, maxAccRefresh, maxAcctimer;

    PairData vp_data, vp_data_upperbound, vp_data_lowerbound;
    double speed[4], lineSpeed[4];
    double dv[3], lineDv[3];
    double error[3],  dverror[3];
    double linefuDv, linefuDv2, linefuDv3, linelastDv;
    double linefuSpeed, linelastSpeed;

    double upperboundnow, lowerboundnow;
    double linefuSpeedonesecondreal;
    double linefuSpeedupperboundreal;
    double linefuSpeedlowerboundreal;
    double possibleSpeedafteronesecondbyaccelerationnow;

    double timeGap, lastControlTime;

    int sysControlAnother[4];
};

#endif // SYSCONTROL_H
