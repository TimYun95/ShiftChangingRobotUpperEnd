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

    typedef std::vector< std::pair<double, double> > PairData; // pair=<time,speed>
    void Init(const PairData& vp_data, const Configuration* conf);

    double sgn(double x);

    //输入 控制模型
    void calCon(double time, double speedNow, double brakeOpen0, double accOpen0);
    void calConW(double time, double speedNow, double brakeOpen0, double accOpen0);
    void calConN(double time, double speedNow, double brakeOpen0, double accOpen0);

    //返回 控制的目标开度
    double getconAcc();
    double getconBrake();
    int GetSysControlMethod();
    void GetPIDParams(double *params);
    void GetInitParams(double *params1,double *params2);

    /** 挡位和离合控制 **/
    void plantrace(); // 换挡路径规划
    bool ifreachedshift(const bool ifmanual, const int aimindex); // 是否到达目标挡位
    bool ifreachedshiftprocess(const int startindex, const int aimindex); // 是否到达目标挡位 过程中使用
    bool ifreachedclutch(const bool ifmanual, const int aimindex); // 是否到达目标离合位置
    bool getconSft(double* conparas, const unsigned int round); // 计算下个目标的挡位
    bool getconClh(double* conparas, const unsigned int round); // 计算下个目标的离合
    bool ifreachedatinitial(const double angle_err = 0.2); // 开始运行挡位时的调校 误差相对较小


    /** ----------- **/

public:

    /* <ACD> */

    //修改2/////////////////////////////////////////////////////////////////////
    double systemModelSelect=0;
    double vpdataSize;
    //修改2/////////////////////////////////////////////////////////////////////

    //修改5/////////////////////////////////////////////////////////////////////
    //PairData vp_data;这样好像不对。因为pedalrobot里也有一个vp_data
    void vpdataCarSpeed2();//predef模式用

    //修改12/////////////////////////////////////////////////////////////////////
    void vpdataCarSpeed1();//online模式用
    double sysTimeStart;
    //修改12/////////////////////////////////////////////////////////////////////

    double sysSpeedStart;
    double sysReadCarSpeed;
    int sysCarSelect=0;
    //修改5/////////////////////////////////////////////////////////////////////

    /* </ACD> */

private:
    size_t getIndex(double time);
    void getError();
    void getSpeedDv(double time,double speedNow);
    double changeAcc();
    double changeAccW();
    double changeAccN();
    double changeBrake();
    double changeBrakeW();

    /** 挡位和离合控制 **/
    //      1       3       5 ----------------
    //      |       |       |                 |
    //     N1&2 -- N3&4 -- N5&6 -->节点挡位     | --> 给定挡位
    //      |       |       |                 |
    //      2       4       6 ----------------
    int getnodeformanual(const int s); // 获得给定挡位的节点挡位

    /** ----------- **/

private:
	double pid[2][3];
	double pidA[2][3];

	double border;
	double conAcc[2],conBrake[2];
    double brakeOpen,accOpen;
    double maxAccRecorder,maxAccRefresh,maxAcctimer;
    double brakechangeRefresh;

	PairData vp_data;
	double speed[4],lineSpeed[4];
	double dv[3],lineDv[3];
    double error[3], dverror[3];
    double linefuDv,linefuDv2,linefuDv3,linelastDv,linefuSpeed,linefuSpeed2,linefuSpeed3,linelastSpeed,linelastSpeed3;
    double linefuSpeedLong,linefuDvLong;
    double linefuSpeedSubLong, linefuDvSubLong;
    double timeGap,lastControlTime;
    int isTrain;//0未训练 1训练完后
    size_t indexNow;

    int sysControlMethod;


};

#endif // SYSCONTROL_H
