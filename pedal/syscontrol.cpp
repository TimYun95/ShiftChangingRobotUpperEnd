#include "syscontrol.h"

#include <math.h>
#include <stdio.h>

#include <iostream>
#include <algorithm>
#include <QtMath>

#include <QMessageBox>
#include <QtMath>

#include "printf.h"

SysControl::SysControl() {
    sysControlAnother[0] = 0;
    sysControlAnother[1] = 0;
    sysControlAnother[2] = 0;
    sysControlAnother[3] = 0;
    lastControlTime = 0.0;
    maxAccRefresh = 0.0;

    //油门0
    pid[0][0]= 0.8;
    pid[0][1]= 0.1;
    pid[0][2]= 0.1;

    //刹车1
    pid[1][0]= -1;
    pid[1][1]= -0.5;
    pid[1][2]= -0.1;

    //Apid 油门0
    pidA[0][0]= accpidfix*2;
    pidA[0][1]= accpidfix*2;
    pidA[0][2]= accpidfix*0.1;

    //刹车1
    pidA[1][0]= -1;//改动1：原100
    pidA[1][1]= -0.1;
    pidA[1][2]= -0.1;

}

SysControl::~SysControl() {

}

double SysControl::sgn(double x) {
    if (x > 0.0) return 1;
    else if (x < 0.0) return -1;
    else return 0;
}

void SysControl::Init(const PairData& data, const Configuration* conf, const PairData &upperbound, const PairData &lowerbound) {
    maxAccRefresh = 0;
    maxAcctimer = 0;
    timeGap = 24;
    vp_data = data;
    vp_data_upperbound = upperbound;
    vp_data_lowerbound = lowerbound;
    accfix=0.5;

    //NEDC
    changepoint1to2=conf->sysControlParams[0]; // 加速到匀速拐点补偿
    changepoint2to3=conf->sysControlParams[1]; // 匀速到刹车拐点补偿
    borderfix=conf->sysControlParams[2]/10.0; // 寻优起点设定
    accpidfix=conf->sysControlParams[3]; // 加速度控制增益
    startupTime=conf->sysControlParams[4]; // 起步提前量
    border=conf->sysControlParams[5]; // 切换border度
    brakefix=conf->sysControlParams[6]; // 总刹车增益

    //WLTC
    borderfixW=conf->sysControlParamsWltc[0]/10; // 寻优起点
    accpidfixW=conf->sysControlParamsWltc[1]; // 加速度增益
    startupTimeW=conf->sysControlParamsWltc[2]; // 起步提前量
    wltc_degree_fix=conf->sysControlParamsWltc[3]; // 刹车到加速补偿
    wltc_degree_toLow_fix=conf->sysControlParamsWltc[4]; // 加速到刹车补偿
    wltc_lowborder_fix=conf->sysControlParamsWltc[5]; // 缓速段边界
    wltc_time_change=conf->sysControlParamsWltc[6]; // 时间区间选择

    pid[0][0]= 0.8;
    pid[0][1]= 0.1;
    pid[0][2]= 0.1;
}

void SysControl::getError() {
    error[0] = error[1];
    error[1] = error[2];
    error[2] = lineSpeed[3]- speed[3];
    dverror[0] = dverror[1];
    dverror[1] = dverror[2];
    dverror[2] = lineDv[2]-dv[2];
}

size_t SysControl::getIndex(double t, const int whichcurve) {
    size_t i;
    if (whichcurve == 1)
    {
        for(i = 1; i<vp_data_upperbound.size(); ++i) {
            if (t >= vp_data_upperbound[i].first - vp_data_upperbound[0].first) {
                continue;
            }
            else {
                break;
            }
        }
        if(i >= vp_data_upperbound.size()){
            i = vp_data_upperbound.size() - 1;
        }
    }
    else if (whichcurve == 2)
    {
        for(i = 1; i<vp_data_lowerbound.size(); ++i) {
            if (t >= vp_data_lowerbound[i].first - vp_data_lowerbound[0].first) {
                continue;
            }
            else {
                break;
            }
        }
        if(i >= vp_data_lowerbound.size()){
            i = vp_data_lowerbound.size() - 1;
        }
    }
    else
    {
        for(i = 1; i<vp_data.size(); ++i) {
            if (t >= vp_data[i].first - vp_data[0].first) {
                continue;
            }
            else {
                break;
            }
        }
        if(i >= vp_data.size()){
            i = vp_data.size() - 1;
        }
    }

    return i;//当前在i-1到i段
}

void SysControl::getSpeedDv(double t,double speedNow) {
    size_t i = 0;

    speed[0] = speed[1];
    speed[1] = speed[2];
    speed[2] = speed[3];
    int speedNowIntver=(int)(speedNow*100);//取小数点后2A位
    speed[3] = ((float)speedNowIntver)/100.0;

    dv[0] = dv[1];
    dv[1] = dv[2];
    dv[2] = (speed[3] - speed[2]) / timeGap;

    possibleSpeedafteronesecondbyaccelerationnow = speedNow + dv[2];

    i = getIndex(t);
    lineDv[0] = lineDv[1];
    lineDv[1] = lineDv[2];
    lineDv[2] = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
    lineSpeed[0] = lineSpeed[1];
    lineSpeed[1] = lineSpeed[2];
    lineSpeed[2] = lineSpeed[3];
    lineSpeed[3] = (t - vp_data[i - 1].first)*lineDv[2] + vp_data[i - 1].second;

    i = getIndex(t, 1);
    upperboundnow = (t - vp_data_upperbound[i - 1].first)*(vp_data_upperbound[i].second - vp_data_upperbound[i - 1].second) / (vp_data_upperbound[i].first - vp_data_upperbound[i - 1].first) + vp_data_upperbound[i - 1].second;

    i = getIndex(t, 2);
    lowerboundnow = (t - vp_data_lowerbound[i - 1].first)*(vp_data_lowerbound[i].second - vp_data_lowerbound[i - 1].second) / (vp_data_lowerbound[i].first - vp_data_lowerbound[i - 1].first) + vp_data_lowerbound[i - 1].second;

    i = getIndex(t+1);
    linefuDv = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
    linefuSpeed=vp_data[i].second;
    linefuSpeedonesecondreal = ( (t + 1) - vp_data[i - 1].first)*linefuDv + vp_data[i - 1].second;

    i = getIndex(t+1, 1);
    linefuSpeedupperboundreal = ( (t + 1) - vp_data_upperbound[i - 1].first)*(vp_data_upperbound[i].second - vp_data_upperbound[i - 1].second) / (vp_data_upperbound[i].first - vp_data_upperbound[i - 1].first) + vp_data_upperbound[i - 1].second;

    i = getIndex(t+1, 2);
    linefuSpeedlowerboundreal = ( (t + 1) - vp_data_lowerbound[i - 1].first)*(vp_data_lowerbound[i].second - vp_data_lowerbound[i - 1].second) / (vp_data_lowerbound[i].first - vp_data_lowerbound[i - 1].first) + vp_data_lowerbound[i - 1].second;

    i = getIndex(t+startupTime);
    linefuDv2 = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);

	i = getIndex(t+startupTimeW);
    linefuDv3 = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);

    i=getIndex(t-1);
    linelastSpeed=vp_data[i].second; //无插值
    linelastDv = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
}

void SysControl::calCon(double time, double speedNow, double brakeOpen0, double accOpen0) {
    //std::cout<<"time="<<time<<std::endl;

    //UNUSED(accOpen);
    timeGap = time - lastControlTime;//求两次控制间的时间间隔
    //timeGap *= 1000.0;//ms
    //PRINTF(LOG_DEBUG, "timegap=%f\n",timeGap);   //应当是14ms
	lastControlTime = time;
    brakeOpen=brakeOpen0;
    accOpen=accOpen0;
    getSpeedDv(time,speedNow);

    getError();
    //double nextTime = getNextLineDuration();
    //PRINTF(LOG_DEBUG, "%s: duration=%f\n", __func__, nextTime);
    if(lineSpeed[3]==0){
        if(linefuDv2 > 0){
            conBrake = -100;
            conAcc = 0;
        }
        else{
            conBrake =100;
            conAcc=-100;
        }
        return;
    }
    else {
        if (lineDv[2] > 0) {//加速行驶
            conBrake=-100;
            if (linefuDv==0) {//将进入匀速段
                if (maxAccRefresh == 0) {
                    maxAccRefresh = 1;
                    maxAccRecorder = accOpen*(changepoint1to2+speed[3]/150.0);	//拐点参数1：accRecorder的比例系数
                }
//                if(linefuSpeed==15&&accOpen>0.25*maxAccRecorder) conAcc=-8;
//                else
                if(accOpen>maxAccRecorder) conAcc=-4;
                else conAcc=changeAcc();  //std::max(changeAcc(),0.0);
                sysControlAnother[0]=-1;
            }
            else if (linelastDv==0&&linelastSpeed>0){//刚驶离匀速段 //此处补偿易不足，曲线中仅出现一次
                if(error[2]>-2) conAcc=changeAcc()+0.01*lineDv[2]*sqrt(speed[3]);
                else conAcc=changeAcc();
            }
            /*
            else if(linefuDv<0){//将进入刹车段  //wltc用
                conAcc=-100;
                maxAccRefresh = 0;
                sysControlAnother[0]=4;
                conBrake=changeBrake();
            }*/

            /**/
            else if(linelastSpeed==0){//零起步漂移
                /*
                if(accOpen<2*lineDv[2]-1){
                    conAcc=2;
                    sysControlAnother[0]=9;
                }
                else conAcc=changeAcc();
                */
                conAcc=0.25*changeAcc();
                return;
            }
            else {
                conAcc = changeAcc();
            }
			return;
        }
        else if(lineDv[2]==0){//匀速行驶
            conBrake=-100;
            if(linefuDv<0){//将进入刹车段  //拐点参数2：此处受刹车性能影响较大
                conAcc=-100;
				maxAccRefresh = 0;
                sysControlAnother[0]=-2;
                if(brakeOpen<-changepoint2to3*linefuDv){
                    conBrake=changeBrake()+8;
                }else{
                    conBrake=0;
                }
            }
            else if (linefuDv>0){//将进入加速段   //此处待考
				maxAccRefresh = 0;
                if(error[2]>-1.8&&((accOpen*(changepoint1to2+speed[3]/150.0))<maxAccRecorder)) {//此处回复原油门开度
                    conAcc=2;
                }
                else conAcc=changeAcc();
            }
            else if(linelastDv>0){//刚驶离加速段  //此处高速补偿过度，需增加限位变量
                if((error[2]<1.5||dverror[2]<0.0)&&accOpen>maxAccRecorder){
                    conAcc = -4;
                    sysControlAnother[0] = -1;
                    //maxAccRefresh=0;
                }
                else{
                    conAcc=changeAcc();
                }
                //PRINTF(LOG_DEBUG, "%f+%f+%f+%f\n",error[2],accOpen,maxAccRecorder,conAcc);
            }
            else conAcc = changeAcc();
        }
        else if (lineDv[2] < 0) {//刹车阶段
//            if(linefuDv==0&&linefuSpeed>5){//将再次进入匀速，适度补偿
//                conBrake=-100;
//                if(accOpen<linefuSpeed*0.2) conAcc=changeAcc()+1;
//                else conAcc=0;
//            }
//            else {
                conBrake = changeBrake();
                conAcc=-100;
//            }
        }
    }

    return;
}

void SysControl::calConW(double time, double speedNow, double brakeOpen0, double accOpen0) {
    timeGap = time - lastControlTime;   //求两次控制间的时间间隔

    lastControlTime = time;
    brakeOpen=brakeOpen0;
    accOpen=accOpen0;

    getSpeedDv(time,speedNow);
    getError();

    if(lineSpeed[3]<=1){    //零速阶段
        if(linefuDv3 > 0.1&&time>5){    //此边界斜率决定是否将要进入加速段
            sysControlAnother[0] = 21;
            sysControlAnother[1] = 0;
            conBrake = -100;
            conAcc = 0;
        }
        else{
            sysControlAnother[0] = 11;
            sysControlAnother[1] = 0;
            conBrake =100;
            conAcc=-100;
        }
        return;
    }
    //---------------------------------------------------------------------//
    else {
        int time_decide=0;
        if(wltc_time_change==0) time_decide=(time>915&&time<920)||(time>1160&&time<1350)||(time>1550&&time<1727)||(time>1741&&time<1766);
        else if(wltc_time_change==1) time_decide=(time>612&&time<645)||(time>840&&time<900)||(time>930&&time<945)||(time>915&&time<920)||(time>1160&&time<1350)||(time>1550&&time<1727)||(time>1741&&time<1766);

        if (lineDv[2] >= 0||time_decide) {   //加速阶段
            conBrake=-100;

            if(speed[3]>=30&&error[2]<-2.2){  //1 检测上界误差，过大则切换刹车控制.2.3 2.5
                sysControlAnother[0] = 31;
                conAcc=-100;
                conBrake=changeBrakeW();
            }
            else if(speed[3]<30&&speed[3]>upperboundnow){ // (7) 实际速度小于30时超过上界要酌情刹车
                if (speed[3] <= linefuSpeedupperboundreal)
                {
                    sysControlAnother[0] = 12;
                    sysControlAnother[1] = 0;
                    conAcc=-100;
                }
                else
                {
                    sysControlAnother[0] = 22;
                    double halfdiff = 0.5*(upperboundnow - lowerboundnow);
                    double out = speed[3] - upperboundnow;
                    conAcc=-100;
                    conBrake= 0.5 * (0.5 + out / halfdiff) *changeBrakeW();
                }
            }
            else if(lineDv[2]>0&&(linefuDv<0)&&lineSpeed[3]<50){
                if(error[2]<0){
                    sysControlAnother[0] = 41;
                    conAcc=-100;
                    conBrake=changeBrakeW();
                }
                else {//此处存疑
                    if (time>=1407&&time<=1413) // (1) 这段时间内不允许补偿
                    {
                        sysControlAnother[0] = 32;
                        conBrake=-100;
                        conAcc=changeAccW();
                    }
                    else
                    {
                        sysControlAnother[0] = 42;
                        sysControlAnother[1] = 0;
                        double prop = 1; // (3) 速度不超过80 但是仍发现加速度达到有可能超过上界的要增大补偿
                        if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                        {
                            prop = dv[2] / lineDv[2];
                            sysControlAnother[0] = 52;
                        }
                        conBrake=-100;
                        conAcc=-wltc_degree_toLow_fix*4 * prop;

                        if (dv[2]<=lineDv[2])
                        {
                            if (error[2]>0)
                            {
                                if (prop == 1)
                                {

                                    conAcc=changeAccW();
                                    sysControlAnother[0] = 62;

                                    if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedonesecondreal)
                                    {
                                        if (conAcc>0) conAcc *= 0.5;
                                        else conAcc *= 1.3;
                                        sysControlAnother[0] = 72;
                                    }
                                }
                            }
                            else  // 在这里不会发生
                            {
                                if (error[2]>=-2)
                                {
                                    if (possibleSpeedafteronesecondbyaccelerationnow <= linefuSpeedonesecondreal)
                                    {
                                        conAcc=changeAccW();
                                        sysControlAnother[0] = 82;
                                    }
                                    else if (possibleSpeedafteronesecondbyaccelerationnow <= linefuSpeedonesecondreal+1.0)
                                    {
                                        conAcc=changeAccW();
                                        if (conAcc>0) conAcc *= 0.8;
                                        else conAcc *= 1.2;
                                        sysControlAnother[0] = 92;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else if ((linefuDv<0.6*lineDv[2])&&lineDv[2]>0){  //3 检测斜率骤降，记录maxAcc并减油门
                if (maxAccRefresh == 0||(time>maxAcctimer+2.0)) {   //Refresh标注是否需要更新maxAcc
                    maxAccRefresh = 1;
                    maxAcctimer=time;
                    maxAccRecorder = accOpen*(0.18+0.3+speed[3]/300.0);	//拐点参数1：accRecorder的比例系数
                }

                if((error[2]<0)||((error[2]<1||(error[2]>2&&error[2]<2.5))&&lineSpeed[3]<40)&&linefuDv>0&&accOpen>maxAccRecorder&&time_decide!=1&&!(time>=1407&&time<=1413)) {
                    if (speed[3]>90) // (2) 速度大于90 看情况给加到刹补偿
                    {
                        if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                        {
                            sysControlAnother[0] = 102;
                            sysControlAnother[1] = 0;
                            conAcc=-wltc_degree_toLow_fix*4 * 0.5;
                        }
                        else
                        {
                            if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedonesecondreal)
                            {
                                sysControlAnother[0] = 112;
                                sysControlAnother[1] = 0;
                                conAcc=-wltc_degree_toLow_fix*4 * 0.25;
                            }
                            else
                            {
                                sysControlAnother[0] = 122;
                                conAcc=changeAccW();
                            }
                        }
                    }
                    else
                    {
                        sysControlAnother[0] = 132;
                        sysControlAnother[1] = 0;
                        double prop = 1; // (3) 速度不超过90 但是仍发现加速度达到有可能超过上界的要增大补偿
                        if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                        {
                            prop = dv[2] / lineDv[2];
                            sysControlAnother[0] = 142;
                        }

                        conAcc=-wltc_degree_toLow_fix*4 * prop;

                        if (dv[2]<=lineDv[2])
                        {
                            if (error[2]>0)
                            {
                                if (prop == 1)
                                {
                                    sysControlAnother[0] = 152;
                                    conAcc=changeAccW();

                                    if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedonesecondreal)
                                    {
                                        if (conAcc>0) conAcc *= 0.5;
                                        else conAcc *= 1.3;
                                        sysControlAnother[0] = 162;
                                    }
                                }
                            }
                            else
                            {
                                if (error[2]>=-2)
                                {
                                    if (possibleSpeedafteronesecondbyaccelerationnow <= linefuSpeedonesecondreal)
                                    {
                                        sysControlAnother[0] = 172;
                                        conAcc=changeAccW();
                                    }
                                    else if (possibleSpeedafteronesecondbyaccelerationnow <= linefuSpeedonesecondreal+1.0)
                                    {
                                        conAcc=changeAccW();
                                        if (conAcc>0) conAcc *= 0.8;
                                        else conAcc *= 1.2;
                                        sysControlAnother[0] = 182;
                                    }
                                }
                            }
                        }
                    }
                }
                else if((error[2]<0||error[2]>2)&&linefuDv<0&&accOpen>0.5*maxAccRecorder&&!(time>=1407&&time<=1413)) {
                    if (speed[3]>90) // (2) 速度大于90 看情况给加到刹补偿
                    {
                        if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                        {
                            sysControlAnother[0] = 192;
                            sysControlAnother[1] = 0;
                            conAcc=-wltc_degree_toLow_fix*6 * 0.5;
                        }
                        else
                        {
                            if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedonesecondreal)
                            {
                                sysControlAnother[0] = 202;
                                sysControlAnother[1] = 0;
                                conAcc=-wltc_degree_toLow_fix*6 * 0.25;
                            }
                            else
                            {
                                sysControlAnother[0] = 212;
                                conAcc=changeAccW();
                            }
                        }
                    }
                    else
                    {
                        sysControlAnother[0] = 222;
                        sysControlAnother[1] = 0;

                        double prop = 1; // (3) 速度不超过90 但是仍发现加速度达到有可能超过上界的要增大补偿
                        if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                        {
                            prop = dv[2] / lineDv[2];
                            sysControlAnother[0] = 232;
                        }

                        conAcc=-wltc_degree_toLow_fix*6 * prop;  //std::max(changeAcc(),0.0);

                        if (dv[2]<=lineDv[2])
                        {
                            if (error[2]>0)
                            {
                                if (prop == 1)
                                {
                                    sysControlAnother[0] = 242;
                                    conAcc=changeAccW();

                                    if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedonesecondreal)
                                    {
                                        if (conAcc>0) conAcc *= 0.5;
                                        else conAcc *= 1.3;
                                        sysControlAnother[0] = 252;
                                    }
                                }
                            }
                            else
                            {
                                if (error[2]>=-2)
                                {
                                    if (possibleSpeedafteronesecondbyaccelerationnow <= linefuSpeedonesecondreal)
                                    {
                                        sysControlAnother[0] = 262;
                                        conAcc=changeAccW();
                                    }
                                    else if (possibleSpeedafteronesecondbyaccelerationnow <= linefuSpeedonesecondreal+1.0)
                                    {
                                        sysControlAnother[0] = 272;
                                        conAcc=0.8*changeAccW();
                                    }
                                }
                            }
                        }
                    }
                }
                else {
                    if (time>=1407&&time<=1413) // (1) 这段时间内不允许补偿
                    {
                        sysControlAnother[0] = 32;;
                        conAcc=changeAccW();
                    }
                    else
                    {
                        conAcc=changeAccW();
                        sysControlAnother[0] = 282;

                        if (conAcc > 0) // (4) 斜率骤降段 如果仍加速则小于目标过多时不易猛踩油门 酌情减半
                        {
                            if (error[2] > 2 && possibleSpeedafteronesecondbyaccelerationnow >= linefuSpeedlowerboundreal)
                            {
                                conAcc *= 0.5;
                                sysControlAnother[0] = 292;
                            }
                        }
                        else // (4) 斜率骤降段 如果正在减速并且可能超过目标的应多缩回油门 酌情加倍
                        {
                            if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedonesecondreal + 1.0)
                            {
                                conAcc *= 1.3;
                                sysControlAnother[0] = 302;
                            }
                        }
                    }
                }
            }
            else if ((linelastDv<0)&&linelastSpeed>0&&lineDv[2]>=0){ //4 检测减速到加速拐点，附加特定加油量
                if (lineDv[2]>0) // (5) 减加速拐点 特殊处理
                {
                    if (linefuDv>=lineDv[2])
                    {
                        if(error[2]>-1.2){//1.5 1.2     高速段 0.4 0.2
                            if (possibleSpeedafteronesecondbyaccelerationnow<linefuSpeedupperboundreal)
                            {
                                if(lineSpeed[3]>=50) {
                                    sysControlAnother[0] = 312;
                                    conAcc=changeAccW()+0.2*(lineDv[2]-linelastDv)*sqrt(speed[3]);
                                }
                                else if(lineSpeed[3]>30) {
                                    sysControlAnother[0] = 322;
                                    conAcc=changeAccW()+0.1*(lineDv[2]-linelastDv)*sqrt(speed[3]);
                                }
                                else {
                                    sysControlAnother[0] = 332;
                                    conAcc=changeAccW();
                                }

                                if (possibleSpeedafteronesecondbyaccelerationnow < linefuSpeedlowerboundreal || speed[3] < lowerboundnow)
                                {
                                    sysControlAnother[0] = 342;
                                    if (conAcc>0) conAcc *= 1.2;
                                    else conAcc *= 0.0;
                                }
                            }
                            else
                            {
                                sysControlAnother[0] = 352;
                                conAcc=changeAccW();
                                if (conAcc>0) conAcc *= 0.6;
                                else conAcc *= 1.2;
                            }
                        }
                        else {
                            if (possibleSpeedafteronesecondbyaccelerationnow<linefuSpeedupperboundreal)
                            {
                                sysControlAnother[0] = 362;
                                conAcc=changeAccW();
                            }
                            else
                            {
                                sysControlAnother[0] = 372;
                                conAcc=changeAccW();
                                if (conAcc>0) conAcc *= 0.5;
                                else conAcc *= 1.3;
                            }
                        }
                    }
                    else
                    {
                        if (possibleSpeedafteronesecondbyaccelerationnow<linefuSpeedupperboundreal)
                        {
                            sysControlAnother[0] = 382;
                            conAcc=changeAccW();
                            if (conAcc>0) conAcc *= 0.6;
                            else conAcc *= 1.2;
                        }
                        else
                        {
                            sysControlAnother[0] = 392;
                            conAcc=changeAccW();
                            if (conAcc>0) conAcc *= 0.4;
                            else conAcc *= 1.4;
                        }
                    }
                }
                else
                {
                    if (linefuDv>lineDv[2])
                    {
                        if(error[2]>-1.2){//1.5 1.2     高速段 0.4 0.2
                            if (possibleSpeedafteronesecondbyaccelerationnow<linefuSpeedupperboundreal)
                            {
                                if(lineSpeed[3]>=50) {
                                    sysControlAnother[0] = 402;
                                    conAcc=changeAccW()+0.2*(lineDv[2]-linelastDv)*sqrt(speed[3]);
                                }
                                else if(lineSpeed[3]>30) {
                                    sysControlAnother[0] = 412;
                                    conAcc=changeAccW()+0.1*(lineDv[2]-linelastDv)*sqrt(speed[3]);
                                }
                                else {
                                    sysControlAnother[0] = 422;
                                    conAcc=changeAccW();
                                }

                                if (possibleSpeedafteronesecondbyaccelerationnow < linefuSpeedlowerboundreal || speed[3] < lowerboundnow)
                                {
                                    if (conAcc>0) conAcc *= 1.2;
                                    else conAcc *= 0.0;
                                    sysControlAnother[0] = 432;
                                }
                            }
                            else
                            {
                                sysControlAnother[0] = 442;
                                conAcc=changeAccW();
                                if (conAcc>0) conAcc *= 0.5;
                                else conAcc *= 1.3;
                            }
                        }
                        else {
                            if (possibleSpeedafteronesecondbyaccelerationnow<linefuSpeedupperboundreal)
                            {
                                sysControlAnother[0] = 452;
                                conAcc=0.9*changeAccW();
                            }
                            else
                            {
                                sysControlAnother[0] = 462;
                                conAcc=changeAccW();
                                if (conAcc>0) conAcc *= 0.4;
                                else conAcc *= 1.4;
                            }
                        }
                    }
                    else if (linefuDv==lineDv[2])
                    {
                        if (error[2]>0)
                        {
                            sysControlAnother[0] = 472;
                            conAcc=changeAccW();
                        }
                        else
                        {
                            if (possibleSpeedafteronesecondbyaccelerationnow<linefuSpeedupperboundreal)
                            {
                                sysControlAnother[0] = 482;
                                conAcc=changeAccW();
                            }
                            else
                            {
                                sysControlAnother[0] = 492;
                                conAcc=changeAccW();
                                if (conAcc>0) conAcc *= 0.5;
                                else conAcc *= 1.3;
                            }
                        }
                    }
                    else
                    {
                        if (error[2]>1.2)
                        {
                            sysControlAnother[0] = 502;
                            conAcc=0.8*changeAccW();

                            if (possibleSpeedafteronesecondbyaccelerationnow < linefuSpeedlowerboundreal)
                            {
                                conAcc=changeAccW();
                                sysControlAnother[0] = 512;
                            }
                        }
                        else
                        {
                            if (possibleSpeedafteronesecondbyaccelerationnow < linefuSpeedlowerboundreal)
                            {
                                conAcc=changeAccW();
                                sysControlAnother[0] = 522;
                            }
                            else if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                            {
                                if (speed[3] > 80)
                                {
                                    sysControlAnother[0] = 532;
                                    conAcc=changeAccW();
                                    if (conAcc>0) conAcc *= 0.5;
                                    else conAcc *= 1.3;
                                }
                                else
                                {
                                    sysControlAnother[0] = 542;
                                    sysControlAnother[1] = 0;
                                    conAcc=-100;
                                }
                            }
                            else
                            {
                                sysControlAnother[0] = 552;
                                conAcc=0.8*changeAccW();
                            }
                        }
                    }
                }
            }
            else {
                sysControlAnother[0] = 101;
                conAcc = changeAccW();
            }
            return;
        }
        else if (lineDv[2] < 0) {   //刹车阶段
            maxAccRefresh=0;
            if(linefuDv>=0&&linefuSpeed>5){    //1 检测减加速拐点，
                if (linefuDv>0) // (8) 减加速拐点特殊处理
                {
                    if(error[2]>-2){
                        conBrake=-100;

                        if(accOpen<linefuSpeed&&lineSpeed[3]>=59&&!(time>=1407&&time<=1413)){
                            if(time>1530&&time<1535) {
                                sysControlAnother[0] = 562;
                                sysControlAnother[1] = 0;
                                conAcc=wltc_degree_fix*6.5;
                            }
                            else {
                                sysControlAnother[0] = 572;
                                sysControlAnother[1] = 0;
                                conAcc=wltc_degree_fix*6;
                            }     //是否要添加斜率判断？
                        }
                        else if(accOpen<linefuSpeed*0.5&&linefuSpeed>25&&!(time>=1407&&time<=1413)) {
                            sysControlAnother[0] = 582;
                            sysControlAnother[1] = 0;
                            conAcc=wltc_degree_fix*3;
                        }
                        else {
                            if (time>=1407&&time<=1413) // (1) 这段时间内不允许补偿
                            {
                                sysControlAnother[0] = 32;
                                conAcc=changeAccW();
                            }
                            else
                            {
                                sysControlAnother[0] = 592;
                                conAcc=changeAccW();
                            }
                        }

                        if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                        {
                            sysControlAnother[0] = 602;
                            if (conAcc>0) conAcc *= 0.5;
                            else conAcc *= 1.3;
                        }
                        else if (possibleSpeedafteronesecondbyaccelerationnow < linefuSpeedlowerboundreal)
                        {
                            sysControlAnother[0] = 612;
                            if (conAcc>0) conAcc *= 1.3;
                            else conAcc *= 0.5;
                        }
                    }
                    else{
                        conBrake = -100;

                        if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                        {
                            if (speed[3] > 80)
                            {
                                sysControlAnother[0] = 622;
                                conAcc = changeAccW();
                                if (conAcc>0) conAcc *= 0.0;
                                else conAcc *= 1.5;
                            }
                            else
                            {
                                sysControlAnother[0] = 632;
                                sysControlAnother[1] = 0;
                                conAcc=-100;
                            }
                        }
                        else
                        {
                            sysControlAnother[0] = 642;
                            sysControlAnother[1] = 0;
                            conAcc=0;
                            conBrake=0;
                        }
                    }
                }
                else
                {
                    if(error[2]>0){
                        conBrake=-100;

                        if(accOpen<linefuSpeed&&lineSpeed[3]>=59&&!(time>=1407&&time<=1413)){
                            if(time>1530&&time<1535) {
                                sysControlAnother[0] = 652;
                                sysControlAnother[1] = 0;
                                conAcc=wltc_degree_fix*6.5;
                            }
                            else {
                                sysControlAnother[0] = 662;
                                sysControlAnother[1] = 0;
                                conAcc=wltc_degree_fix*6;
                            }     //是否要添加斜率判断？
                        }
                        else if(accOpen<linefuSpeed*0.5&&linefuSpeed>25&&!(time>=1407&&time<=1413)) {
                            sysControlAnother[0] = 672;
                            sysControlAnother[1] = 0;
                            conAcc=wltc_degree_fix*3;
                        }
                        else {
                            if (time>=1407&&time<=1413) // (1) 这段时间内不允许补偿
                            {
                                sysControlAnother[0] = 32;
                                conAcc=changeAccW();
                            }
                            else
                            {
                                sysControlAnother[0] = 682;
                                conAcc=changeAccW();
                            }
                        }

                        if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                        {
                            if (conAcc>0) conAcc *= 0.5;
                            else conAcc *= 1.3;
                            sysControlAnother[0] = 692;
                        }
                        else if (possibleSpeedafteronesecondbyaccelerationnow < linefuSpeedlowerboundreal)
                        {
                            if (conAcc>0) conAcc *= 1.3;
                            else conAcc *= 0.5;
                            sysControlAnother[0] = 702;
                        }
                    }
                    else{
                        conBrake = -100;
                        if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                        {
                            sysControlAnother[0] = 712;
                            conAcc=-100;
                            conBrake=0.5*changeBrakeW();
                        }
                        else
                        {
                            if (possibleSpeedafteronesecondbyaccelerationnow < linefuSpeedlowerboundreal)
                            {
                                sysControlAnother[0] = 722;
                                conAcc=changeAccW();
                            }
                            else
                            {
                                sysControlAnother[0] = 732;
                                sysControlAnother[1] = 0;
                                conAcc=0;
                                conBrake=0;
                            }
                        }
                    }
                }
            }
            else if((linelastDv<2*lineDv[2]||lineDv[2]>-1||linefuDv>-1||error[2]>2)&&lineSpeed[3]>5){  //2 检测刹车缓速段
                //缓速段判定——斜率变化，斜率绝对值（低于15边界大幅降低）
                // (8) 缓速段刹车细化
                double tempcon=pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
                Q_UNUSED(tempcon);
                //if(error[2]>=-1.0&&error[2]<=1.0){  //a 主体采用加速度控制
                //    conBrake=-100;
                //    conAcc=tempcon;
                //}
                if(error[2]<(wltc_lowborder_fix-0.2)){  //b 超上界切换刹车
                    conAcc=-100;

                    if (speed[3] > upperboundnow)
                    {
                        sysControlAnother[0] = 742;
                        conBrake=0.9*changeBrakeW();
                    }
                    else
                    {
                        sysControlAnother[0] = 752;
                        conBrake=0.5*changeBrakeW();

                        if (possibleSpeedafteronesecondbyaccelerationnow < linefuSpeedlowerboundreal)
                        {
                            if (conBrake>0) conBrake *= 0.5;
                            else conBrake *= 1.3;
                            sysControlAnother[0] = 762;
                        }
                    }
                }
                else if(error[2]>=(wltc_lowborder_fix-0.2)&&error[2]<wltc_lowborder_fix){
                    if (possibleSpeedafteronesecondbyaccelerationnow < linefuSpeedlowerboundreal)
                    {
                        sysControlAnother[0] = 772;
                        sysControlAnother[1] = 0;
                        conAcc=0;
                        conBrake=-100;
                    }
                    else if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                    {
                        sysControlAnother[0] = 782;
                        sysControlAnother[1] = 0;
                        conAcc=-100;
                        conBrake=0;
                    }
                    else
                    {
                        sysControlAnother[0] = 792;
                        sysControlAnother[1] = 0;
                        conAcc=0;
                        conBrake=0;
                    }
                }
                else{
                    conBrake=-100;

                    if (speed[3] < lowerboundnow)
                    {
                        sysControlAnother[0] = 802;
                        conAcc=0.8*changeAccW();
                    }
                    else
                    {
                        sysControlAnother[0] = 812;
                        conAcc=0.5*changeAccW();

                        if (possibleSpeedafteronesecondbyaccelerationnow > linefuSpeedupperboundreal)
                        {
                            if (conAcc>0) conAcc *= 0.5;
                            else conAcc *= 1.3;
                            sysControlAnother[0] = 822;
                        }
                        else if (possibleSpeedafteronesecondbyaccelerationnow < linefuSpeedlowerboundreal)
                        {
                            conAcc=0.7*changeAccW();
                            if (conAcc>0) conAcc *= 1.3;
                            else conAcc *= 0.5;
                            sysControlAnother[0] = 832;
                        }
                    }
                }
            }
            else {
                sysControlAnother[0] = 51;
                conAcc=-100;
                conBrake = changeBrakeW();
            }
        }
    }

    return;
}

double SysControl::changeAcc() {
    pidA[0][0]= accpidfix*2;
    pidA[0][1]= accpidfix*2;
    pidA[0][2]= accpidfix*0.1;

    //正常运行con1
    double con1=borderfix*5 * (error[2] - error[1]) + borderfix*0.2 * error[2] + 0.1 * (error[2] - 2 * error[1] + error[0]);

    if(lineDv[2]==0){
        if(error[2]<-1.0||error[2]>1.0){//改动2：原1.0
            return con1;
        }
        else{
            sysControlAnother[1]=3;
            return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
        }
    }
    else{
        if(error[2]<-1.0||error[2]>1.0){//改动3：原0.5 到 1.0 到 1.5(wltc)
            sysControlAnother[1]=1;
            return con1;

        }
        else{
            sysControlAnother[1]=2;
            return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);

            //return pid[0][0] * (error[2] - error[1]) + pid[0][1] * error[2] + pid[0][2] * (error[2] - 2 * error[1] + error[0]);
        }
    }

    PRINTF(LOG_DEBUG, "%s: this should never be reached!\n",__func__);
    return 0.0;
}

double SysControl::changeAccW() {
    pidA[0][0]=accpidfixW*2;
    pidA[0][1]=accpidfixW*2;
    pidA[0][2]=accpidfixW*0.1;

    //正常运行con1
    double con1=borderfixW*5 * (error[2] - error[1]) + borderfixW*0.2 * error[2] + 0.1 * (error[2] - 2 * error[1] + error[0]);

    if(lineDv[2]==0){
        if(error[2]<-1.0||error[2]>1.0){//改动2：原1.0
            sysControlAnother[1] = 3;
            return con1;
        }
        else{
            sysControlAnother[1] = 4;
            return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
        }
    }
    else{
        if(error[2]<-1.5||error[2]>1.5){//改动3：原0.5 到 1.0 到 1.5(wltc)
            sysControlAnother[1] = 1;
            if(error[2]<0) return con1;
            else if(error[2]>0) return con1;
        }
        else{
            sysControlAnother[1] = 2;
            return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
        }
    }

    PRINTF(LOG_DEBUG, "%s: this should never be reached!\n",__func__);
    return 0.0;
}

double SysControl::changeBrake() {
    if (linelastDv==0||lineDv[2]<1.5*linelastDv){
        return 30*pid[1][0] * (error[2] - error[1]) + 4*pid[1][1] * error[2] + pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]<-1.5||(error[2]<-0.5*border&&dverror[2]<=0)){
        return 40*pid[1][0] * (error[2] - error[1]) +1* pid[1][1] * error[2] + 10*pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]<-0.5*border&&dverror[2]>0){
        return 20*pid[1][0] * (error[2] - error[1]) + 0.001*pid[1][1] * error[2] + 5*pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]>1.5||(error[2]>0.5*border&&dverror[2]>=0)){
        return 40*pid[1][0] * (error[2] - error[1]) + 2*pid[1][1] * error[2] +10* pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]>0.5*border&&dverror[2]<0){
        return 20*pid[1][0] * (error[2] - error[1]) + 0.001*pid[1][1] * error[2] +5* pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else{
        sysControlAnother[1]=8;
        return 10*pid[1][0] * (error[2] - error[1]) + 0.00001*pid[1][1] * error[2] + pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }

    PRINTF(LOG_DEBUG, "%s: this should never be reached!\n",__func__);
    return 0.0;
}

double SysControl::changeBrakeW() {
    if (linelastDv>=0||lineDv[2]<1.5*linelastDv){
        sysControlAnother[1] = 6;
        return 60*pid[1][0] * (error[2] - error[1]) + 6*pid[1][1] * error[2] + pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]<-1.5||(error[2]<-0.5&&dverror[2]<=0)){
        sysControlAnother[1] = 7;
        return 40*pid[1][0] * (error[2] - error[1]) +4* pid[1][1] * error[2] + 10*pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]<-0.5&&dverror[2]>0){
        sysControlAnother[1] = 8;
        return 20*pid[1][0] * (error[2] - error[1]) + 0.001*pid[1][1] * error[2] + 5*pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]>1.5||(error[2]>0.5&&dverror[2]>=0)){
        sysControlAnother[1] = 9;
        return 40*pid[1][0] * (error[2] - error[1]) + 2*pid[1][1] * error[2] +10* pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]>0.5&&dverror[2]<0){
        sysControlAnother[1] = 10;
        return 20*pid[1][0] * (error[2] - error[1]) + 0.001*pid[1][1] * error[2] +5* pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else{
        sysControlAnother[1] = 11;
        return pidA[1][0] * (dverror[2] - dverror[1]) + pidA[1][1] * dverror[2] + pidA[1][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
    }

    PRINTF(LOG_DEBUG, "%s: this should never be reached!\n",__func__);
    return 0.0;
}

double SysControl::getconAcc() {
    sysControlAnother[2] = 4;
    double factor = 1.0;

    if(lineSpeed[3]==15&&lineDv[2]==0&&conAcc>0) {
        sysControlAnother[2] = 1;
        factor *= 0.5;
    }
    if(lineSpeed[3]<30&&accOpen>50&&conAcc>0) {
        sysControlAnother[2] = 2;
        factor *= 0.0;
    }
    if(lineSpeed[3]<20&&lineSpeed[3]!=15&&lineDv[2]<1&&lineDv[2]>=0) {
        sysControlAnother[2] = 3;
        factor *= 0.5;
    }
    return accfix * factor * conAcc;
}

double SysControl::getconBrake() {
    sysControlAnother[3] = 5;
    return brakefix * conBrake;
}

int SysControl::GetSysControlMethod() {
    return sysControlAnother[0] + 1000*sysControlAnother[1] + 100000*sysControlAnother[2] + 1000000*sysControlAnother[3];
}
