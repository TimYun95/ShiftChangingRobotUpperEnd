#include "syscontrol.h"

#include <math.h>
#include <stdio.h>

#include <iostream>
#include <algorithm>
#include <QtMath>

#include <QMessageBox>
#include <QtMath>

#include "printf.h"

using namespace std;

const int p=1;
int speedmethod=0;
double controlparams1[7];
double initcontrolparams1[7]={0.18,35,60,2,1,1.2,0.5}; // PULSE信号
double controlparams2[7];
double initcontrolparams2[7]={0.1,30,20,1,1,1.2,0.5}; // CAN信号

const double speedErrorLimit=2.0;
double changepoint1to2=0.1;//1
const double changepoint1to2fix=2;
double changepoint2to3=30;//2
const double changepoint2to3fix=5;
double accfix=0.5;//pass
double borderfix=2;//3 速度增益
double accpidfix=1;//4 加速度增益
double startupTime=1;//5
//border here//6
double brakefix=0.5;//7

double borderfixW=6;
double accpidfixW=1;
double startupTimeW=0;
//根据油门特性调整，默认1，较灵敏0.5
double wltc_degree_fix=1;
double wltc_degree_toLow_fix=1.5; //pass
double wltc_lowborder_fix=0.8; //pass
double wltc_time_change=1;//取1大区间

SysControl::SysControl()
{
    sysControlMethod=0;
    lastControlTime=0.0;
	maxAccRefresh = 0;

    //油门0
    pid[0][0]= 0.8;
    pid[0][1]= 0.1;
    pid[0][2]= 0.1;

    //刹车1
    pid[1][0]= -1;
    pid[1][1]= -0.5;
    pid[1][2]= -0.1;

    //Apid
    pidA[0][0]= accpidfix*2;
    pidA[0][1]= accpidfix*2;
    pidA[0][2]= accpidfix*0.1;

    //刹车1
    pidA[1][0]= -1;//改动1：原100
    pidA[1][1]= -0.1;
    pidA[1][2]= -0.1;

}

SysControl::~SysControl()
{

}

//修改5/////////////////////////////////////////////////////////////////////
//控制速度[0]  ACD模式2 predefined模式 速度控制 更新vpdata数据
void SysControl::vpdataCarSpeed2() {
    double deltaSpeed=sysReadCarSpeed-sysSpeedStart;
    double deltaTen=(deltaSpeed)/(fabs(deltaSpeed))*10.0;

    double acc=1.5;//加速度的定义变量，即10千米用1.5秒到达，折算加速度是6.66>6(wltc最大加速度)
    switch(sysCarSelect)
    {
    case 0:
        acc=2;
        break;
    case 1:
        acc=1.5;
        break;
    case 2:
        acc=1;
        break;
    default:
        acc=2;
    }
    //std::cout<<" -system-acc-  "<<acc<<std::endl;

    double timeStart=0;
    if(fabs(deltaSpeed)<10)
    {
        vp_data.clear();
        vp_data.push_back( std::make_pair(0,0) );
        vp_data.push_back( std::make_pair(timeStart+1,sysSpeedStart) );
        vp_data.push_back( std::make_pair(timeStart+1+acc,sysReadCarSpeed) );
        vp_data.push_back( std::make_pair(99999,sysReadCarSpeed) );
    }

    if(fabs(deltaSpeed)>=10)
    {
        double count=floor(fabs(deltaSpeed)/10.0)+1;
        vp_data.clear();

        vp_data.push_back( std::make_pair(0,0) );
        vp_data.push_back( std::make_pair(timeStart+1,sysSpeedStart) );

        for(int i=1;i<count;i++)
        {
            vp_data.push_back( std::make_pair(timeStart+1+acc*i,sysSpeedStart+deltaTen*i) );
        }

        vp_data.push_back( std::make_pair(timeStart+1+acc*count,sysReadCarSpeed) );
        vp_data.push_back( std::make_pair(99999,sysReadCarSpeed) );
    }
}

//修改12/////////////////////////////////////////////////////////////////////
//控制速度[0]  ACD模式1 online模式 速度控制 更新vpdata数据
void SysControl::vpdataCarSpeed1() {
    double deltaSpeed=sysReadCarSpeed-sysSpeedStart;
    double deltaTen=(deltaSpeed)/(fabs(deltaSpeed))*10.0;

    double acc=1.5;//加速度的定义变量，即10千米用1.5秒到达，折算加速度是6.66>6(wltc最大加速度)
    switch(sysCarSelect)
    {
    case 0:
        acc=2;
        break;
    case 1:
        acc=1.5;
        break;
    case 2:
        acc=1;
        break;
    default:
        acc=2;
    }
    //std::cout<<" -system-acc-  "<<acc<<std::endl;

    //修改13/////////////////////////////////////////////////////////////////////
    double timeStart=0;
    timeStart=sysTimeStart;

    //因为本版本online对于elapsedSeconds强制做归零修改，所以下面这句 不用注释掉
    timeStart=0;//因为每次刷新时elapsedSeconds都归0.且在pedalrobotui里，timeStart的赋值在elapsedSeconds归0之前

    if(timeStart==0)
    {
        if(fabs(deltaSpeed)<10)
        {
            vp_data.clear();
            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart+1,sysSpeedStart) );
            vp_data.push_back( std::make_pair(timeStart+1+acc,sysReadCarSpeed) );
            vp_data.push_back( std::make_pair(99999,sysReadCarSpeed) );
        }

        if(fabs(deltaSpeed)>=10)
        {
            double count=floor(fabs(deltaSpeed)/10.0)+1;
            vp_data.clear();

            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart+1,sysSpeedStart) );

            for(int i=1;i<count;i++)
            {
                vp_data.push_back( std::make_pair(timeStart+1+acc*i,sysSpeedStart+deltaTen*i) );
            }

            vp_data.push_back( std::make_pair(timeStart+1+acc*count,sysReadCarSpeed) );
            vp_data.push_back( std::make_pair(99999,sysReadCarSpeed) );
        }
    }

    if(timeStart!=0)
    {
        if(fabs(deltaSpeed)<10)
        {
            vp_data.clear();
            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart,sysSpeedStart) );
            vp_data.push_back( std::make_pair(timeStart+1,sysSpeedStart) );
            vp_data.push_back( std::make_pair(timeStart+1+acc,sysReadCarSpeed) );
            vp_data.push_back( std::make_pair(99999,sysReadCarSpeed) );
        }

        if(fabs(deltaSpeed)>=10)
        {
            double count=floor(fabs(deltaSpeed)/10.0)+1;
            vp_data.clear();

            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart,sysSpeedStart) );
            vp_data.push_back( std::make_pair(timeStart+1,sysSpeedStart) );

            for(int i=1;i<count;i++)
            {
                vp_data.push_back( std::make_pair(timeStart+1+acc*i,sysSpeedStart+deltaTen*i) );
            }

            vp_data.push_back( std::make_pair(timeStart+1+acc*count,sysReadCarSpeed) );
            vp_data.push_back( std::make_pair(99999,sysReadCarSpeed) );
        }
    }
    //修改13/////////////////////////////////////////////////////////////////////

}

//修改12/////////////////////////////////////////////////////////////////////
/*
        std::cout<<vp_data.size()<<"   "<<elapsedSeconds<<std::endl;
        std::cout<<vp_data[0].first<<"   "<<vp_data[0].second<<std::endl;
        std::cout<<vp_data[1].first<<"   "<<vp_data[1].second<<std::endl;
        std::cout<<vp_data[2].first<<"   "<<vp_data[2].second<<std::endl;
        std::cout<<vp_data[3].first<<"   "<<vp_data[3].second<<std::endl;
        std::cout<<vp_data[4].first<<"   "<<vp_data[4].second<<std::endl;
        std::cout<<vp_data[5].first<<"   "<<vp_data[5].second<<std::endl;
        std::cout<<vp_data[6].first<<"   "<<vp_data[6].second<<std::endl;
        std::cout<<vp_data[7].first<<"   "<<vp_data[7].second<<std::endl;
        std::cout<<vp_data[8].first<<"   "<<vp_data[8].second<<std::endl;
        std::cout<<vp_data[9].first<<"   "<<vp_data[9].second<<std::endl;
        std::cout<<vp_data[10].first<<"   "<<vp_data[10].second<<std::endl;
*/
//修改5/////////////////////////////////////////////////////////////////////

void SysControl::Init(const PairData& data, const Configuration* conf) {
    maxAccRefresh = 0;
    maxAcctimer=0;
    timeGap = 24;
    vp_data = data;
    isTrain = 0;
    border = 1.2;
	
    //修改5/////////////////////////////////////////////////////////////////////

    //修改21////////////////////////////////////////////////////////////////////
    if(systemModelSelect==2 || systemModelSelect==1){//ACD的predef模式和online模式
        //修改21////////////////////////////////////////////////////////////////////

        vp_data.clear();
        //double vpSize=vp_data.size();//这样显示不对，始终0
        //std::cout<<vpSize<<std::endl;

        //vpdataSize=vp_data.size();//这样显示不对，始终0
        std::cout<<vp_data.size()<<std::endl;

        vp_data.push_back( std::make_pair(0,0) );
        vp_data.push_back( std::make_pair(1,0) );
        vp_data.push_back( std::make_pair(99990,0) );
        vp_data.push_back( std::make_pair(99999,0) );
        std::cout<<vp_data.size()<<" -system-   "<<vpdataSize<<"   "<<"   "<<vp_data[1].first<<"   "<<vp_data[1].second<<"   "<<vp_data[vpdataSize-1].second<<std::endl;

        /*
            vp_data.push_back( std::make_pair(1,2) );
            vp_data.push_back( std::make_pair(999999,9) );


            vpdataSize=vp_data.size();
            std::cout<<vpdataSize<<std::endl;
            std::cout<<"vp_data.size: "<<vp_data.size()<<"   "<<"   "<<vp_data[0].first<<"   "<<vp_data[vpdataSize-1].second<<std::endl;

            vector<pair<int,int> > ::iterator iter;
            //iter=vp_data.begin();
            vp_data.erase(vp_data.end());
            //double a=(*iter).first;

            vpdataSize=vp_data.size();
            std::cout<<vpdataSize<<std::endl;


            std::cout<<"vp_data.size: "<<vp_data.size()<<"   "<<"   "<<vp_data[0].first<<"   "<<vp_data[vpdataSize-1].second<<std::endl;
            //std::cout<<vp_data[5].second<<"   "<<vpSize-1<<std::endl;
            */
    }
    //修改5/////////////////////////////////////////////////////////////////////

    speedmethod = conf->getSpeedMethod;

    if(speedmethod==2){ // CAN
        for(int i=0; i<7; ++i){
            controlparams1[i] = conf->sysControlParams[i];
        }
        accfix=0.5;
    }
    else if(speedmethod==1){ // PULSE
        for(int i=0; i<7; ++i){
            controlparams2[i] = conf->sysControlParams[i];
        }
        accfix=0.5;
    }
    changepoint1to2=conf->sysControlParams[0]; // 加速到匀速拐点补偿
    changepoint2to3=conf->sysControlParams[1]; // 匀速到刹车拐点补偿
    borderfix=conf->sysControlParams[2]/10.0; // 寻优起点设定
    accpidfix=conf->sysControlParams[3]; // 加速度控制增益
    startupTime=conf->sysControlParams[4]; // 起步提前量
    border=conf->sysControlParams[5]; // 切换border度
    brakefix=conf->sysControlParams[6]; // 总刹车增益

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

size_t SysControl::getIndex(double t) {
    size_t i;
    for(i = 0; i<vp_data.size(); ++i) {
        if (t >= vp_data[i].first - vp_data[0].first) {
            continue;
        }
        else {
            break;
        }
    }
    return i;//当前在i-1到i段
}

void SysControl::getSpeedDv(double t,double speedNow) {
    size_t i = getIndex(t);
    indexNow = i;

    speed[0] = speed[1];
    speed[1] = speed[2];
    speed[2] = speed[3];
    int speedNowIntver=(int)(speedNow*10);//取小数点后2A位
    speedNow=((float)speedNowIntver)/10.0;
    speed[3] = speedNow;

    dv[0] = dv[1];
    dv[1] = dv[2];
    dv[2] = (speed[3] - speed[2]) / timeGap;

    i = getIndex(t + 0.0*speed[3]/60);
    if (i==0) i=1;
    lineDv[0] = lineDv[1];
    lineDv[1] = lineDv[2];
    lineDv[2] = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
    lineSpeed[0] = lineSpeed[1];
    lineSpeed[1] = lineSpeed[2];
    lineSpeed[2] = lineSpeed[3];
    lineSpeed[3] = (t - vp_data[i - 1].first)*lineDv[2] + vp_data[i - 1].second;

    i = getIndex(t+1);
    if (i==0) i=1;
    if(i<vp_data.size()){
        linefuDv = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
        linefuSpeed=vp_data[i].second;
    }else{
        linefuDv=0;
        linefuSpeed=0;
    }
    i = getIndex(t+startupTime);
    if (i==0) i=1;
    if(i<vp_data.size()){
        linefuDv2 = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
        linefuSpeed2=vp_data[i].second;
    }else{
        linefuDv2=0;
        linefuSpeed2=0;
    }
	i = getIndex(t+startupTimeW);
    if (i==0) i=1;
    if(i<vp_data.size()){
        linefuDv3 = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
        linefuSpeed3=vp_data[i].second;
    }else{
        linefuDv3=0;
        linefuSpeed3=0;
    }
    i = getIndex(t+3);
    if (i==0) i=1;
    if(i<vp_data.size()){
        linefuDvLong = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
        linefuSpeedLong=vp_data[i].second;
    }else{
        linefuDvLong=0;
        linefuSpeedLong=0;
    }
    i = getIndex(t+2);
    if (i==0) i=1;
    if(i<vp_data.size()){
        linefuDvSubLong = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
        linefuSpeedSubLong=vp_data[i].second;
    }else{
        linefuDvSubLong=0;
        linefuSpeedSubLong=0;
    }
    i=getIndex(t-1);
    if (i==0) i=1;
    linelastSpeed=vp_data[i].second;   //无插值
    linelastDv = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
    i=getIndex(t-3);
    if (i==0) i=1;
    linelastSpeed3=vp_data[i].second;
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
//    printf("%f  %f  %f  %f %f  %f %f %f\n",error[2],accOpen,speed[3],conAcc[1],conBrake[1],timeGap,maxAccRecorder,time);

    getError();
	if (lineSpeed[3] == 0) {
		conAcc[0] = accOpen;
        conBrake[0] = brakeOpen;
	}
    //double nextTime = getNextLineDuration();
    //PRINTF(LOG_DEBUG, "%s: duration=%f\n", __func__, nextTime);
    if(lineSpeed[3]==0){
        if(linefuDv2 > 0){
            conBrake[1] = -100;
            conAcc[1] = 0;
        }
        else{
            conBrake[1] =100;
            conAcc[1]=-100;
        }
        return;
    }
    else {
        if (lineDv[2] > 0) {//加速行驶
            conBrake[1]=-100;
            if (linefuDv==0) {//将进入匀速段
                if (maxAccRefresh == 0) {
                    maxAccRefresh = 1;
                    maxAccRecorder = accOpen*(changepoint1to2+speed[3]/150.0);	//拐点参数1：accRecorder的比例系数
                }
                if(linefuSpeed==15&&accOpen>0.25*maxAccRecorder) conAcc[1]=-8;
                else if(accOpen>maxAccRecorder) conAcc[1]=-4;
                else conAcc[1]=changeAcc();  //std::max(changeAcc(),0.0);
                sysControlMethod=-1;
            }
            else if (linelastDv==0&&linelastSpeed>0){//刚驶离匀速段 //此处补偿易不足，曲线中仅出现一次
                if(error[2]>-2) conAcc[1]=changeAcc()+0.01*lineDv[2]*sqrt(speed[3]);
                else conAcc[1]=changeAcc();
            }
            /*
            else if(linefuDv<0){//将进入刹车段  //wltc用
                conAcc[1]=-100;
                maxAccRefresh = 0;
                sysControlMethod=4;
                conBrake[1]=changeBrake();
            }*/

            /**/
            else if(linelastSpeed==0){//零起步漂移
                /*
                if(accOpen<2*lineDv[2]-1){
                    conAcc[1]=2;
                    sysControlMethod=9;
                }
                else conAcc[1]=changeAcc();
                */
                conAcc[1]=0.25*changeAcc();
                return;
            }
            else {
                conAcc[1] = changeAcc();
            }
			return;
        }
        else if(lineDv[2]==0){//匀速行驶
            conBrake[1]=-100;
            if(linefuDv<0){//将进入刹车段  //拐点参数2：此处受刹车性能影响较大
                conAcc[1]=-100;
				maxAccRefresh = 0;
                sysControlMethod=-2;
                if(brakeOpen<-changepoint2to3*linefuDv){
                    conBrake[1]=changeBrake()+8;
                }else{
                    conBrake[1]=0;
                }
            }
            else if (linefuDv>0){//将进入加速段   //此处待考
				maxAccRefresh = 0;
                if(error[2]>-1.8&&((accOpen*(changepoint1to2+speed[3]/150.0))<maxAccRecorder)) {//此处回复原油门开度
                    conAcc[1]=2;
                }
                else conAcc[1]=changeAcc();
            }
            else if(linelastDv>0){//刚驶离加速段  //此处高速补偿过度，需增加限位变量
                if((error[2]<1.5||dverror[2]<0.0)&&accOpen>maxAccRecorder){
                    conAcc[1] = -4;
                    sysControlMethod = -1;
                    //maxAccRefresh=0;
                }
                else{
                    conAcc[1]=changeAcc();
                }
                //PRINTF(LOG_DEBUG, "%f+%f+%f+%f\n",error[2],accOpen,maxAccRecorder,conAcc[1]);
            }
            else conAcc[1] = changeAcc();
        }
        else if (lineDv[2] < 0) {//刹车阶段
            if(linefuDv==0&&linefuSpeed>5){//将再次进入匀速，适度补偿
                conBrake[1]=-100;
                if(accOpen<linefuSpeed*0.2) conAcc[1]=changeAcc()+1;
                else conAcc[1]=0;
            }
            else {
                conBrake[1] = changeBrake();
                conAcc[1]=-100;
            }
        }
    }
	conAcc[0] = conAcc[0] + conAcc[1];
	if (conAcc[0] < 0)conAcc[0] = 0;
	conBrake[0] = conBrake[0] + conBrake[1];//

    isTrain=0;
    //PRINTF(LOG_DEBUG, "%f+%f\n",maxAccRecorder,accOpen);
}

void SysControl::calConN(double time, double speedNow, double brakeOpen0, double accOpen0)
{
    //std::cout<<"time="<<time<<std::endl;

    //UNUSED(accOpen);
    timeGap = time - lastControlTime;   //求两次控制间的时间间隔
    //timeGap *= 1000.0;//ms
    //PRINTF(LOG_DEBUG, "timegap=%f\n",timeGap);   //应当是14ms
    lastControlTime = time;
    brakeOpen=brakeOpen0;
    accOpen=accOpen0;
    getSpeedDv(time,speedNow);
//    printf("%f  %f  %f %f  %f  %f %f  %f\n",error[2],accOpen,speed[3],conAcc[1],conBrake[1],timeGap,maxAccRecorder,time);

    getError();
    if (lineSpeed[3] == 0) {
        conAcc[0] = accOpen;
        conBrake[0] = brakeOpen;
    }
    //double nextTime = getNextLineDuration();
    //PRINTF(LOG_DEBUG, "%s: duration=%f\n", __func__, nextTime);
    //---------------------------------------------------------------------//
    if(lineSpeed[3]==-5||lineDv[2]>=100){   //free测试用
        conAcc[1]=-100;
    }

    //---------------------------------------------------------------------//
    else {
        if(speed[3]>=30&&error[2]<-2.2){  //1 检测上界误差，过大则切换刹车控制.2.3 2.5
            conAcc[1]=-100;
        }
        //else if(speed[3]<30&&error[2]<-2){
        //    conAcc[1]=-100;
        //    conBrake[1]=changeBrakeW();
        //}
        else if(lineDv[2]>0&&(linefuDv<0)&&lineSpeed[3]<50){
            if(error[2]<0){
                conAcc[1]=-100;
            }
            else {//此处存疑
                conAcc[1]=-wltc_degree_toLow_fix*4;
            }
        }
        else if ((linefuDv<0.6*lineDv[2])&&lineDv[2]>0){  //3 检测斜率骤降，记录maxAcc并减油门
            if (maxAccRefresh == 0||(time>maxAcctimer+2.0)) {   //Refresh标注是否需要更新maxAcc
                maxAccRefresh = 1;
                maxAcctimer=time;
                maxAccRecorder = accOpen*(0.18+0.3+speed[3]/300.0);	//拐点参数1：accRecorder的比例系数
            }
            if((error[2]<0)||((error[2]<1||(error[2]>2&&error[2]<2.5))&&lineSpeed[3]<40)&&linefuDv>0&&accOpen>maxAccRecorder) conAcc[1]=-wltc_degree_toLow_fix*4;
            else if((error[2]<0||error[2]>2)&&linefuDv<0&&accOpen>0.5*maxAccRecorder) conAcc[1]=-wltc_degree_toLow_fix*6;  //std::max(changeAcc(),0.0);
            else conAcc[1]=changeAccN();
            sysControlMethod=-1;
        }
        else if (lineDv[2]<=0){ //4 检测减速到加速拐点，附加特定加油量
            if(error[2]>-1.2){//1.5 1.2     高速段 0.4 0.2
                conAcc[1]=changeAccN() + startupTimeW*(linefuSpeedSubLong-lineDv[2])*sqrt(speed[3]);
                sysControlMethod=-90;
            }
            else
            {
                conAcc[1]=changeAccN();
                sysControlMethod=-95;
            }
        }
        else {
            conAcc[1] = changeAccN();
            sysControlMethod=-80;
        }
        return;
    }
    conAcc[0] = conAcc[0] + conAcc[1];
    if (conAcc[0] < 0)conAcc[0] = 0;
    conBrake[0] = conBrake[0] + conBrake[1];//

    isTrain=0;
    //PRINTF(LOG_DEBUG, "%f+%f\n",maxAccRecorder,accOpen);
}

double SysControl::changeAccN() {
    pidA[0][0]=accpidfixW*2;
    pidA[0][1]=accpidfixW*2;
    pidA[0][2]=accpidfixW*0.1;
    isTrain=1;
    if (isTrain == 1) {
        //测试用con1，屏蔽来自设置信号
        //double con1=1*5 * (error[2] - error[1]) + 4*0.2 * error[2] + 0.1 * (error[2] - 2 * error[1] + error[0]);

        //正常运行con1
        double con1=borderfixW*5 * (error[2] - error[1]) + borderfixW*0.2 * error[2] + 0.1 * (error[2] - 2 * error[1] + error[0]);
        double con2=0.5*con1;
        Q_UNUSED(con2);
        //con1=slidingAdjustion();

        if(lineDv[2]==0){
            if(error[2]<-1.0||error[2]>1.0){//改动2：原1.0
                return con1;
            }
            else{
                sysControlMethod=3;
                return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
            }
        }
        else{
            double lim = 1.5;
            if(error[2]<-lim||error[2]>lim){//改动3：原0.5 到 1.0 到 1.5(wltc)
                sysControlMethod=1;
                if(error[2]<0) return con1;
                else if(error[2]>0) return con1;
            }
            else{
                sysControlMethod=2;
                return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);

                //return pid[0][0] * (error[2] - error[1]) + pid[0][1] * error[2] + pid[0][2] * (error[2] - 2 * error[1] + error[0]);
            }
        }
    }
    PRINTF(LOG_DEBUG, "%s: this should never be reached!\n",__func__);
    return 0.0;
}

void SysControl::calConW(double time, double speedNow, double brakeOpen0, double accOpen0)
{
    //std::cout<<"time="<<time<<std::endl;

    //UNUSED(accOpen);
    timeGap = time - lastControlTime;   //求两次控制间的时间间隔
    //timeGap *= 1000.0;//ms
    //PRINTF(LOG_DEBUG, "timegap=%f\n",timeGap);   //应当是14ms
    lastControlTime = time;
    brakeOpen=brakeOpen0;
    accOpen=accOpen0;
    getSpeedDv(time,speedNow);
//    printf("%f  %f  %f %f  %f  %f %f  %f\n",error[2],accOpen,speed[3],conAcc[1],conBrake[1],timeGap,maxAccRecorder,time);

    getError();
    if (lineSpeed[3] == 0) {
        conAcc[0] = accOpen;
        conBrake[0] = brakeOpen;
    }
    //double nextTime = getNextLineDuration();
    //PRINTF(LOG_DEBUG, "%s: duration=%f\n", __func__, nextTime);
    //---------------------------------------------------------------------//
    if(lineSpeed[3]==-5||lineDv[2]>=100){   //free测试用
        conBrake[1]=-100;
        conAcc[1]=-100;
    }
    else if(lineSpeed[3]<=1){    //零速阶段
        if(linefuDv3 > 0.1&&time>5){    //此边界斜率决定是否将要进入加速段
            conBrake[1] = -100;
            conAcc[1] = 0;
        }
        else{
            conBrake[1] =100;
            conAcc[1]=-100;
        }
        return;
    }
    //---------------------------------------------------------------------//
    else {
        //time=time+1530;
        //time=time+1020;
        int time_decide=0;
        if(wltc_time_change==0) time_decide=(time>915&&time<920)||(time>1160&&time<1350)||(time>1550&&time<1727)||(time>1741&&time<1766);
        else if(wltc_time_change==1) time_decide=(time>612&&time<645)||(time>840&&time<900)||(time>930&&time<945)||(time>915&&time<920)||(time>1160&&time<1350)||(time>1550&&time<1727)||(time>1741&&time<1766);

        if (lineDv[2] >= 0||time_decide) {   //加速阶段
            //(time>612&&time<645)||(time>840&&time<900)||(time>922&&time<945)||
            //(time>1360&&time<1375)||

            conBrake[1]=-100;

            if(speed[3]>=30&&error[2]<-2.2){  //1 检测上界误差，过大则切换刹车控制.2.3 2.5
                conAcc[1]=-100;
                conBrake[1]=changeBrakeW();
            }
            //else if(speed[3]<30&&error[2]<-2){
            //    conAcc[1]=-100;
            //    conBrake[1]=changeBrakeW();
            //}
            else if(lineDv[2]>0&&(linefuDv<0)&&lineSpeed[3]<50){
                if(error[2]<0){
                    conAcc[1]=-100;
                    conBrake[1]=changeBrakeW();
                }
                else {//此处存疑
                    conBrake[1]=-100;
                    conAcc[1]=-wltc_degree_toLow_fix*4;
                }
            }
            else if ((linefuDv<0.6*lineDv[2])&&lineDv[2]>0){  //3 检测斜率骤降，记录maxAcc并减油门
                if (maxAccRefresh == 0||(time>maxAcctimer+2.0)) {   //Refresh标注是否需要更新maxAcc
                    maxAccRefresh = 1;
                    maxAcctimer=time;
                    maxAccRecorder = accOpen*(0.18+0.3+speed[3]/300.0);	//拐点参数1：accRecorder的比例系数
                }
                if((error[2]<0)||((error[2]<1||(error[2]>2&&error[2]<2.5))&&lineSpeed[3]<40)&&linefuDv>0&&accOpen>maxAccRecorder&&time_decide!=1) conAcc[1]=-wltc_degree_toLow_fix*4;
                else if((error[2]<0||error[2]>2)&&linefuDv<0&&accOpen>0.5*maxAccRecorder) conAcc[1]=-wltc_degree_toLow_fix*6;  //std::max(changeAcc(),0.0);
                else conAcc[1]=changeAccW();
                sysControlMethod=-1;
            }
            else if ((linelastDv<0)&&linelastSpeed>0&&lineDv[2]>=0){ //4 检测减速到加速拐点，附加特定加油量
                if(error[2]>-1.2){//1.5 1.2     高速段 0.4 0.2
                    if(lineSpeed[3]>=50) conAcc[1]=changeAccW()+0.2*(lineDv[2]-linelastDv)*sqrt(speed[3]);
                    else if(lineSpeed[3]>30) conAcc[1]=changeAccW()+0.1*(lineDv[2]-linelastDv)*sqrt(speed[3]);
                    else conAcc[1]=changeAccW();
                }
                else conAcc[1]=changeAccW();
            }
            else {
                conAcc[1] = changeAccW();
            }
            return;
        }
        else if (lineDv[2] < 0) {   //刹车阶段
            maxAccRefresh=0;
            if(linefuDv>=0&&linefuSpeed>5){    //1 检测减加速拐点，
                if(error[2]>-2){
                    conBrake[1]=-100;
                    if(accOpen<linefuSpeed&&lineSpeed[3]>=59){
                        if(time>1530&&time<1535) conAcc[1]=wltc_degree_fix*6.5;
                        else conAcc[1]=wltc_degree_fix*6;     //是否要添加斜率判断？
                    }
                    else if(accOpen<linefuSpeed*0.5&&linefuSpeed>25) conAcc[1]=wltc_degree_fix*3;
                    else conAcc[1]=changeAccW();
                }
                else{
                    conAcc[1]=-100;
                    conBrake[1]=changeBrakeW();
                }
            }
            else if((linelastDv<2*lineDv[2]||lineDv[2]>-1||linefuDv>-1||error[2]>2)&&lineSpeed[3]>5){  //2 检测刹车缓速段
                //缓速段判定——斜率变化，斜率绝对值（低于15边界大幅降低）
                double tempcon=pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
                Q_UNUSED(tempcon);
                //if(error[2]>=-1.0&&error[2]<=1.0){  //a 主体采用加速度控制
                //    conBrake[1]=-100;
                //    conAcc[1]=tempcon;
                //}
                if(error[2]<(wltc_lowborder_fix-0.2)){  //b 超上界切换刹车
                    conAcc[1]=-100;
                    conBrake[1]=0.5*changeBrakeW();
                }
                else if(error[2]>=(wltc_lowborder_fix-0.2)&&error[2]<wltc_lowborder_fix){
                    conAcc[1]=0;
                    conBrake[1]=0;
                }
                else{
                    conBrake[1]=-100;
                    conAcc[1]=0.5*changeAccW();
                }
            }
            /*
            else if(linelastDv>=0&&error[2]<1.5){   //3 检测加减速拐点，由brakeW处理
                conAcc[1]=-100;
                if(brakeOpen<-changepoint2to3*linefuDv){
                    conBrake[1]=changeBrakeW()+8;
                }else{
                    conBrake[1]=changeBrakeW();
                }
                conBrake[1]=changeBrakeW();
            }
            */
            else {
                conAcc[1]=-100;
                conBrake[1] = changeBrakeW();
            }
        }
    }
    conAcc[0] = conAcc[0] + conAcc[1];
    if (conAcc[0] < 0)conAcc[0] = 0;
    conBrake[0] = conBrake[0] + conBrake[1];//

    isTrain=0;
    //PRINTF(LOG_DEBUG, "%f+%f\n",maxAccRecorder,accOpen);
}

double SysControl::changeAcc() {
    pidA[0][0]= accpidfix*2;
    pidA[0][1]= accpidfix*2;
    pidA[0][2]= accpidfix*0.1;
	isTrain=1;
    if (isTrain == 1) {
        //测试用con1，屏蔽来自设置信号
        //double con1=1*5 * (error[2] - error[1]) + 4*0.2 * error[2] + 0.1 * (error[2] - 2 * error[1] + error[0]);

        //正常运行con1
        double con1=borderfix*5 * (error[2] - error[1]) + borderfix*0.2 * error[2] + 0.1 * (error[2] - 2 * error[1] + error[0]);

        //con1=slidingAdjustion();

        if(lineDv[2]==0){
            if(error[2]<-1.0||error[2]>1.0){//改动2：原1.0
                return con1;
            }
            else{
                sysControlMethod=3;
                return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
            }
        }
        else{
            if(error[2]<-1.0||error[2]>1.0){//改动3：原0.5 到 1.0 到 1.5(wltc)
                sysControlMethod=1;
                return con1;

            }
            else{
                sysControlMethod=2;
                return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);

                //return pid[0][0] * (error[2] - error[1]) + pid[0][1] * error[2] + pid[0][2] * (error[2] - 2 * error[1] + error[0]);
            }
        }

    }
    else if (isTrain == 0) {
        if (error[2] <= 0.5*border&&error[2]>=-0.5*border) {
            sysControlMethod=1;
            double con1;
            con1= pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
            //PRINTF(LOG_DEBUG, "%f\n",con1);
            return con1;
        }
        else if(error[2]<-1.8||(error[2]<-0.5*border&&dverror[2]<=0)){
            sysControlMethod=2;
            return 25*pid[0][0] * (error[2] - error[1]) + 2*pid[0][1] * error[2] + 1*pid[0][2] * (error[2] - 2 * error[1] + error[0]);
        }
        else if(error[2]<-0.5*border&&dverror[2]>0){
            sysControlMethod=3;
            return 20*pid[0][0] * (error[2] - error[1]) + 0.0001*pid[0][1] * error[2] + 0.0001*pid[0][2] * (error[2] - 2 * error[1] + error[0]);
        }
        else if(error[2]>1.8||(error[2]>0.5*border&&dverror[2]>=0)){
            sysControlMethod=4;
            return 25*pid[0][0] * (error[2] - error[1]) + 3*pid[0][1] * error[2] + 1*pid[0][2] * (error[2] - 2 * error[1] + error[0]);
        }
        else if(error[2]>0.5*border&&dverror[2]<0){
            sysControlMethod=5;
            float con5;
            con5=20*pid[0][0] * (error[2] - error[1]) + 0.0001*pid[0][1] * error[2] + 0.0001*pid[0][2] * (error[2] - 2 * error[1] + error[0]);

            //PRINTF(LOG_DEBUG, "con5=%f\n",con5);
            return con5;
        }
        /*
        else if(error[2]<-2*border||(error[2]<-border&&dverror[2]<=0)){
            sysControlMethod=2;
            return 10*pid[0][0] * (error[2] - error[1]) + pid[0][1] * error[2] + 10*pid[0][2] * (error[2] - 2 * error[1] + error[0]);
        }
        else if(error[2]>2*border||(error[2]>border&&dverror[2]>=0)){
            sysControlMethod=3;
            return 10*pid[0][0] * (error[2] - error[1]) + pid[0][1] * error[2] + 10*pid[0][2] * (error[2] - 2 * error[1] + error[0]);
        }
        else{
            sysControlMethod=4;
            return pid[0][0] * (error[2] - error[1]) + pid[0][1] * error[2] + pid[0][2] * (error[2] - 2 * error[1] + error[0]);
        }*/
    }
    PRINTF(LOG_DEBUG, "%s: this should never be reached!\n",__func__);
    return 0.0;
}

double SysControl::changeAccW() {
	pidA[0][0]=accpidfixW*2;
	pidA[0][1]=accpidfixW*2;
	pidA[0][2]=accpidfixW*0.1;
    isTrain=1;
    if (isTrain == 1) {
        //测试用con1，屏蔽来自设置信号
        //double con1=1*5 * (error[2] - error[1]) + 4*0.2 * error[2] + 0.1 * (error[2] - 2 * error[1] + error[0]);

        //正常运行con1
        double con1=borderfixW*5 * (error[2] - error[1]) + borderfixW*0.2 * error[2] + 0.1 * (error[2] - 2 * error[1] + error[0]);
        double con2=0.5*con1;
        Q_UNUSED(con2);
        //con1=slidingAdjustion();

        if(lineDv[2]==0){
            if(error[2]<-1.0||error[2]>1.0){//改动2：原1.0
                return con1;
            }
            else{
                sysControlMethod=3;
                return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
            }
        }
        else{
            if(error[2]<-1.5||error[2]>1.5){//改动3：原0.5 到 1.0 到 1.5(wltc)
                sysControlMethod=1;
                if(error[2]<0) return con1;
                else if(error[2]>0) return con1;
            }
            else{
                sysControlMethod=2;
                return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);

                //return pid[0][0] * (error[2] - error[1]) + pid[0][1] * error[2] + pid[0][2] * (error[2] - 2 * error[1] + error[0]);
            }
        }
    }
    PRINTF(LOG_DEBUG, "%s: this should never be reached!\n",__func__);
    return 0.0;
}

double SysControl::changeBrake() {
    if (isTrain == 2) {
        return pid[1][0] * error[2] + pid[1][1] * error[1] + pid[1][2] * error[0];
    }
    else  {
        if (linelastDv==0||lineDv[2]<1.5*linelastDv){
            return 30*pid[1][0] * (error[2] - error[1]) + 4*pid[1][1] * error[2] + pid[1][2] * (error[2] - 2 * error[1] + error[0]);
        }
        //else if (error[2] < 0.5*border&&error[2]>-0.5*border) {
            //sysControlMethod=6;
        //    return pidA[1][0] * (dverror[2] - dverror[1]) + pidA[1][1] * dverror[2] + pidA[1][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
        //}
        else if(error[2]<-1.5||(error[2]<-0.5*border&&dverror[2]<=0)){
            //sysControlMethod=7;
            return 40*pid[1][0] * (error[2] - error[1]) +1* pid[1][1] * error[2] + 10*pid[1][2] * (error[2] - 2 * error[1] + error[0]);
        }
        else if(error[2]<-0.5*border&&dverror[2]>0){
            //sysControlMethod=8;
            return 20*pid[1][0] * (error[2] - error[1]) + 0.001*pid[1][1] * error[2] + 5*pid[1][2] * (error[2] - 2 * error[1] + error[0]);
        }
        else if(error[2]>1.5||(error[2]>0.5*border&&dverror[2]>=0)){
            //sysControlMethod=9;
            return 40*pid[1][0] * (error[2] - error[1]) + 2*pid[1][1] * error[2] +10* pid[1][2] * (error[2] - 2 * error[1] + error[0]);
        }
        else if(error[2]>0.5*border&&dverror[2]<0){
            //sysControlMethod=10;
            return 20*pid[1][0] * (error[2] - error[1]) + 0.001*pid[1][1] * error[2] +5* pid[1][2] * (error[2] - 2 * error[1] + error[0]);
        }
        /*
        else if(error[2]<-2*border||(error[2]<-border&&dverror[2]<=0)){
            sysControlMethod=6;
            return 10*pid[1][0] * (error[2] - error[1]) + pid[1][1] * error[2] + 3*pid[1][2] * (error[2] - 2 * error[1] + error[0]);
        }
        else if(error[2]>2*border||(error[2]>border&&dverror[2]>=0)){
            sysControlMethod=7;
           return 10*pid[1][0] * (error[2] - error[1]) + pid[1][1] * error[2] +3* pid[1][2] * (error[2] - 2 * error[1] + error[0]);
        }
        */
        else{
            sysControlMethod=8;
            return 10*pid[1][0] * (error[2] - error[1]) + 0.00001*pid[1][1] * error[2] + pid[1][2] * (error[2] - 2 * error[1] + error[0]);
        }
    }
    PRINTF(LOG_DEBUG, "%s: this should never be reached!\n",__func__);
    return 0.0;
}

double SysControl::changeBrakeW() {
    if (linelastDv>=0||lineDv[2]<1.5*linelastDv){
        sysControlMethod=6;
        return 60*pid[1][0] * (error[2] - error[1]) + 6*pid[1][1] * error[2] + pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    //else if (error[2] < 0.5*border&&error[2]>-0.5*border) {
        //sysControlMethod=6;
    //    return pidA[1][0] * (dverror[2] - dverror[1]) + pidA[1][1] * dverror[2] + pidA[1][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
    //}
    else if(error[2]<-1.5||(error[2]<-0.5&&dverror[2]<=0)){
        sysControlMethod=7;
        return 40*pid[1][0] * (error[2] - error[1]) +4* pid[1][1] * error[2] + 10*pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]<-0.5&&dverror[2]>0){
        sysControlMethod=8;
        return 20*pid[1][0] * (error[2] - error[1]) + 0.001*pid[1][1] * error[2] + 5*pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]>1.5||(error[2]>0.5&&dverror[2]>=0)){
        sysControlMethod=9;
        return 40*pid[1][0] * (error[2] - error[1]) + 2*pid[1][1] * error[2] +10* pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else if(error[2]>0.5&&dverror[2]<0){
        sysControlMethod=10;
        return 20*pid[1][0] * (error[2] - error[1]) + 0.001*pid[1][1] * error[2] +5* pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    else{
        sysControlMethod=11;
        return pidA[1][0] * (dverror[2] - dverror[1]) + pidA[1][1] * dverror[2] + pidA[1][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
        //return 10*pid[1][0] * (error[2] - error[1]) + 0.00001*pid[1][1] * error[2] + pid[1][2] * (error[2] - 2 * error[1] + error[0]);
    }
    PRINTF(LOG_DEBUG, "%s: this should never be reached!\n",__func__);
    return 0.0;
}

double SysControl::getconAcc() {	//为相对控制量
    //if (conAcc[1] > 0.0 && accOpen<2.0) {//加油门 且 油门空行程
    //	return std::max(conAcc[1], 5.0);
    //}
    /*
    static int cnt=0;
    if(++cnt%10==0){
        printf("%f  %f  %f  %f %f  %f\n",error[2],accOpen,conAcc[1],speed[3],speed[2],timeGap);
    }*/
    if(lineSpeed[3]==15&&lineDv[2]==0&&conAcc[1]>0) conAcc[1]=0.5*conAcc[1];//需与changeACC配合
    if(lineSpeed[3]<30&&accOpen>50&&conAcc[1]>0) conAcc[1]=0;
    if(lineSpeed[3]<20&&lineSpeed[3]!=15&&lineDv[2]<1&&lineDv[2]>=0) conAcc[1]=0.5*conAcc[1];
    return accfix*conAcc[1];
}

double SysControl::getconBrake() {
    //if (conBrake[1] > 0.0 && brakeOpen<10.0) {//加刹车 且 刹车空行程
    //	return std::max(conBrake[1], 10.0);
    //}

    //static int cnt=0;
    //if(++cnt%10==0){
    //printf("%f+%f+%f+%f\n",error[2],brakeOpen,conBrake[1],linefuDv);
    //}

    return brakefix*conBrake[1];
}

int SysControl::GetSysControlMethod()
{
    return sysControlMethod;
}

double SysControl::sgn(double x) {
	if (x > 0.0) return 1;
	else if (x < 0.0) return -1;
    else return 0;
}

void SysControl::GetPIDParams(double *params)
{
    params[0]=pid[0][0];
    params[1]=pid[0][1];
    params[2]=pid[0][2];
}

void SysControl::GetInitParams(double *params1,double *params2){
    params1=initcontrolparams1;
    params2=initcontrolparams2;
    Q_UNUSED(params1);
    Q_UNUSED(params2);
}

/* -------------------- 挡位离合控制 -------------------- */
int SysControl::getnodeformanual(const int s)
{
    switch (s)
    {
    case 0:
    case 3:
    case 4:
        return 0;
        break;
    case 1:
    case 5:
    case 6:
        return 1;
        break;
    case 2:
    case 7:
    case 8:
        return 2;
        break;
    default:
        return -1;
        PRINTF(LOG_WARNING, "%s: (%d) has no matched nodeshift.\n", __func__, s);
        break;
    }
}

void SysControl::plantrace()
{
    const int cs = RobotParams::currentshiftindex;
    const int as = RobotParams::aimshiftindex;
    const int ns_cs = getnodeformanual(cs);
    const int ns_as = getnodeformanual(as);

    if (Configuration::GetInstance()->ifGoBack)
    {
        int prelen = 0;
        if (ns_cs == 1)
        {
            if (cs == 1)
            {
                prelen = 0;
            }
            else
            {
                prelen = 1;
                RobotParams::shiftrunpath[0] = cs;
            }
        }
        else
        {
            prelen = 2;
            RobotParams::shiftrunpath[0] = cs;
            RobotParams::shiftrunpath[1] = ns_cs;
        }

        RobotParams::shiftrunpath[prelen] = 1;
        prelen++;

        if (ns_as == 1)
        {
            if (as == 1)
            {
                RobotParams::shiftrunlength = prelen;
            }
            else
            {
                RobotParams::shiftrunlength = prelen + 1;
                RobotParams::shiftrunpath[prelen] = as;
            }
        }
        else
        {
            RobotParams::shiftrunlength = prelen + 2;
            RobotParams::shiftrunpath[prelen] = ns_as;
            RobotParams::shiftrunpath[prelen + 1] = as;
        }
    }
    else
    {
        if (ns_cs == ns_as)
        {
            RobotParams::shiftrunlength = 2;
            RobotParams::shiftrunpath[0] = cs;
            RobotParams::shiftrunpath[1] = as;
        }
        else
        {
            if (ns_cs == cs)
            {
                RobotParams::shiftrunlength = 3;
                RobotParams::shiftrunpath[0] = cs;
                RobotParams::shiftrunpath[1] = ns_as;
                RobotParams::shiftrunpath[2] = as;
            }
            else if (ns_as == as)
            {
                RobotParams::shiftrunlength = 3;
                RobotParams::shiftrunpath[0] = cs;
                RobotParams::shiftrunpath[1] = ns_cs;
                RobotParams::shiftrunpath[2] = as;
            }
            else
            {
                RobotParams::shiftrunlength = 4;
                RobotParams::shiftrunpath[0] = cs;
                RobotParams::shiftrunpath[1] = ns_cs;
                RobotParams::shiftrunpath[2] = ns_as;
                RobotParams::shiftrunpath[3] = as;
            }
        }
    }

    RobotParams::shiftrunpointer = 0;
}

bool SysControl::ifreachedshift(const bool ifmanual, const int aimindex)
{
    double err[2] = {0.0, 0.0};

    if (ifmanual)
    {
        err[0] = Configuration::GetInstance()->angleErr_M[1];
        err[1] = Configuration::GetInstance()->angleErr_M[2];
    }
    else
    {
        err[0] = Configuration::GetInstance()->angleErr_A[1];
        err[1] = Configuration::GetInstance()->angleErr_A[2];
    }

    if ( fabs(RobotParams::angleRealTime[3] - Configuration::GetInstance()->shiftAxisAngles1[aimindex]) < err[0] && fabs(RobotParams::angleRealTime[4] - Configuration::GetInstance()->shiftAxisAngles2[aimindex]) < err[1] )
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool SysControl::ifreachedshiftprocess(const int startindex, const int aimindex)
{
    double err = max(Configuration::GetInstance()->angleErr_P[1] * fabs(Configuration::GetInstance()->shiftAxisAngles1[aimindex] - Configuration::GetInstance()->shiftAxisAngles1[startindex]) / 100.0, Configuration::GetInstance()->angleErr_P[2] * fabs(Configuration::GetInstance()->shiftAxisAngles2[aimindex] - Configuration::GetInstance()->shiftAxisAngles2[startindex]) / 100.0);

    if ( fabs(RobotParams::angleRealTime[3] - Configuration::GetInstance()->shiftAxisAngles1[aimindex]) < err && fabs(RobotParams::angleRealTime[4] - Configuration::GetInstance()->shiftAxisAngles2[aimindex]) < err )
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool SysControl::ifreachedclutch(const bool ifmanual, const int aimindex)
{
    double err = 0.0;

    if (ifmanual)
    {
        err = Configuration::GetInstance()->angleErr_M[0];
    }
    else
    {
        err = Configuration::GetInstance()->angleErr_A[0];
    }

    if ( fabs(RobotParams::angleRealTime[2] - Configuration::GetInstance()->clutchAngles[aimindex]) < err )
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool SysControl::getconSft(double *conparas, const unsigned int round)
{
    const int startindex = RobotParams::currentshiftindex;
    const int stopindex = RobotParams::aimshiftindex;
    const double startPos[2] = {Configuration::GetInstance()->shiftAxisAngles1[startindex], Configuration::GetInstance()->shiftAxisAngles2[startindex]};
    const double stopPos[2] = {Configuration::GetInstance()->shiftAxisAngles1[stopindex], Configuration::GetInstance()->shiftAxisAngles2[stopindex]};

    const double errPos[2] = {fabs(startPos[0] - stopPos[0]), fabs(startPos[1] - stopPos[1])};

    // 两个轴运动都大 运动不解耦
    if (errPos[0] > Configuration::GetInstance()->angleErr_A[1] && errPos[1] > Configuration::GetInstance()->angleErr_A[2])
    {
        PRINTF(LOG_ERR, "%s: No coupled motion.\n", __func__);
        return false;
    }

    // 两个轴运动都小 直接到位置
    if (errPos[0] < Configuration::GetInstance()->angleErr_A[1] && errPos[1] < Configuration::GetInstance()->angleErr_A[2])
    {
        PRINTF(LOG_INFO, "%s: Small motion.\n", __func__);
        *conparas = stopPos[0];
        *(conparas + 1) = stopPos[1];
        return true;
    }

    // 判断哪个轴为主运动
    int whichAxis;
    if (errPos[0] > errPos[1])
    {
        whichAxis = 0;
    }
    else
    {
        whichAxis = 1;
    }

    // 计算主运动轴目标值
    /* 曲线运动 */
//    const double speed = Configuration::GetInstance()->curveMotionSpeed[whichAxis + 1];
//    const double t_next = round * speed;

//    double x_next = 1/(1 + qExp(5.5 - 11 * t_next));
//    if (x_next > 0.995) x_next = 1; // 0.995 = 1/(1 + qExp(5.5 - 11 * 0.98))

//    const double pos = x_next * (stopPos[whichAxis] - startPos[whichAxis]) + startPos[whichAxis];
    /* */

    /* 分段线性*/
    const double speed = Configuration::GetInstance()->curveMotionSpeed[whichAxis + 1];
    double thresholdround = qCeil(0.9 * errPos[whichAxis] / speed);

    double pos = startPos[whichAxis] + sgn(stopPos[whichAxis] - startPos[whichAxis]) * round * speed;
    if ((pos - startPos[whichAxis])/(stopPos[whichAxis] - startPos[whichAxis]) > 1.0) pos = stopPos[whichAxis];
    else if ((pos - startPos[whichAxis])/(stopPos[whichAxis] - startPos[whichAxis]) > 0.9)
    {
        pos = startPos[whichAxis] + sgn(stopPos[whichAxis] - startPos[whichAxis]) * thresholdround * speed + sgn(stopPos[whichAxis] - startPos[whichAxis]) * (round - thresholdround) * speed / 2;
    }
    /* */


    // 返回计算结果
    if (whichAxis == 0)
    {
        *conparas = pos;
        *(conparas + 1) = stopPos[1];
    }
    else
    {
        *conparas = stopPos[0];
        *(conparas + 1) = pos;
    }

    return true;
}

bool SysControl::getconClh(double *conparas, const unsigned int round)
{
    const int startindex = RobotParams::currentclutchindex;
    const int stopindex = RobotParams::aimclutchindex;
    const double startPos = Configuration::GetInstance()->clutchAngles[startindex];
    const double stopPos = Configuration::GetInstance()->clutchAngles[stopindex];

    const double errPos = fabs(startPos - stopPos);

    // 运动小 直接到位置
    if (errPos < Configuration::GetInstance()->angleErr_A[0])
    {
        PRINTF(LOG_INFO, "%s: Small motion.\n", __func__);
        *conparas = stopPos;
        return true;
    }

    // 计算运动目标值
    /* 曲线运动 */
//    const double speed = Configuration::GetInstance()->curveMotionSpeed[0];
//    const double t_next = round * speed;

//    double x_next = 1/(1 + qExp(5.5 - 11 * t_next));
//    if (x_next > 0.995) x_next = 1; // 0.995 = 1/(1 + qExp(5.5 - 11 * 0.98))

//    const double pos = x_next * (stopPos - startPos) + startPos;
    /* */

    /* 分段线性*/
    const double speed = Configuration::GetInstance()->curveMotionSpeed[0];
    double thresholdround = qCeil(0.9 * errPos / speed);

    double pos = startPos + sgn(stopPos - startPos) * round * speed;
    if ((pos - startPos)/(stopPos - startPos) > 1.0) pos = stopPos;
    else if ((pos - startPos)/(stopPos - startPos) > 0.9)
    {
        pos = startPos + sgn(stopPos - startPos) * thresholdround * speed + sgn(stopPos - startPos) * (round - thresholdround) * speed / 2;
    }
    /* */
    // 返回计算结果
    *conparas = pos;

    return true;

}

bool SysControl::ifreachedatinitial(const double angle_err)
{
    if ( fabs(RobotParams::angleRealTime[2] - Configuration::GetInstance()->clutchAngles[1]) < angle_err )
    {
        if ( fabs(RobotParams::angleRealTime[3] - Configuration::GetInstance()->shiftAxisAngles1[1]) < angle_err && fabs(RobotParams::angleRealTime[4] - Configuration::GetInstance()->shiftAxisAngles2[1]) < angle_err )
        {
            if (Configuration::GetInstance()->ifManualShift)
            {
                if ( fabs(RobotParams::angleRealTime[0] - Configuration::GetInstance()->deathPos[0]) < angle_err * 2.0 && fabs(RobotParams::angleRealTime[1] - Configuration::GetInstance()->deathPos[1]) < angle_err * 2.0 )
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                if ( fabs(RobotParams::angleRealTime[0] - Configuration::GetInstance()->brakeThetaAfterGoHome) < angle_err * 2.0 && fabs(RobotParams::angleRealTime[1] - Configuration::GetInstance()->deathPos[1]) < angle_err * 2.0 )
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}
