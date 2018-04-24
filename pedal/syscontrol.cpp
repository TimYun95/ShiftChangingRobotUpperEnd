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

void SysControl::train(const std::string& filename) {
    PRINTF(LOG_DEBUG, "%s: receive file=%s\n",__func__, filename.c_str());
    if(QMessageBox::question(NULL,"inform",QString("确定开始学习?"), QMessageBox::Ok, QMessageBox::Cancel) == QMessageBox::Cancel){
        return;
    }
    ifstream dataFile;
    ofstream outFile;
    dataFile.open(filename, std::fstream::binary);
    if(dataFile.bad()){
        QMessageBox::information(NULL,"inform",QString("无法打开学习样本文件\n")+filename.c_str());
        return;
    }
    vector<double> in,out1,out2,aim;
    double d[10];
    while (!dataFile.eof()) {
        dataFile >> d[0] >> d[1] >> d[2] >> d[3] >> d[4] >> d[5]>>d[6]>>d[7]>>d[8]>>d[9];
        aim.push_back(d[9]);
        in.push_back(d[9]-d[8]);
        out1.push_back(d[7]);
        out2.push_back(d[6]);
    }
    dataFile.close();
    double cos1, cos2, cos3, mtap, e, e1, e2, y, ym;
    double nita = 1;
    while (1) {
        cos1 = 0;
        cos2 = 0;
        cos3 = 0;
        mtap = 0;
        for(size_t i = 2; i < in.size(); i++) {
            if (aim[i] - aim[i - 1] >= 0) {
                e = in[i];
                e1 = in[i - 1];
                e2 = in[i - 2];
                y = out1[i]-out1[i-1];
                ym = pid[0][0] * e + pid[0][1] * e1 + pid[0][2] * e2;
                cos1 += (ym - y)*e;
                cos2 += (ym - y)*e1;
                cos3 += (ym - y)*e2;
                mtap += 1;
            }
        }
        pid[0][0] -= nita*cos1 / mtap;
        pid[0][1] -= nita*cos2 / mtap;
        pid[0][2] -= nita*cos3 / mtap;
        if(fabs(cos1) + fabs(cos2) + fabs(cos3) < 0.0001){
            break;
        }
    }
    while (1) {
        cos1 = 0;
        cos2 = 0;
        cos3 = 0;
        mtap = 0;
        for(size_t i =2; i < in.size(); i++) {
            if (aim[i] - aim[i - 1] < 0) {
                e = in[i];
                e1 = in[i - 1];
                e2 = in[i - 2];
                y = out2[i]-out2[i-1];
                ym = pid[1][0] * e + pid[1][1] * e1 + pid[1][2] * e2;
                cos1 += (ym - y)*e;
                cos2 += (ym - y)*e1;
                cos3 += (ym - y)*e2;
                mtap += 1;
            }
        }
        pid[1][0] -= nita*cos1 / mtap;
        pid[1][1] -= nita*cos2 / mtap;
        pid[1][2] -= nita*cos3 / mtap;
        if(fabs(cos1) + fabs(cos2) + fabs(cos3) < 0.0001){
            break;
        }
    }

    outFile.open("parameters", std::fstream::binary);
    for(int i=0; i<2; ++i){
        for(int j=0; j<3; ++j){
            outFile<<pid[i][j];
        }
        outFile<<"\n";
    }
    outFile.close();

    isTrain = 1;
}

void SysControl::onlineTraining() {
    double xitep = 1;
    double xitei = 0.00000;
    double xited = 0.1;
    double K = 25;
    double dw1=0;
    double dw2=0;
    double dw3=0;
	double x1=error[2] - error[1];
	double x2=error[2];
	double x3=error[2] - 2*error[1]+error[0];
	
	/*
	w1 = w1_1 + xitep*accOpen*(error[2] - error[1]);
	w2 = w2_1 + xitei*accOpen*(error[2] );
	w3 = w3_1 + xited*accOpen*(error[2] - 2*error[1]+error[0]);
    */

    dw1=xitep*sqrt(accOpen)*error[2]*x1;
    dw2=xitei*sqrt(accOpen)*error[2]*x2;
    dw3=xited*sqrt(accOpen)*error[2]*x3;
    if(dw1>0.1) dw1=0.1;
    else if(dw1<-0.1) dw1=-0.1;
    if(dw3>0.01) dw3=0.01;
    else if(dw3<-0.01) dw3=-0.01;
    //dw1=std::max(dw1,-0.002);
    //dw1=std::min(dw1,0.002);
	//dw3= std::max(dw3, -0.001);
	//dw3= std::min(dw3, 0.001);
    w1 = w1_1 + dw1;
    w2 = w2_1 + dw2;
    w3 = w3_1 + dw3;
    //K=(60-40)*(fabs(error[2])-0.0)*(fabs(error[2])-0.0)/((1.5-0.0)*(1.5-0.0))+40;
	
    /*
	//调1
	Tv_2=Tv_1;//初值为1
	K_2=K_1;//初值为K
	w1 = w1_1 + xitep*sqrt(accOpen)*error[2]*(2*error[2] - error[1]);
    w2 = w2_1 + xitei*sqrt(accOpen)*error[2]*(2*error[2] - error[1]);
    w3 = w3_1 + xited*sqrt(accOpen)*error[2]*(2*error[2] - error[1]);
	if(error[2]*error[1]>0) K_1=K_2+0.02*K_2/Tv_2;
	else K_1=0.75*K_2;
	Tv_1=Tv_2+0.05*sgn(fabs(error[2] - error[1])-Tv_2*fabs(error[2] - 2*error[1]+error[0]));
	//调2
	dw1=xitep*sqrt(accOpen)*error[2]*x1;
    dw2=xitei*sqrt(accOpen)*error[2]*x2;
    dw3=xited*sqrt(accOpen)*error[2]*x3;
	w1 = w1_1 + dw1;
    w2 = w2_1 + dw2;
	w3 = w3_1 + dw3;
	if(fabs(error[2])>1.5) K=20;
	else if(fabs(error[2])<0.5) K=5;
	else K=(20-5)*(fabse(error[2])-0.5)*(fabse(error[2])-0.5)/((1.5-0.5)*(1.5-0.5))+5；
	//二次型
    double K1=10;
    double K2=1;
    double K3=0.1;
    wtotal=w1*x1+w2*x2+w3*x3;
    dw1=xitep*K1*(K2*error[2]*x1-K3*wtotal*x1);
    dw2=xitei*K1*(K2*error[2]*x2-K3*wtotal*x2);
    dw3=xited*K1*(K2*error[2]*x3-K3*wtotal*x3);
    if(dw1>1) dw1=1;
    else if(dw1<-1) dw1=-1;
    if(dw3>0.01) dw3=0.01;
    else if(dw3<-0.01) dw3=-0.01;
    w1 = w1_1 + dw1;
    w2 = w2_1 + dw2;
    w3 = w3_1 + dw3;
	*/

    w1 = std::max(w1, 0.0);
	w2 = std::max(w2, 0.0);
	w3 = std::max(w3, 0.0);
    wtotal = fabs(w1) + fabs(w2) + fabs(w3);
    wtotal=std::max(wtotal,1.00);
	pid[0][0] =K* w1 / wtotal;
	pid[0][1] =K* w2 / wtotal;
	pid[0][2] =K* w3 / wtotal;
    qDebug()<<wtotal;
	w1_1 = w1;
	w2_1 = w2;
	w3_1 = w3;
    isTrain=1;
}

void SysControl::onlineTraining2(){
    double xitep = 5;
    double xitei = 0.00000;
    double xited = 0.01;
    double xite=1;
    double alfa=0.01;
    Q_UNUSED(alfa);
	double x1=error[2] - error[1];
	double x2=error[2];
    Q_UNUSED(x2);
	double x3=error[2] - 2*error[1]+error[0];
    Q_UNUSED(x3);
	double rbfinput[3];
    rbfinput[0]=conAcc[1];
	rbfinput[1]=speed[3];
	rbfinput[2]=speed[2];
	double hsum[10];
	double ym=0;
	double yu=0;
	for(int i=0;i<10;i++){
		hsum[i]=0;
		for(int j=0;j<3;j++){
			hsum[i]+=(rbfinput[j]-cj[i][j])*(rbfinput[j]-cj[i][j]);
		}
		ym+=wj[i]*std::exp(-hsum[i]/(2*bj[i]*bj[i]));
	}
	for(int i=0;i<10;i++){
		wj[i]+=xite*(speed[3]-ym)*std::exp(-hsum[i]/(2*bj[i]*bj[i]));//未加入动量因子alfa*（wj_1-wj_2);
		bj[i]+=xite*(speed[3]-ym)*wj[i]*std::exp(-hsum[i]/(2*bj[i]*bj[i]))*hsum[i]/(bj[i]*bj[i]*bj[i]);
		for(int j=0;j<3;j++){
			cj[i][j]+=xite*(speed[3]-ym)*wj[i]*std::exp(-hsum[i]/(2*bj[i]*bj[i]))*(rbfinput[i]-cj[i][j])/(bj[i]*bj[i]);
		}
	}
	for(int i=0;i<10;i++){
		yu+=wj[i]*std::exp(-hsum[i]/(2*bj[i]*bj[i]))*(cj[i][0]-rbfinput[0])/(bj[i]*bj[i]);
	}
    if(yu>2) yu=2;
    else if(yu<-2) yu=-2;
    printf("yu=%f==%f\n",yu,x1);
    w1+=xitep*error[2]*yu*x1;
    w2+=xitei*error[2]*yu*x1;
    w3+=xited*error[2]*yu*x1;
    pid[0][0]=w1;
    pid[0][1]=w2;
    pid[0][2]=w3;
	isTrain=1;
}

double SysControl::slidingAdjustion(){
    double s=5*error[2]+dverror[2];
    return 0.1*error[2]+2*sgn(s);
}

void SysControl::Init(const PairData& data, const Configuration* conf) {
    maxAccRefresh = 0;
    maxAcctimer=0;
    timeGap = 24;
    vp_data = data;
    isTrain = 0;
    border = 1.2;
	
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
    w1 = 10*pid[0][0];
    w2 = 10*pid[0][1]/1000;
    w3 = 10*pid[0][2];
	w1_1 = w1;
	w2_1 = w2;
	w3_1 = w3;
	
	for(int i=0;i<10;i++){
        bj[i]=10;
        wj[i]=0.1;
		for(int j=0;j<3;j++){
			cj[i][j]=0;
		}
	}
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

double SysControl::getNextLineDuration()
{
    size_t i = indexNow;//当前时间在i-1到i段
    if(i >= vp_data.size()-1){//最后一段
        return 0.0;
    }
    return vp_data[i+1].first-vp_data[i].first;//i到i+1段的时间
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
    lineDv[0] = lineDv[1];
    lineDv[1] = lineDv[2];
    lineDv[2] = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
    lineSpeed[0] = lineSpeed[1];
    lineSpeed[1] = lineSpeed[2];
    lineSpeed[2] = lineSpeed[3];
    lineSpeed[3] = (t - vp_data[i - 1].first)*lineDv[2] + vp_data[i - 1].second;

    i = getIndex(t+1);
    if(i<vp_data.size()){
        linefuDv = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
        linefuSpeed=vp_data[i].second;
    }else{
        linefuDv=0;
        linefuSpeed=0;
    }
    i = getIndex(t+startupTime);
    if(i<vp_data.size()){
        linefuDv2 = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
        linefuSpeed2=vp_data[i].second;
    }else{
        linefuDv2=0;
        linefuSpeed2=0;
    }
	i = getIndex(t+startupTimeW);
    if(i<vp_data.size()){
        linefuDv3 = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
        linefuSpeed3=vp_data[i].second;
    }else{
        linefuDv3=0;
        linefuSpeed3=0;
    }
    i = getIndex(t+3);
    if(i<vp_data.size()){
        linefuDvLong = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
        linefuSpeedLong=vp_data[i].second;
    }else{
        linefuDvLong=0;
        linefuSpeedLong=0;
    }
    i=getIndex(t-1);
    linelastSpeed=vp_data[i].second;   //无插值
    linelastDv = (vp_data[i].second - vp_data[i - 1].second) / (vp_data[i].first - vp_data[i - 1].first);
    i=getIndex(t-3);
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
    printf("%f  %f  %f  %f %f  %f %f %f\n",error[2],accOpen,speed[3],conAcc[1],conBrake[1],timeGap,maxAccRecorder,time);

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
    printf("%f  %f  %f %f  %f  %f %f  %f\n",error[2],accOpen,speed[3],conAcc[1],conBrake[1],timeGap,maxAccRecorder,time);

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
                onlineTraining();
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
                onlineTraining();
                return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);

                //return pid[0][0] * (error[2] - error[1]) + pid[0][1] * error[2] + pid[0][2] * (error[2] - 2 * error[1] + error[0]);
            }
        }
    }
    PRINTF(LOG_DEBUG, "%s: this should never be reached!\n",__func__);
    return 0.0;
}

double SysControl::changeHoldon(){
	if(error[2]<-1.5||error[2]>1.5||(error[2]<-0.5&&dverror[2]<=0)||(error[2]>0.5&&dverror[2]>=0)){
            sysControlMethod=1;
            return 25 * (error[2] - error[1]) + 0.4 * error[2] + 0.1 * (error[2] - 2 * error[1] + error[0]);
    }
    else {
		return  pidA[0][0] * (dverror[2] - dverror[1]) + pidA[0][1] * dverror[2] + pidA[0][2] * (dverror[2] - 2 * dverror[1] + dverror[0]);
	}
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
        PRINTF(LOG_ERR, "%s: No coupled motion.", __func__);
        return false;
    }

    // 两个轴运动都小 直接到位置
    if (errPos[0] < Configuration::GetInstance()->angleErr_A[1] && errPos[1] < Configuration::GetInstance()->angleErr_A[2])
    {
        PRINTF(LOG_INFO, "%s: Small motion.", __func__);
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
//    const double actualPos[2] = {RobotParams::angleRealTime[3], RobotParams::angleRealTime[4]};
//    double x_here = (actualPos[whichAxis] - startPos[whichAxis])/(stopPos[whichAxis] - startPos[whichAxis]);
//    if (x_here < 0.005) x_here = 0;

//    double t_next = Configuration::GetInstance()->curveMotionSpeed[whichAxis + 1];
//    if (x_here != 0)
//    {
//        t_next = (5.5 - qLn(1/x_here - 1))/11 + Configuration::GetInstance()->curveMotionSpeed[whichAxis + 1];
//    }
//    double x_next = 1/(1 + qExp(5.5 - 11 * t_next));
//    if (x_next > 0.995) x_next = 1; // 0.995 = 1/(1 + qExp(5.5 - 11 * 0.98))
//    const double pos = x_next * (stopPos[whichAxis] - startPos[whichAxis]) + startPos[whichAxis];



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
        PRINTF(LOG_INFO, "%s: Small motion.", __func__);
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
