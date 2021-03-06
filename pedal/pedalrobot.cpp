#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include "pedalrobot.h"

#include <sys/time.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <fstream>
#include <sstream>

#include <QLabel>
#include <QVector>
#include <QPalette>
#include <QFileDialog>
#include <QMessageBox>
#include <QDate>
#include <QTime>
#include <QFile>

#include "printf.h"
#include "robotparams.h"
#include "fileoperation/normalfile.h"
#include "settingwidget/settingwidgetpedalrobotgetspeed.h"
#include "autodriverobotapiclient.h"
#include "robotapi/AssistantFunc/fileassistantfunc.h"

const int fileCurve = 0; // 示教文件的曲线
const int actionCurve = 1; // 动作曲线
const int upperCurve = 2; // 速度上界
const int lowerCurve =3 ; // 速度下界
const double minY = -5.0; // 速度下限

PedalRobot::PedalRobot(QCustomPlot *_widget, Configuration *_conf)
    :pQCustomPlot(_widget), conf(_conf)
{
    myLogger = new Logger();
    myLogger->clear();
    myLogger->EnableCustomizedLogger(true);
    autoSaveLogger = false;

    mySysControl = new SysControl();

    InitParameters();
    InitQCustomPlot(20.0);
}

PedalRobot::~PedalRobot()
{
    delete mySysControl;
    delete myLogger;
}

void PedalRobot::SoftStop()
{
    isControlling=false;
    PRINTF(LOG_DEBUG, "%s: ...\n",__func__);

    CheckIfSaveLogger();
}

bool PedalRobot::SelectSpeedCurve(const bool selectFile)
{
    int ret = 0;

    //1) 选择曲线文件
    if(selectFile){
        QString str = QFileDialog::getOpenFileName(NULL, QString("请选择实验曲线文件"), (Configuration::mainFolder+"/stdand_files/").c_str());
        if(str==""){
            return false;
        }else if( !QFile::exists(str) ) {
            QMessageBox::information(NULL, "Warning", QObject::tr("所选定的文件不存在，请重新选择！")) ;
            return false;
        }
        curveFilePath = str.toStdString();
        conf->defaultFile = curveFilePath+"_ARM";
    }else{//使用上一次的文件
        size_t index = conf->defaultFile.find("_ARM");
        std::string str = conf->defaultFile;
        str.resize(index);//去除_ARM的尾部
        curveFilePath = str;
    }
    if( !QFile::exists(conf->defaultFile.c_str()) ){
        QMessageBox::information(NULL, "Warning", QObject::tr("用于机器人控制的文件不存在:\n") + conf->defaultFile.c_str());
        return false;
    }

    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

    ret = QMessageBox::information(
                NULL,"提示", QString(curveFilePath.c_str()) + QObject::tr("请再次确认挡位离合信息后发动车辆!\n点击确认后开始运行!"),
                QObject::tr("确认"),QObject::tr("取消"));

    if(ret == 1){
        PRINTF(LOG_DEBUG, "%s...cancel\n", __func__);
        return false;
    }
    else if (ret == 0)
    {
        if (RobotParams::powerMode != PedalRobot::Run)
        {
            QMessageBox::information(NULL,"警告", QString("检测到车辆并没有发动，请重试！"));
            PRINTF(LOG_WARNING, "%s: car is checked not lauched.\n", __func__);
            return false;
        }
        else
        {
            conf->SaveToFile();
            return true;
        }
    }
    else
    {
        PRINTF(LOG_DEBUG, "%s: unknown choice...cancel\n", __func__);
        return false;
    }
}

void PedalRobot::StartQCustomPlot(const std::__cxx11::string &fileNameARM)
{
    std::string fileName = fileNameARM.substr(0, fileNameARM.length()-4); // 去除"_ARM"=4 为供UI使用的示教文件
    if(ReadListeningTeachFile(fileName) == -1){
        QMessageBox::information(NULL,"warning",QString("无法读取该目标曲线文件!")+fileName.c_str());
        return;
    }

    size_t sz = vp_data.size();
    QVector<double> time(sz), speed(sz);
    QVector<double> upperTime(sz), upperSpeed(sz);
    QVector<double> lowerTime(sz), lowerSpeed(sz);
    const double maxY = CalculateUpperLowerBound(time, speed, upperTime, upperSpeed, lowerTime, lowerSpeed);

    InitQCustomPlot(maxY);
    pQCustomPlot->graph(fileCurve)->setData(time,speed);
    pQCustomPlot->graph(upperCurve)->setData(upperTime,upperSpeed);
    pQCustomPlot->graph(lowerCurve)->setData(lowerTime,lowerSpeed);
    pQCustomPlot->replot();

    vp_data_upperbound.clear();
    vp_data_upperbound.reserve(sz);
    vp_data_lowerbound.clear();
    vp_data_lowerbound.reserve(sz);
    for (size_t i = 0; i < sz; i++)
    {
        vp_data_upperbound.push_back( std::make_pair(upperTime[i],upperSpeed[i]) );
        vp_data_lowerbound.push_back( std::make_pair(lowerTime[i],lowerSpeed[i]) );
    }

    // 运动时间的记录
    gettimeofday(&actionStartTime, NULL);
    PRINTF(LOG_DEBUG, "%s: actionStartTime=%ld:%ld\n", __func__, actionStartTime.tv_sec, actionStartTime.tv_usec);
    actionDurationSecond = time[sz-1] - time[0];
    PRINTF(LOG_DEBUG, "%s: actionDurationSecond=%f\n",__func__,actionDurationSecond);

    InitControlMethod();

    // 准备日志数据
    myLogger->clear();
    myLogger->Customize("ControlParams:\n");
    if(conf->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::NedcControl){
        myLogger->Customize("Run as NEDC: ");
        for(size_t i=0; i<sizeof(conf->sysControlParams)/sizeof(conf->sysControlParams[0]); ++i){
            myLogger->Customize(conf->sysControlParams[i]).Customize("\t");
        }
    }else{
        myLogger->Customize("Run as WLTC: ");
        for(size_t i=0; i<sizeof(conf->sysControlParamsWltc)/sizeof(conf->sysControlParamsWltc[0]); ++i){
            myLogger->Customize(conf->sysControlParamsWltc[i]).Customize("\t");
        }
    }
    myLogger->Customize("\n");
    myLogger->Customize("TimeStamp BrakeOpenValue AccOpenValue CANSpeed PULSESpeed TargetSpeed ControlMethod ConBrake ConAcc BrakeAxis AccAxis CurrentTime Shift Clutch ClutchAxis ShiftAxis1 ShiftAxis2 SteeringAxis\n");
    autoSaveLogger = true;
    PRINTF(LOG_DEBUG, "%s: logger is cleared and ready!\n", __func__);

    timeStamp=0;
    isControlFinished = false;
    isControlling=true;

    changeShiftIndex = 0;
}

void PedalRobot::UpdatePart1()
{
//    CheckTimerAccuracy(); // 校验时间差
    return;
}

void PedalRobot::UpdatePart2()
{
    if (!isControlling)
    {
        return;
    }

    // 绘图
    UpdateQCustomPlot();

    // 非换挡情况下 执行油门刹车踏板计算
    double values[RobotParams::axisNum];
    //int ifABS[RobotParams::axisNum];
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        values[i] = 0; //ifABS[i] = 0;
    }

    if(conf->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::NedcControl){//NEDC曲线
        mySysControl->calCon(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);
    }else if(conf->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::WltcControl){//WLTC曲线
        mySysControl->calConW(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);
    }

    //获得增量控制(开度)
    values[0] = mySysControl->getconBrake();
    values[1] = mySysControl->getconAcc();

    if (Configuration::GetInstance()->ifManualShift)
    {
        // 检查时间是否到了换挡的时刻
        if (changeShiftIndex  < manaulShiftIndexTable.size() && elapsedSeconds >= manaulShiftIndexTable[changeShiftIndex].first)
        {
            SendMotionCmd(values[0], values[1], true, false,
                          manaulShiftIndexTable[changeShiftIndex].second,
                          (int)ShiftClutchUI::ClutchState::Released,
                          (int)ShiftClutchUI::ShiftChangingMode::ManualShift,
                          (int)ShiftClutchUI::ClutchReleasingMode::SlowlyWithControl);
            changeShiftIndex++;
        }
        else
        {
            SendMotionCmd(values[0], values[1]);
        }
    }
    else
    {
        SendMotionCmd(values[0], values[1]);
    }

    //    SendMoveCommandAll(values, ifABS);

    timeStamp++;

    return;
}

double PedalRobot::GetBrakePosition()
{
    return RobotParams::brakeOpenValue;
}

double PedalRobot::GetAcceleratorPosition()
{
    return RobotParams::accOpenValue;
}

double PedalRobot::GetCanCarSpeed()
{
    return RobotParams::canCarSpeed;
}

int PedalRobot::GetPowerMode()
{
    return RobotParams::powerMode;
}

int PedalRobot::GetSysControlMethod()
{
    return mySysControl->GetSysControlMethod();
}

double PedalRobot::GetMP412CarSpeed()
{
    return RobotParams::pulseCarSpeed;
}

double PedalRobot::GetError()
{
    if(!isControlling){ // 未在运行时
        return 0.0;
    }else{
        return GetCurrentTargetSpeed() - GetCarSpeed();
    }
}

void PedalRobot::SaveLoggerFile(const char *filePath)
{
    NormalFile::WriteToFile(filePath, myLogger->data(), myLogger->GetLength());
    PRINTF(LOG_DEBUG, "%s: logger of PC is wtiten into %s\n", __func__, filePath);
}

void PedalRobot::CheckIfSaveLogger()
{
    if(autoSaveLogger){
        autoSaveLogger = false;

        QDate currDate = QDate::currentDate();
        QTime currTime = QTime::currentTime();
        std::string fileName =
                currDate.toString("yyyy_MM_dd").toStdString() + "_"
                + currTime.toString("hh_mm_ss").toStdString();
        SaveLoggerFile( (Configuration::logCurvePath + fileName).c_str() );
        PRINTF(LOG_DEBUG, "%s: save to file=%s\n", __func__, fileName.c_str());
    }
}

bool PedalRobot::ifUnderControlling()
{
    return isControlling;
}

bool PedalRobot::ifControllingOver()
{
    return isControlFinished;
}

void PedalRobot::changeOverState()
{
    isControlFinished = false;
}

void PedalRobot::InitParameters()
{
    vp_data.clear();
    vp_data.reserve(64);
    isControlling=false;
    isControlFinished = false;
    curveFilePath="";
}

int PedalRobot::ReadListeningTeachFile(const std::__cxx11::string &fileName)
{
    std::ifstream fs(fileName.c_str(), std::fstream::binary);
    PRINTF(LOG_DEBUG, "%s: read file = %s\n",__func__, fileName.c_str());
    if(fs.fail()){
        PRINTF(LOG_DEBUG, "%s: fail to open teach file\n",__func__);
        return -1;
    }

    char buf[1024];
    bool firstLine=true;
    while(fs.getline(buf,1024)){
        if(firstLine){
            firstLine=false;
            if(strcmp(buf,RobotParams::robotType.c_str()) != 0){//wrong type
                QMessageBox::information(NULL,"warning",QString("机器人速度曲线文件类型错误!"));
                PRINTF(LOG_ERR, "%s: robot type is wrong\n",__func__);
                return -1;
            }
            continue;
        }

        if(strlen(buf) == 1){
            size_t pointsNum;
            double time, speed;
            int index;

            switch(buf[0]){
            case 'R':
                fs.getline(buf, 1024);//R段的内容对于UI无用
                break;
            case 'S':
                manaulShiftIndexTable.clear();
                fs>>pointsNum;
                manaulShiftIndexTable.reserve(pointsNum);

                for (size_t i = 0; i < pointsNum; ++i)
                {
                     fs>>time>>index;
                     manaulShiftIndexTable.push_back( std::make_pair(time, index) );
                }
                break;
            case 'M':
                vp_data.clear();
                fs>>pointsNum;
                vp_data.reserve(pointsNum);

                while(!fs.eof()){
                    fs>>time>>speed;
                    vp_data.push_back( std::make_pair(time,speed) );
                }
                break;
            default:
                PRINTF(LOG_ERR, "%s: (%s) switch default return -1\n",__func__,buf);
                return -1;
            }
        }else{
            QString s(buf);
            if(s.length() != s.count(' ')){//空白行不计
                PRINTF(LOG_ERR, "%s: else return -1\n",__func__);
                PRINTF(LOG_ERR, "%s: unknown contens=%s\n",__func__,buf);
                return -1;
            }
        }
    }
    fs.close();
    return 0;
}

void PedalRobot::CheckTimerAccuracy()
{
    //定时器的准确性
    static timeval tv1;
    static bool flag=false;
    if(!flag){
        gettimeofday(&tv1,NULL);
        flag=true;
    }

    static timeval tv2;
    gettimeofday(&tv2,NULL);
    PRINTF(LOG_DEBUG, "%s: elapsedtime %f.\n", __func__, GetElapsedMillisecond(tv1,tv2));
    tv1=tv2;
}

double PedalRobot::CalculateUpperLowerBound(QVector<double>& time, QVector<double>& speed,
                                            QVector<double>& upperTime, QVector<double>& upperSpeed,
                                            QVector<double>& lowerTime, QVector<double>& lowerSpeed)
{
    // 计算误差上下界曲线的数据
    double maxY = 0.0;
    const size_t sz = vp_data.size();
    time.clear(); time.resize(sz); speed.clear(); speed.resize(sz);
    upperTime.clear(); upperTime.resize(sz); upperSpeed.clear(); upperSpeed.resize(sz);
    lowerTime.clear(); lowerTime.resize(sz); lowerSpeed.clear(); lowerSpeed.resize(sz);

    // vp_data生成时间time-速度speed曲线
    for(size_t i=0; i<sz; ++i){
        time[i] = vp_data[i].first;
        speed[i]= vp_data[i].second;
        maxY = std::max(maxY, speed[i]+10.0);
    }

    // 构造速度误差的上下界
    for(size_t i=0; i<sz; ++i){
        upperSpeed[i] = speed[i] + 2.0;
        lowerSpeed[i] = speed[i] - 2.0;
    }

    if(conf->calcSpeedErrorMethod == 0){//正负2km/h 没有时间偏移
        upperTime = time;
        lowerTime = time;
    }else if(conf->calcSpeedErrorMethod == 1){//正负2km/h 时间偏移1s
        for(size_t i=0; i<sz; ++i){
            int accBefore, accAfter;
            CalculateSgnOfAcc(i, accBefore, accAfter);

            switch(conf->pedalRobotUsage){
            case SettingWidgetPedalRobotGetSpeed::NedcControl:
                if(accBefore==1 || accAfter==1){//加速段
                    upperTime[i] = time[i] - 1.0;
                    lowerTime[i] = time[i] + 1.0;
                }else if(accBefore==-1 || accAfter==-1){//减速段
                    upperTime[i] = time[i] + 1.0;
                    lowerTime[i] = time[i] - 1.0;
                }else{//匀速段
                    upperTime[i] = time[i];
                    lowerTime[i] = time[i];
                }
                break;
            case SettingWidgetPedalRobotGetSpeed::WltcControl:
                if(accBefore==1 && accAfter==1){//加速段
                    upperTime[i] = time[i] - 1.0;
                    lowerTime[i] = time[i] + 1.0;
                }else if(accBefore==-1 && accAfter==-1){//减速段
                    upperTime[i] = time[i] + 1.0;
                    lowerTime[i] = time[i] - 1.0;
                }else{//匀速段 加减速段 减加速段
                    upperTime[i] = time[i];
                    lowerTime[i] = time[i];
                }
                break;
            default:
                PRINTF(LOG_ERR, "%s: unvalid pedalRobotUsage(%d)\n", __func__, conf->pedalRobotUsage);
                Q_ASSERT(false);
                break;
            }
        }
    }else{
        Q_ASSERT(false);
    }

    CalibrateBoundSpeed(upperTime, upperSpeed, 1);
    ProcessSpeedWithSameTime(upperTime, upperSpeed, 1);
    CalibrateBoundSpeed(lowerTime, lowerSpeed, -1);
    ProcessSpeedWithSameTime(lowerTime, lowerSpeed, -1);

    return maxY;
}

void PedalRobot::CalculateSgnOfAcc(size_t index, int &accBefore, int &accAfter)
{
    // 目标曲线上的加速度的正负
    if(index==0 || index==vp_data.size()-1){ // 起始和结束点点 必定为静止 sgn=0
        accBefore = 0;
        accAfter = 0;
        return;
    }
    if(index >= vp_data.size()){ // 越界
        accBefore = 0;
        accAfter = 0;
        return;
    }

    double deltaSpeed = vp_data[index].second - vp_data[index-1].second;
    if(deltaSpeed > 0.0){
        accBefore = 1;
    }else if(deltaSpeed < 0.0){
        accBefore = -1;
    }else{
        accBefore = 0;
    }

    // 向前看的加速度
    deltaSpeed = vp_data[index+1].second - vp_data[index].second;
    if(deltaSpeed > 0.0 ){
        accAfter = 1;
    }else if(deltaSpeed < 0.0){
        accAfter = -1;
    }else{
        accAfter = 0;
    }
}

void PedalRobot::CalibrateBoundSpeed(const QVector<double> &boundTime, QVector<double> &boundSpeed, const int lowerUpperBound)
{
    if(conf->pedalRobotUsage != SettingWidgetPedalRobotGetSpeed::WltcControl){
        // 非WLTC不处理
        return;
    }

    // lowerUpperBound速度界=上界1/下界-1
    // 时间偏移出来的速度上下界 可能不满足最小+-2km/h
    const double elapsedSecondsBack = elapsedSeconds;//备份
    for(int i=0; i<boundTime.size(); ++i){
        elapsedSeconds = boundTime[i];
        const double targetSpeed = GetCurrentTargetSpeed();
        if(lowerUpperBound == 1){//上界 取大
            boundSpeed[i] = std::max(boundSpeed[i], targetSpeed+2.0);
        }else{//下界 取小
            boundSpeed[i] = std::min(boundSpeed[i], targetSpeed-2.0);
        }
    }
    elapsedSeconds = elapsedSecondsBack;//恢复备份
}

void PedalRobot::ProcessSpeedWithSameTime(const QVector<double> &boundTime, QVector<double> &boundSpeed, const int lowerUpperBound)
{
    if(conf->pedalRobotUsage != SettingWidgetPedalRobotGetSpeed::WltcControl){
        //非WLTC不处理
        return;
    }

    // lowerUpperBound速度界=上界1/下界-1
    // 处理WLTC出现的 时间偏移后,在同一个时间点有多个速度的情况
    const size_t sz = boundTime.size();
    for(size_t i=0; i<sz; ++i){
        if((i+1<sz) && (boundTime[i] == boundTime[i+1])){//检查重复的时间点
            //i~j-1的点 时间相同
            size_t j = i+1;
            while(j<sz && (boundTime[j] == boundTime[i])){
                ++j;
            }

            //i~j-1的点 速度最值
            double minMax = boundSpeed[i];
            for(size_t index=i+1; index<j; ++index){
                if(lowerUpperBound == -1){//下界 取小
                    minMax = std::min(minMax, boundSpeed[index]);
                }else{//上界 取大
                    minMax = std::max(minMax, boundSpeed[index]);
                }
            }
            //i~j-1的点 速度赋为最值
            for(size_t index=i; index<j; ++index){
                boundSpeed[index] = minMax;
            }
            //调整i
            i = j-1;
        }else if((i+2<sz) && (boundTime[i] == boundTime[i+2])){//同时检查i+2点(可能出现time=156 157 156 157的顺序)
            double minMax;
            if(lowerUpperBound == -1){//下界 取小
                minMax = std::min(boundSpeed[i], boundSpeed[i+2]);
            }else{//上界 取大
                minMax = std::max(boundSpeed[i], boundSpeed[i+2]);
            }
            boundSpeed[i] = minMax;
            boundSpeed[i+2] = minMax;
        }
    }
}

void PedalRobot::InitQCustomPlot(double maxY)
{
    pQCustomPlot->clearGraphs();
    pQCustomPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    pQCustomPlot->xAxis->setLabel(QObject::tr("时间(s)"));
    pQCustomPlot->yAxis->setLabel(QObject::tr("速度(km/h)"));

    QPen myPen;
    myPen.setWidth(3);
    //0)目标曲线
    pQCustomPlot->addGraph();
    myPen.setColor(Qt::red);
    pQCustomPlot->graph(fileCurve)->setPen(myPen);
    //1)实际曲线
    pQCustomPlot->addGraph();
    myPen.setColor(Qt::green);
    pQCustomPlot->graph(actionCurve)->setPen(myPen);

    //2)上界
    pQCustomPlot->addGraph();
    myPen.setWidth(1);
    myPen.setColor(Qt::blue);
    pQCustomPlot->graph(upperCurve)->setPen(myPen);
    //3)下界
    pQCustomPlot->addGraph();
    pQCustomPlot->graph(lowerCurve)->setPen(myPen);

    pQCustomPlot->yAxis->setRange(minY, maxY);
}

void PedalRobot::UpdateQCustomPlot()
{
    if(!isControlling){
        return;
    }

    struct timeval tv;
    gettimeofday(&tv,NULL);
    elapsedSeconds = GetElapsedMillisecond(actionStartTime, tv)/1000.0 + conf->pedalStartTimeS; // 加上调试开始时间的偏移量

    pQCustomPlot->graph(actionCurve)->addData(elapsedSeconds, GetCarSpeed());//(s, km/h)
    double lowerX = GetDisplayLowerBound(elapsedSeconds);
    pQCustomPlot->xAxis->setRange(lowerX, lowerX+30.0);
    pQCustomPlot->replot();

    if(elapsedSeconds <= actionDurationSecond){
        return;
    }else{
        PRINTF(LOG_DEBUG, "%s: finish this action!. elapsedSeconds/duration=%f/%f\n",
               __func__, elapsedSeconds, actionDurationSecond);
        FinishQCustomPlot(false);
    }
}

void PedalRobot::FinishQCustomPlot(bool showMessageBox)
{
    isControlling=false;
    isControlFinished = true;
    CheckIfSaveLogger();

    if(showMessageBox){
        QMessageBox::information(NULL,"提示","自动运行曲线运行已完成!请熄火车辆!");
    }
}

double PedalRobot::GetElapsedMillisecond(struct timeval tvStart, struct timeval tvEnd)//ms
{
    return (tvEnd.tv_sec-tvStart.tv_sec)*1000.0 + (tvEnd.tv_usec-tvStart.tv_usec)/1000.0;
}

double PedalRobot::GetDisplayLowerBound(double value)
{
    return value<15.0? 0.0: value-15.0;
}

double PedalRobot::GetDisplayLowerBoundSpeed(double value)
{
    return value<30.0? 0.0: value-30.0;
}

double PedalRobot::GetDisplayUpperBound(double value)
{
    return value>actionDurationSecond-30.0? actionDurationSecond: value+30.0;
}

double PedalRobot::GetCarSpeed()
{
    switch (conf->getSpeedMethod){
    case SettingWidgetPedalRobotGetSpeed::PulseFrequency2Speed:
        return RobotParams::pulseCarSpeed;
        break;
    case SettingWidgetPedalRobotGetSpeed::CANSpeed:
        return RobotParams::canCarSpeed;
        break;
    default:
        PRINTF(LOG_ERR, "%s: undefined getSpeedMethod(%d)\n", __func__, conf->getSpeedMethod);
        break;
    }
    return -1.0;
}

void PedalRobot::LoggerStudySamples()
{
    myLogger->Customize(timeStamp).Customize(" ");
    myLogger->Customize(RobotParams::brakeOpenValue).Customize(" ");
    myLogger->Customize(RobotParams::accOpenValue).Customize(" ");
    myLogger->Customize(RobotParams::canCarSpeed).Customize(" ");
    myLogger->Customize(RobotParams::pulseCarSpeed).Customize(" ");
    myLogger->Customize(GetCurrentTargetSpeed()).Customize(" ");
    myLogger->Customize(mySysControl->GetSysControlMethod()).Customize(" ");
    myLogger->Customize(mySysControl->getconBrake()).Customize(" ");
    myLogger->Customize(mySysControl->getconAcc()).Customize(" ");
    myLogger->Customize(RobotParams::angleRealTime[0]).Customize(" ");
    myLogger->Customize(RobotParams::angleRealTime[1]).Customize(" ");
    myLogger->Customize(QTime::currentTime().toString("hhmmsszzz").toStdString().c_str()).Customize(" ");
    myLogger->Customize((int)RobotParams::shiftIndex).Customize(" ");
    myLogger->Customize((int)RobotParams::clutchIndex).Customize(" ");
    myLogger->Customize(RobotParams::angleRealTime[2]).Customize(" ");
    myLogger->Customize(RobotParams::angleRealTime[3]).Customize(" ");
    myLogger->Customize(RobotParams::angleRealTime[4]).Customize("\n");
}

double PedalRobot::GetCurrentTargetSpeed()
{
    for(size_t i=1; i<vp_data.size(); ++i){//从下标1开始
        double time = vp_data[i].first;
        if(elapsedSeconds <= time){//当前在i-1~i段中
            double k = (vp_data[i].second - vp_data[i-1].second) / (vp_data[i].first - vp_data[i-1].first);
            double timeLeft = time - elapsedSeconds;
            return vp_data[i].second - k*timeLeft;
        }
    }
    return 0.0;
}

void PedalRobot::InitControlMethod()
{
    mySysControl->Init(vp_data, conf, vp_data_upperbound, vp_data_lowerbound);
}

void PedalRobot::SendMoveCommandAll(double *values, int *ifABS)
{
    std::vector<int> actionMethod;
    std::vector<int> actionAxes;
    std::vector<double> actionTheta;

    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        actionAxes.push_back(i);
        actionTheta.push_back(values[i]);

        if (ifABS[i] == 1)
        {
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
        }
        else
        {
            actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
        }
    }

    AutoDriveRobotApiClient::GetInstance()->Send_SetMonitorActionThetaMsg(actionMethod, actionAxes, actionTheta);
}

void PedalRobot::SendMotionCmd(const double deltabrk, const double deltaacc, const bool ifshiftmode, const bool ifpause, const int aimshift, const int aimclutch, const int howtochangeshift, const int howtoreleaseclutch)
{
    QVector<double> values(6, 0.0);
    QVector<int> cmdstate(6, 0);

    if (ifshiftmode)
    {
        int stateflag = 0;
        stateflag += ifshiftmode ? 1 : 0;
        stateflag += (ifpause ? 1 : 0) * 10;
        stateflag += howtoreleaseclutch * 100;
        stateflag += howtochangeshift * 1000;

        values[0] = deltabrk;
        values[1] = deltaacc;

        cmdstate[2] = aimclutch;
        cmdstate[3] = aimshift;

        SendMoveCommandAll(values, cmdstate, stateflag);
    }
    else
    {
        values[0] = deltabrk;
        values[1] = deltaacc;
        SendMoveCommandAll(values, cmdstate);
    }
}

void PedalRobot::SendMoveCommandAll(QVector<double> values, QVector<int> cmdstate, const int customvariable)
{
    std::vector<int> actionMethod;
    std::vector<int> actionAxes;
    std::vector<double> actionTheta;

    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        actionAxes.push_back(i);
        actionTheta.push_back(values[i]);

        if (customvariable == 0)
        {
            if (cmdstate[i] == 1)
            {
                actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            }
            else
            {
                actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
            }
        }
        else
        {
            actionMethod.push_back(cmdstate[i]);
        }
    }

    AutoDriveRobotApiClient::GetInstance()->Send_SetMonitorActionThetaMsg(actionMethod, actionAxes, actionTheta, customvariable);
}
