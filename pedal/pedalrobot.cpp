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

PedalRobot::PedalRobot(QCustomPlot *_widget, Configuration *_conf, QCustomPlot *_widgetnvh1, QCustomPlot *_widgetnvh2)
    :pQCustomPlot(_widget), conf(_conf), plotNVH1(_widgetnvh1), plotNVH2(_widgetnvh2)
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
    RobotParams::switchflag[0] = false;
    RobotParams::switchflag[1] = false;
    RobotParams::switchflag[2] = false;
    RobotParams::switchflag[3] = false;
    RobotParams::switchflag[4] = false;
    RobotParams::isExaming = false;
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

    //2) 提示发动车辆 准备开始
    if (conf->ifManualShift)
    {
        RobotParams::currentclutchindex = 1;
        RobotParams::currentclutchvalue = "松开";
        RobotParams::currentshiftindex = 1;
        RobotParams::currentshiftvalue = RobotParams::manulShiftValues[RobotParams::currentshiftindex];
        RobotParams::aimclutchindex = 0;
    }
    else
    {
        RobotParams::currentclutchindex = 1;
        RobotParams::currentclutchvalue = "自动";
        RobotParams::currentshiftindex = 1;
        RobotParams::currentshiftvalue = RobotParams::autoShiftValues[RobotParams::currentshiftindex];
        RobotParams::aimshiftindex = 2;
    }

    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

    // 开始监听模式
    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::examFilePath + "SC_ARM");
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return false;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

    RobotParams::round = 1;
    RobotParams::round2 = 1;
    RobotParams::switchflag[1] = false;
    RobotParams::switchflag[0] = true;

    ret = QMessageBox::information(
                NULL,"提示", QString(curveFilePath.c_str()) + QObject::tr("请再次确认挡位离合信息后发动车辆!\n点击确认后开始运行!"),
                QObject::tr("确认"),QObject::tr("取消"));

    while (RobotParams::switchflag[0]) {;}

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
        PRINTF(LOG_DEBUG, "%sunknown choice...cancel\n", __func__);
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

    if (conf->ifManualShift)
    {
        // 确定初始换挡查询点
        size_t listlen = RobotParams::changeshiftlist.size();

        if (conf->pedalStartTimeS < RobotParams::changeshiftlist[0].first)
        {
            RobotParams::checkshiftlist = 0;
        }
        else
        {
            for (size_t i=0; i<listlen - 1; ++i)
            {
                if (conf->pedalStartTimeS > RobotParams::changeshiftlist[i].first && conf->pedalStartTimeS < RobotParams::changeshiftlist[i+1].first)
                {
                    RobotParams::checkshiftlist = i+1;
                    break;
                }
            }
        }
    }

    // 初始化换挡轮数
    RobotParams::round = 1;
    RobotParams::round2 = 1;
    RobotParams::changeshiftprocess = 0;
    RobotParams::startchangeshifttimeflag = false;
    RobotParams::changeshiftstart = false;
    RobotParams::changeshiftend = false;

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
    myLogger->Customize("TimeStamp\tBrakeOpenValue\tAccOpenValue\tCANSpeed\t\tPULSESpeed\t\tTargetSpeed\t\tControlMethod\tConBrake\t\tConAcc\t\t\tShift\t\t\tClutch\t\t\tBrakeAxis\t\tAccAxis\t\t\tClutchAxis\t\tShiftAxis1\t\tShiftAxis2\t\tSteeringAxis\tCurrentTime\n");
    autoSaveLogger = true;
    PRINTF(LOG_DEBUG, "%s: logger is cleared and ready!\n", __func__);

    timeStamp=0;
    isControlling=true;
}

void PedalRobot::UpdatePart1()
{
//    CheckTimerAccuracy(); // 校验时间差

    double values[RobotParams::axisNum];
    int ifABS[RobotParams::axisNum];
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        values[i] = 0; ifABS[i] = 0;
    }

    if (RobotParams::switchflag[0])
    {
        if (!RobotParams::switchflag[1])
        {
            if (RobotParams::round2 > 5)
            {
                RobotParams::switchflag[1] = true;

                if (conf->ifManualShift)
                {
                    if (mySysControl->ifreachedclutch(false, 0) && fabs(RobotParams::angleRealTime[0] - conf->brakeThetaAfterGoHome) < 1.0)
                    {
                        RobotParams::currentclutchindex = 0;
                        RobotParams::currentclutchvalue = "踩下";

                        RobotParams::round = 1;
                        RobotParams::round2 = 1;
                        RobotParams::switchflag[1] = false;
                        RobotParams::switchflag[0] = false;

                        AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
                    }
                    else
                    {
                        double *clutchaim = new double();
                        mySysControl->getconClh(clutchaim, RobotParams::round);

                        values[0] = conf->brakeThetaAfterGoHome;
                        values[2] = *clutchaim;
                        ifABS[0] = 1;
                        ifABS[2] = 1;

                        SendMoveCommandAll(values, ifABS);
                        delete clutchaim;

                        RobotParams::round++;
                    }
                }
                else
                {
                    if (mySysControl->ifreachedshift(false, 2) && fabs(RobotParams::angleRealTime[0] - conf->brakeThetaAfterGoHome) < 1.0)
                    {
                        RobotParams::currentshiftindex = 2;
                        RobotParams::currentshiftvalue = "D";

                        RobotParams::round = 1;
                        RobotParams::round2 = 1;
                        RobotParams::switchflag[1] = false;
                        RobotParams::switchflag[0] = false;

                        AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
                    }
                    else
                    {
                        double *shiftaims = new double[2];
                        mySysControl->getconSft(shiftaims, RobotParams::round);

                        values[0] = conf->brakeThetaAfterGoHome;
                        values[3] = *shiftaims;
                        values[4] = *(shiftaims + 1);
                        ifABS[0] = 1;/*
                        ifABS[3] = 1;*/
                        ifABS[4] = 1;

                        SendMoveCommandAll(values, ifABS);
                        delete shiftaims;

                        RobotParams::round++;
                    }
                }
            }
            else
            {
                RobotParams::round2++;
                SendMoveCommandAll(values, ifABS);
            }
        }
        else
        {
            if (conf->ifManualShift)
            {
                if (mySysControl->ifreachedclutch(false, 0) && fabs(RobotParams::angleRealTime[0] - conf->brakeThetaAfterGoHome) < 1.0)
                {
                    RobotParams::currentclutchindex = 0;
                    RobotParams::currentclutchvalue = "踩下";

                    RobotParams::round = 1;
                    RobotParams::round2 = 1;
                    RobotParams::switchflag[1] = false;
                    RobotParams::switchflag[0] = false;

                    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
                }
                else
                {
                    double *clutchaim = new double();
                    mySysControl->getconClh(clutchaim, RobotParams::round);

                    values[0] = conf->brakeThetaAfterGoHome;
                    values[2] = *clutchaim;
                    ifABS[0] = 1;
                    ifABS[2] = 1;

                    SendMoveCommandAll(values, ifABS);
                    delete clutchaim;

                    RobotParams::round++;
                }
            }
            else
            {
                if (mySysControl->ifreachedshift(false, 2) && fabs(RobotParams::angleRealTime[0] - conf->brakeThetaAfterGoHome) < 1.0)
                {
                    RobotParams::currentshiftindex = 2;
                    RobotParams::currentshiftvalue = "D";

                    RobotParams::round = 1;
                    RobotParams::round2 = 1;
                    RobotParams::switchflag[1] = false;
                    RobotParams::switchflag[0] = false;

                    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
                }
                else
                {
                    double *shiftaims = new double[2];
                    mySysControl->getconSft(shiftaims, RobotParams::round);

                    values[0] = conf->brakeThetaAfterGoHome;
                    values[3] = *shiftaims;
                    values[4] = *(shiftaims + 1);
                    ifABS[0] = 1;
                    ifABS[3] = 1;
                    ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);
                    delete shiftaims;

                    RobotParams::round++;
                }
            }
        }
    }

    return;
}

void PedalRobot::UpdatePart2()
{
    if (!isControlling || RobotParams::switchflag[0] || RobotParams::switchflag[1] || RobotParams::switchflag[2] || RobotParams::switchflag[3] || RobotParams::switchflag[4] || RobotParams::isExaming)
    {
        return;
    }

    // 绘图
    UpdateQCustomPlot();

    double values[RobotParams::axisNum];
    int ifABS[RobotParams::axisNum];
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        values[i] = 0; ifABS[i] = 0;
    }

    // 换挡
    if (conf->ifManualShift)
    {
        int end_index = RobotParams::changeshiftlist.size()-1;
        if (elapsedSeconds >= RobotParams::changeshiftlist[RobotParams::checkshiftlist].first && RobotParams::checkshiftlist <= end_index)
        {
            // 换挡点到了
            RobotParams::checkshiftlist++;
            RobotParams::changeshiftstart = true;
            RobotParams::changeshiftend = false;
            RobotParams::round = 1;
            RobotParams::round2 = 1;
            RobotParams::changeshiftprocess = 0;
            RobotParams::startchangeshifttimeflag = false;

            if (!RobotParams::startchangeshifttimeflag)
            {
                RobotParams::startchangeshifttimeflag = true;
                gettimeofday(&RobotParams::starttime, NULL);
            }

            values[0] = Configuration::GetInstance()->deathPos[0];
            values[1] = Configuration::GetInstance()->deathPos[1];

            RobotParams::aimclutchindex = 0;
            double *clutchaim = new double();
            mySysControl->getconClh(clutchaim, RobotParams::round);
            values[2] = *clutchaim;
            delete clutchaim;
            RobotParams::round++;

            ifABS[0] = 1; ifABS[1] = 1; ifABS[2] = 1;

            PRINTF(LOG_INFO, "%s: ready to change shift.\n", __func__);
            SendMoveCommandAll(values, ifABS);

            LoggerStudySamples();
            timeStamp++;
            return;
        }

        if (RobotParams::changeshiftstart)
        {
            if (RobotParams::changeshiftprocess == 0)
            {
                if (mySysControl->ifreachedclutch(false, 0) && fabs(RobotParams::angleRealTime[0]-Configuration::GetInstance()->deathPos[0]) < 1.0 && fabs(RobotParams::angleRealTime[1]-Configuration::GetInstance()->deathPos[1]) < 1.0)
                {
                    gettimeofday(&RobotParams::stoptime, NULL);
                    double timeduring = (RobotParams::stoptime.tv_sec-RobotParams::starttime.tv_sec)*1000.0 + (RobotParams::stoptime.tv_usec-RobotParams::starttime.tv_usec)/1000.0;
                    PRINTF(LOG_INFO, "%s: clutch has been trodden, using time %f ms.\n", __func__, timeduring);

                    RobotParams::currentclutchindex = 0;
                    RobotParams::currentclutchvalue = "踩下";

                    RobotParams::round = 1;
                    RobotParams::changeshiftprocess = 1;

                    // 规划路径
                    RobotParams::aimshiftindex = RobotParams::changeshiftlist[RobotParams::checkshiftlist - 1].second;
                    mySysControl->plantrace();
                    std::string indexaim = RobotParams::manulShiftValues[RobotParams::aimshiftindex];
                    PRINTF(LOG_INFO, "%s: to shift %s.\n", __func__, indexaim.c_str());

                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = RobotParams::manulShiftValues[RobotParams::currentshiftindex];
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySysControl->getconSft(shiftaims, RobotParams::round);
                    values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                    delete shiftaims;
                    RobotParams::round++;

                    ifABS[3] = 1; ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);

                    LoggerStudySamples();
                    timeStamp++;
                    return;
                }
                else
                {
                    values[0] = Configuration::GetInstance()->deathPos[0];
                    values[1] = Configuration::GetInstance()->deathPos[1];

                    RobotParams::aimclutchindex = 0;
                    double *clutchaim = new double();
                    mySysControl->getconClh(clutchaim, RobotParams::round);
                    values[2] = *clutchaim;
                    delete clutchaim;
                    RobotParams::round++;

                    ifABS[0] = 1; ifABS[1] = 1; ifABS[2] = 1;

                    SendMoveCommandAll(values, ifABS);

                    LoggerStudySamples();
                    timeStamp++;
                    return;
                }
            }
            else if (RobotParams::changeshiftprocess == 1)
            {
                if (RobotParams::shiftrunpointer + 1 == RobotParams::shiftrunlength - 1)
                {
                    if (mySysControl->ifreachedshift(false,RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                    {
                        RobotParams::shiftrunpointer++;

                        RobotParams::round = 1;

                        gettimeofday(&RobotParams::stoptime, NULL);
                        double timeduring = (RobotParams::stoptime.tv_sec-RobotParams::starttime.tv_sec)*1000.0 + (RobotParams::stoptime.tv_usec-RobotParams::starttime.tv_usec)/1000.0;
                        PRINTF(LOG_INFO, "%s: shift changing is finished, using time %f ms.\n", __func__, timeduring);

                        RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                        RobotParams::currentshiftvalue = RobotParams::manulShiftValues[RobotParams::currentshiftindex];
                        RobotParams::lastshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer - 1];
                        RobotParams::shiftrunpointer = 0;

                        RobotParams::changeshiftprocess = 0;
                        RobotParams::changeshiftstart = false;
                        RobotParams::changeshiftend = true;

                        SendMoveCommandAll(values, ifABS);

                        LoggerStudySamples();
                        timeStamp++;
                        return;
                    }
                    else
                    {
                        RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                        RobotParams::currentshiftvalue = RobotParams::manulShiftValues[RobotParams::currentshiftindex];
                        RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                        double *shiftaims = new double[2];
                        mySysControl->getconSft(shiftaims, RobotParams::round);
                        values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                        delete shiftaims;
                        RobotParams::round++;

                        ifABS[3] = 1; ifABS[4] = 1;

                        SendMoveCommandAll(values, ifABS);

                        LoggerStudySamples();
                        timeStamp++;
                        return;
                    }
                }
                else
                {
                    if (mySysControl->ifreachedshiftprocess(RobotParams::shiftrunpath[RobotParams::shiftrunpointer], RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                    {
                        RobotParams::shiftrunpointer++;

                        RobotParams::round = 1;

                        gettimeofday(&RobotParams::stoptime, NULL);
                        double timeduring = (RobotParams::stoptime.tv_sec-RobotParams::starttime.tv_sec)*1000.0 + (RobotParams::stoptime.tv_usec-RobotParams::starttime.tv_usec)/1000.0;
                        PRINTF(LOG_INFO, "%s: shift is changing, using time %f ms.\n", __func__, timeduring);

                        RobotParams::lastshiftindex = RobotParams::currentshiftindex;
                        RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                        RobotParams::currentshiftvalue = RobotParams::manulShiftValues[RobotParams::currentshiftindex];
                        RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                        double *shiftaims = new double[2];
                        mySysControl->getconSft(shiftaims, RobotParams::round);
                        values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                        delete shiftaims;
                        RobotParams::round++;

                        ifABS[3] = 1; ifABS[4] = 1;

                        SendMoveCommandAll(values, ifABS);

                        LoggerStudySamples();
                        timeStamp++;
                        return;
                    }
                    else
                    {
                        RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                        RobotParams::currentshiftvalue = RobotParams::manulShiftValues[RobotParams::currentshiftindex];
                        RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                        double *shiftaims = new double[2];
                        mySysControl->getconSft(shiftaims, RobotParams::round);
                        values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                        delete shiftaims;
                        RobotParams::round++;

                        ifABS[3] = 1; ifABS[4] = 1;

                        SendMoveCommandAll(values, ifABS);

                        LoggerStudySamples();
                        timeStamp++;
                        return;
                    }
                }
            }
        }
    }

    // 非换挡情况下 执行油门刹车踏板计算
    double deltaBrake, deltaAcc;

    if(conf->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::NedcControl){//NEDC曲线
        mySysControl->calCon(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);
    }else if(conf->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::WltcControl){//WLTC曲线
        mySysControl->calConW(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);
    }

    //获得增量控制(开度)
    deltaBrake = mySysControl->getconBrake();
    deltaAcc   = mySysControl->getconAcc();

    // 软件限位
    bool overmax[2] = {false, false};
    bool overmin[2] = {false, false};
    if ((RobotParams::angleRealTime[0] + deltaBrake) > conf->limPos[0])
    {
        overmax[0] = true;
    }
    else if ((RobotParams::angleRealTime[0] + deltaBrake) < conf->deathPos[0])
    {
        overmin[0] = true;
    }
    else { }

    if ((RobotParams::angleRealTime[1] + deltaAcc) > conf->limPos[1])
    {
        overmax[1] = true;
    }
    else if ((RobotParams::angleRealTime[1] + deltaAcc) < conf->deathPos[1])
    {
        overmin[1] = true;
    }
    else { }

    // 换挡结束时 刹车油门和离合的配合
    if (conf->ifManualShift)
    {
        if (RobotParams::changeshiftend)
        {
            if (RobotParams::currentshiftindex != 3)
            {
                if (mySysControl->ifreachedclutch(false, 1))
                {
                    gettimeofday(&RobotParams::stoptime, NULL);
                    double timeduring = (RobotParams::stoptime.tv_sec-RobotParams::starttime.tv_sec)*1000.0 + (RobotParams::stoptime.tv_usec-RobotParams::starttime.tv_usec)/1000.0;
                    PRINTF(LOG_INFO, "%s: changing shift is finished, totally using time %f ms.\n", __func__, timeduring);

                    RobotParams::currentclutchindex = 1;
                    RobotParams::currentclutchvalue = "松开";

                    RobotParams::startchangeshifttimeflag = false;
                    RobotParams::changeshiftend = false;

                    RobotParams::round = 1;

                    SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, false);
                }
                else
                {
                    RobotParams::currentclutchindex = 2;
                    RobotParams::currentclutchvalue = "抬升";

                    double clutchaim = 0;
                    if ((clutchaim = Configuration::GetInstance()->clutchAngles[0] - RobotParams::round * Configuration::GetInstance()->clutchUpSpeed) < Configuration::GetInstance()->clutchAngles[1])
                    {
                        clutchaim = Configuration::GetInstance()->clutchAngles[1];
                    }

                    values[2] = clutchaim;
                    RobotParams::round++;

                    ifABS[2] = 1;

                    SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, true, values[2]);
                }
            }
            else
            {
                if (mySysControl->ifreachedclutch(false, 1) && fabs(RobotParams::angleRealTime[1] - Configuration::GetInstance()->startAccAngleValue) < 1.0)
                {
                    gettimeofday(&RobotParams::stoptime, NULL);
                    double timeduring = (RobotParams::stoptime.tv_sec-RobotParams::starttime.tv_sec)*1000.0 + (RobotParams::stoptime.tv_usec-RobotParams::starttime.tv_usec)/1000.0;
                    PRINTF(LOG_INFO, "%s: changing shift is finished, totally using time %f ms.\n", __func__, timeduring);

                    RobotParams::currentclutchindex = 1;
                    RobotParams::currentclutchvalue = "松开";

                    RobotParams::startchangeshifttimeflag = false;
                    RobotParams::changeshiftend = false;

                    RobotParams::round = 1;

                    SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, false);
                }
                else
                {
                    RobotParams::currentclutchindex = 2;
                    RobotParams::currentclutchvalue = "抬升";

                    double clutchaim = 0, accaim = 0;
                    if ((clutchaim = Configuration::GetInstance()->clutchAngles[0] - RobotParams::round * Configuration::GetInstance()->clutchUpSpeed) < Configuration::GetInstance()->clutchAngles[1])
                    {
                        clutchaim = Configuration::GetInstance()->clutchAngles[1];
                    }
                    values[2] = clutchaim;

                    if ( ( accaim = Configuration::GetInstance()->deathPos[1] + RobotParams::round * (Configuration::GetInstance()->clutchUpSpeed / (Configuration::GetInstance()->clutchAngles[0] - Configuration::GetInstance()->clutchAngles[1]) * (Configuration::GetInstance()->startAccAngleValue - Configuration::GetInstance()->deathPos[1]) ) ) > Configuration::GetInstance()->startAccAngleValue)
                    {
                        accaim = Configuration::GetInstance()->startAccAngleValue;
                    }
                    values[1] = accaim;
                    ifABS[1] = 1;
                    ifABS[2] = 1;

                    RobotParams::round++;

                    SendMoveCommandAll(values, ifABS);
                }
            }
        }
        else
        {
            SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, false);
        }
    }
    else
    {
        // 自动挡结束前回空挡
        if ((elapsedSeconds > actionDurationSecond - 2) && (GetCarSpeed() < 10))
        {
            if (mySysControl->ifreachedshift(false, 1))
            {
                RobotParams::round = 1;

                RobotParams::currentshiftindex = 1;
                RobotParams::currentshiftvalue = "N";
                RobotParams::currentclutchindex = 1;
                RobotParams::currentclutchvalue = "自动";

                SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, false);
            }
            else
            {
                RobotParams::aimshiftindex = 1;

                double *shiftaims = new double[2];
                mySysControl->getconSft(shiftaims, RobotParams::round);

                values[3] = *shiftaims;
                values[4] = *(shiftaims + 1);
                ifABS[3] = 1;
                ifABS[4] = 1;

                SendMoveCommandAll(values, ifABS);

                delete shiftaims;

                RobotParams::round++;
            }
        }
        else
        {
            SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, false);
        }
    }

    LoggerStudySamples();
    timeStamp++;

    return;
}

void PedalRobot::UpdatePart3()
{
    if (isControlling || RobotParams::switchflag[0] || RobotParams::switchflag[1] || RobotParams::switchflag[3] || RobotParams::switchflag[4] || RobotParams::isExaming)
    {
        return;
    }

    // 是否在NVH曲线运行下
    if (RobotParams::switchflag[2])
    {
        if (RobotParams::NVHcurvestate == 9)
        {
            return;
        }
        else if (RobotParams::NVHcurvestate == 0)
        {
            // 绘图
            UpdateQCustomPlotNVH();

            double values[RobotParams::axisNum];
            int ifABS[RobotParams::axisNum];
            for (unsigned int i=0; i<RobotParams::axisNum; ++i)
            {
                values[i] = 0; ifABS[i] = 0;
            }

            // 非换挡情况下 执行油门刹车踏板计算
            double deltaBrake, deltaAcc;

            mySysControl->calConW(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);

            //获得增量控制(开度)
            deltaBrake = mySysControl->getconBrake();
            deltaAcc   = mySysControl->getconAcc();

            // 软件限位
            bool overmax[2] = {false, false};
            bool overmin[2] = {false, false};
            if ((RobotParams::angleRealTime[0] + deltaBrake) > conf->limPos[0])
            {
                overmax[0] = true;
            }
            else if ((RobotParams::angleRealTime[0] + deltaBrake) < conf->deathPos[0])
            {
                overmin[0] = true;
            }
            else { }

            if ((RobotParams::angleRealTime[1] + deltaAcc) > conf->limPos[1])
            {
                overmax[1] = true;
            }
            else if ((RobotParams::angleRealTime[1] + deltaAcc) < conf->deathPos[1])
            {
                overmin[1] = true;
            }
            else { }

            // 换挡结束时 刹车油门和离合的配合
            // 自动挡结束前送油门
            if ( (elapsedSeconds > actionDurationSecond - 2) )
            {
                if ( fabs(RobotParams::angleRealTime[1] - conf->deathPos[1]) < 1.0 )
                {
                    bool temp[2] = {false, false};
                    SendMoveCommand(0.0, 0.0, temp, temp, false);
                }
                else
                {
                    values[1] = conf->deathPos[1];
                    ifABS[1] = 1;

                    SendMoveCommandAll(values, ifABS);
                }
            }
            else
            {
                SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, false);
            }

            LoggerStudySamples();
            timeStamp++;
        }
        else if (RobotParams::NVHcurvestate == 1)
        {
            // 绘图
            UpdateQCustomPlotNVH();

            double values[RobotParams::axisNum];
            int ifABS[RobotParams::axisNum];
            for (unsigned int i=0; i<RobotParams::axisNum; ++i)
            {
                values[i] = 0; ifABS[i] = 0;
            }

            // 非换挡情况下 执行油门刹车踏板计算
            double deltaBrake, deltaAcc;

            mySysControl->calConW(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);

            //获得增量控制(开度)
            deltaBrake = mySysControl->getconBrake();
            deltaAcc   = mySysControl->getconAcc();

            // 软件限位
            bool overmax[2] = {false, false};
            bool overmin[2] = {false, false};
            if ((RobotParams::angleRealTime[0] + deltaBrake) > conf->limPos[0])
            {
                overmax[0] = true;
            }
            else if ((RobotParams::angleRealTime[0] + deltaBrake) < conf->deathPos[0])
            {
                overmin[0] = true;
            }
            else { }

            if ((RobotParams::angleRealTime[1] + deltaAcc) > conf->limPos[1])
            {
                overmax[1] = true;
            }
            else if ((RobotParams::angleRealTime[1] + deltaAcc) < conf->deathPos[1])
            {
                overmin[1] = true;
            }
            else { }

            // 换挡结束时 刹车油门和离合的配合
            // 自动挡结束前送油门
            if ( (elapsedSeconds > actionDurationSecond - 2) )
            {
                if ( fabs(RobotParams::angleRealTime[1] - conf->deathPos[1]) < 1.0 )
                {
                    bool temp[2] = {false, false};
                    SendMoveCommand(0.0, 0.0, temp, temp, false);
                }
                else
                {
                    values[1] = conf->deathPos[1];
                    ifABS[1] = 1;

                    SendMoveCommandAll(values, ifABS);
                }
            }
            else
            {
                SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, false);
            }

            LoggerStudySamples();
            timeStamp++;
        }
        else if (RobotParams::NVHcurvestate == 2)
        {
            // 绘图
            UpdateQCustomPlotNVH();

            double values[RobotParams::axisNum];
            int ifABS[RobotParams::axisNum];
            for (unsigned int i=0; i<RobotParams::axisNum; ++i)
            {
                values[i] = 0; ifABS[i] = 0;
            }

            // 换挡结束时 刹车油门和离合的配合
            // 自动挡结束前送油门
            if ( (GetCarSpeed() >= 140) || (elapsedSeconds > actionDurationSecond - 2) )
            {
                if ( fabs(RobotParams::angleRealTime[1] - conf->deathPos[1]) < 1.0 )
                {
                    bool temp[2] = {false, false};
                    SendMoveCommand(0.0, 0.0, temp, temp, false);
                    FinishQCustomPlotNVH(true);
                }
                else
                {
                    values[1] = conf->deathPos[1];
                    ifABS[1] = 1;

                    SendMoveCommandAll(values, ifABS);
                }
            }
            else
            {
                if (fabs(RobotParams::angleRealTime[1] - conf->limPos[1]) < 1.0)
                {
                    SendMoveCommandAll(values, ifABS);
                }
                else
                {
                    values[1] = conf->limPos[1];
                    ifABS[1] = 1;

                    SendMoveCommandAll(values, ifABS);
                }
            }

            LoggerStudySamples();
            timeStamp++;
        }
        else if (RobotParams::NVHcurvestate == 3)
        {
            // 绘图
            UpdateQCustomPlotNVH();

            double values[RobotParams::axisNum];
            int ifABS[RobotParams::axisNum];
            for (unsigned int i=0; i<RobotParams::axisNum; ++i)
            {
                values[i] = 0; ifABS[i] = 0;
            }

            // 非换挡情况下 执行油门刹车踏板计算
            double deltaBrake, deltaAcc;

            mySysControl->calConW(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);

            //获得增量控制(开度)
            deltaBrake = mySysControl->getconBrake();
            deltaAcc   = mySysControl->getconAcc();

            // 软件限位
            bool overmax[2] = {false, false};
            bool overmin[2] = {false, false};
            if ((RobotParams::angleRealTime[0] + deltaBrake) > conf->limPos[0])
            {
                overmax[0] = true;
            }
            else if ((RobotParams::angleRealTime[0] + deltaBrake) < conf->deathPos[0])
            {
                overmin[0] = true;
            }
            else { }

            if ((RobotParams::angleRealTime[1] + deltaAcc) > conf->limPos[1])
            {
                overmax[1] = true;
            }
            else if ((RobotParams::angleRealTime[1] + deltaAcc) < conf->deathPos[1])
            {
                overmin[1] = true;
            }
            else { }

            // 换挡结束时 刹车油门和离合的配合
            // 自动挡结束前送油门
            if ( (elapsedSeconds > actionDurationSecond - 2) )
            {
                if ( fabs(RobotParams::angleRealTime[1] - conf->deathPos[1]) < 1.0 )
                {
                    bool temp[2] = {false, false};
                    SendMoveCommand(0.0, 0.0, temp, temp, false);
                }
                else
                {
                    values[1] = conf->deathPos[1];
                    ifABS[1] = 1;

                    SendMoveCommandAll(values, ifABS);
                }
            }
            else
            {
                SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, false);
            }

            LoggerStudySamples();
            timeStamp++;
        }
        else if (RobotParams::NVHcurvestate == 4)
        {
            if (ifFirstToExitNVH)
            {
                // 停止
                AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

                // 开始监听模式
                std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::examFilePath + "SC_ARM");
                if(fileContent.empty()){
                    PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
                    return;
                }
                AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

                RobotParams::aimshiftindex = 1;

                ifFirstToExitNVH = false;
                return;
            }

            if (RobotParams::round2 < 5)
            {
                RobotParams::round2++;
                return;
            }

            double values[RobotParams::axisNum];
            int ifABS[RobotParams::axisNum];
            for (unsigned int i=0; i<RobotParams::axisNum; ++i)
            {
                values[i] = 0; ifABS[i] = 0;
            }

            if (mySysControl->ifreachedshift(false, 1) && fabs(RobotParams::angleRealTime[0] - conf->brakeThetaAfterGoHome) < 1.0)
            {
                RobotParams::currentshiftindex = 1;
                RobotParams::currentshiftvalue = "N";

                RobotParams::round = 1;
                RobotParams::round2 = 1;
                RobotParams::switchflag[2] = false;

                AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
            }
            else
            {
                double *shiftaims = new double[2];
                mySysControl->getconSft(shiftaims, RobotParams::round);

                values[0] = conf->brakeThetaAfterGoHome;
                values[3] = *shiftaims;
                values[4] = *(shiftaims + 1);
                ifABS[0] = 1;
                ifABS[3] = 1;
                ifABS[4] = 1;

                SendMoveCommandAll(values, ifABS);
                delete shiftaims;

                RobotParams::round++;
            }
        }
    }
    else
    {
        return;
    }

    return;
}

bool PedalRobot::ReadyToNVH()
{
  int ret  = 0;

  RobotParams::currentclutchindex = 1;
  RobotParams::currentclutchvalue = "自动";
  RobotParams::currentshiftindex = 1;
  RobotParams::currentshiftvalue = RobotParams::autoShiftValues[RobotParams::currentshiftindex];
  RobotParams::aimshiftindex = 2;

  // 停止
  AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

  // 开始监听模式
  std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::examFilePath + "SC_ARM");
  if(fileContent.empty()){
      PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
      return false;
  }
  AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

  RobotParams::round = 1;
  RobotParams::round2 = 1;
  RobotParams::switchflag[1] = false;
  RobotParams::switchflag[0] = true;

  ret = QMessageBox::information(
              NULL,"提示", QString(curveFilePath.c_str()) + QObject::tr("请再次确认挡位离合信息后发动车辆!\n点击确认后开始运行!"),
              QObject::tr("确认"),QObject::tr("取消"));

  while (RobotParams::switchflag[0]) {;}

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
      PRINTF(LOG_DEBUG, "%sunknown choice...cancel\n", __func__);
      return false;
  }
}

bool PedalRobot::SelectSpeedCurveNVH(const unsigned int selectMode)
{
    // 选择曲线文件
    QString str = QString::fromStdString(Configuration::mainFolder+"/stdand_files/NVH"+QString::number(selectMode).toStdString());
    if(str==""){
        return false;
    }else if( !QFile::exists(str) ) {
        QMessageBox::information(NULL, "Warning", QObject::tr("所选定的文件不存在，请重新选择！")) ;
        return false;
    }
    curveFilePath = str.toStdString();
    conf->defaultFile = curveFilePath+"_ARM";

    if( !QFile::exists(conf->defaultFile.c_str()) ){
        QMessageBox::information(NULL, "Warning", QObject::tr("用于机器人控制的文件不存在:\n") + conf->defaultFile.c_str());
        return false;
    }

    return true;
}

void PedalRobot::StartQCustomPlotNVH(const std::__cxx11::string &fileNameARM, const unsigned int index)
{
    std::string fileName = fileNameARM.substr(0, fileNameARM.length()-4); // 去除"_ARM"=4 为供UI使用的示教文件
    if(ReadListeningTeachFile(fileName) == -1){
        QMessageBox::information(NULL,"warning",QString("无法读取该目标曲线文件!")+fileName.c_str());
        return;
    }

    size_t sz = vp_data.size();
    PairData tempvp_data;

    switch (index)
    {
    case 0:
        tempvp_data.clear(); tempvp_data.reserve(sz + 3);

        tempvp_data.push_back( std::make_pair(0.0, 8.0) );
        tempvp_data.push_back( std::make_pair(5.0, 8.0) );
        for(size_t i=0; i<sz; ++i){
            tempvp_data.push_back( std::make_pair(vp_data[i].first + 6.0, vp_data[i].second) );
        }
        if ( (tempvp_data[sz + 1].first - tempvp_data[sz].first) == 0 )
        {
            tempvp_data.push_back( std::make_pair(tempvp_data[sz + 1].first + 3.0, tempvp_data[sz + 1].second) );
        }
        else
        {
            tempvp_data.push_back( std::make_pair(tempvp_data[sz + 1].first + 3.0, tempvp_data[sz + 1].second + (tempvp_data[sz + 1].second - tempvp_data[sz].second) / (tempvp_data[sz + 1].first - tempvp_data[sz].first) * 3.0) );
        }

        vp_data.clear(); vp_data.reserve(tempvp_data.size());

        for(size_t i=0; i<tempvp_data.size(); ++i){
            vp_data.push_back( std::make_pair(tempvp_data[i].first, tempvp_data[i].second) );
        }
        break;
    case 1:
        tempvp_data.clear(); tempvp_data.reserve(sz + 3);

        tempvp_data.push_back( std::make_pair(0.0, 0.0) );
        tempvp_data.push_back( std::make_pair(5.0, 0.0) );
        for(size_t i=0; i<sz; ++i){
            tempvp_data.push_back( std::make_pair(vp_data[i].first + 6.0, vp_data[i].second) );
        }
        if ( (tempvp_data[sz + 1].first - tempvp_data[sz].first) == 0 )
        {
            tempvp_data.push_back( std::make_pair(tempvp_data[sz + 1].first + 3.0, tempvp_data[sz + 1].second) );
        }
        else
        {
            tempvp_data.push_back( std::make_pair(tempvp_data[sz + 1].first + 3.0, tempvp_data[sz + 1].second + (tempvp_data[sz + 1].second - tempvp_data[sz].second) / (tempvp_data[sz + 1].first - tempvp_data[sz].first) * 3.0) );
        }

        vp_data.clear(); vp_data.reserve(tempvp_data.size());

        for(size_t i=0; i<tempvp_data.size(); ++i){
            vp_data.push_back( std::make_pair(tempvp_data[i].first, tempvp_data[i].second) );
        }
        break;
    case 2:
        break;
    case 3:
        tempvp_data.clear(); tempvp_data.reserve(sz + 24);

        tempvp_data.push_back( std::make_pair(0.0, 0.0) );
        tempvp_data.push_back( std::make_pair(1.0, 2.2) );
        tempvp_data.push_back( std::make_pair(2.0, 4.6) );
        tempvp_data.push_back( std::make_pair(3.0, 7.2) );
        tempvp_data.push_back( std::make_pair(4.0, 10.0) );
        tempvp_data.push_back( std::make_pair(5.0, 13.0) );
        tempvp_data.push_back( std::make_pair(6.0, 16.2) );
        tempvp_data.push_back( std::make_pair(7.0, 19.6) );
        tempvp_data.push_back( std::make_pair(8.0, 23.2) );
        tempvp_data.push_back( std::make_pair(17.0, 56.8) );
        tempvp_data.push_back( std::make_pair(18.0, 60.4) );
        tempvp_data.push_back( std::make_pair(19.0, 63.8) );
        tempvp_data.push_back( std::make_pair(20.0, 67.0) );
        tempvp_data.push_back( std::make_pair(21.0, 70.0) );
        tempvp_data.push_back( std::make_pair(22.0, 72.8) );
        tempvp_data.push_back( std::make_pair(23.0, 75.4) );
        tempvp_data.push_back( std::make_pair(24.0, 77.8) );
        tempvp_data.push_back( std::make_pair(25.0, 80.0) );
        tempvp_data.push_back( std::make_pair(26.0, 80.0) );
        tempvp_data.push_back( std::make_pair(27.0, 77.8) );
        tempvp_data.push_back( std::make_pair(28.0, 75.4) );
        tempvp_data.push_back( std::make_pair(29.0, 72.8) );
        tempvp_data.push_back( std::make_pair(30.0, 70.2) );
        for(size_t i=0; i<sz; ++i){
            tempvp_data.push_back( std::make_pair(vp_data[i].first + 31.0, vp_data[i].second) );
        }
        if ( (tempvp_data[sz + 1].first - tempvp_data[sz].first) == 0 )
        {
            tempvp_data.push_back( std::make_pair(tempvp_data[sz + 22].first + 3.0, tempvp_data[sz + 22].second) );
        }
        else
        {
            tempvp_data.push_back( std::make_pair(tempvp_data[sz + 22].first + 3.0, tempvp_data[sz + 22].second + (tempvp_data[sz + 22].second - tempvp_data[sz + 21].second) / (tempvp_data[sz + 22].first - tempvp_data[sz + 21].first) * 3.0) );
        }

        vp_data.clear(); vp_data.reserve(tempvp_data.size());

        for(size_t i=0; i<tempvp_data.size(); ++i){
            vp_data.push_back( std::make_pair(tempvp_data[i].first, tempvp_data[i].second) );
        }
        break;
    default:
        break;
    }

    sz = vp_data.size();
    QVector<double> time(sz), speed(sz);
    QVector<double> upperTime(sz), upperSpeed(sz);
    QVector<double> lowerTime(sz), lowerSpeed(sz);
    const double maxY = CalculateUpperLowerBound(time, speed, upperTime, upperSpeed, lowerTime, lowerSpeed);

    InitQCustomPlotNVH1(maxY);
    InitQCustomPlotNVH2(maxY);
    plotNVH2->graph(fileCurve)->setData(time,speed);
    plotNVH2->graph(upperCurve)->setData(upperTime,upperSpeed);
    plotNVH2->graph(lowerCurve)->setData(lowerTime,lowerSpeed);
    plotNVH2->replot();

    // 初始化换挡轮数
    RobotParams::round = 1;
    RobotParams::round2 = 1;
    RobotParams::changeshiftprocess = 0;
    RobotParams::startchangeshifttimeflag = false;
    RobotParams::changeshiftstart = false;
    RobotParams::changeshiftend = false;

    // 运动时间的记录
    gettimeofday(&actionStartTime, NULL);
    PRINTF(LOG_DEBUG, "%s: actionStartTime=%ld:%ld\n", __func__, actionStartTime.tv_sec, actionStartTime.tv_usec);
    actionDurationSecond = time[sz-1] - time[0];
    PRINTF(LOG_DEBUG, "%s: actionDurationSecond=%f\n",__func__,actionDurationSecond);

    InitControlMethod();

    // 准备日志数据
    myLogger->clear();
    myLogger->Customize("ControlParams:\n");

    myLogger->Customize("Run as WLTC(NVH): ");
    for(size_t i=0; i<sizeof(conf->sysControlParamsWltc)/sizeof(conf->sysControlParamsWltc[0]); ++i){
        myLogger->Customize(conf->sysControlParamsWltc[i]).Customize("\t");
    }

    myLogger->Customize("\n");
    myLogger->Customize("TimeStamp\tBrakeOpenValue\tAccOpenValue\tCANSpeed\t\tPULSESpeed\t\tTargetSpeed\t\tControlMethod\tConBrake\t\tConAcc\t\t\tShift\t\t\tClutch\t\t\tBrakeAxis\t\tAccAxis\t\t\tClutchAxis\t\tShiftAxis1\t\tShiftAxis2\t\tSteeringAxis\tCurrentTime\n");
    autoSaveLogger = true;
    PRINTF(LOG_DEBUG, "%s: logger is cleared and ready!\n", __func__);

    timeStamp=0;

    RobotParams::NVHcurvestate = index;
}

void PedalRobot::InitQCustomPlotNVH1(double maxY)
{
    Q_UNUSED(maxY);
}

void PedalRobot::InitQCustomPlotNVH2(double maxY)
{
    plotNVH2->clearGraphs();
    plotNVH2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    plotNVH2->xAxis->setLabel(QObject::tr("时间(s)"));
    plotNVH2->yAxis->setLabel(QObject::tr("速度(km/h)"));

    QPen myPen;
    myPen.setWidth(3);
    //0)目标曲线
    plotNVH2->addGraph();
    myPen.setColor(Qt::red);
    plotNVH2->graph(fileCurve)->setPen(myPen);
    //1)实际曲线
    plotNVH2->addGraph();
    myPen.setColor(Qt::green);
    plotNVH2->graph(actionCurve)->setPen(myPen);

    //2)上界
    plotNVH2->addGraph();
    myPen.setWidth(1);
    myPen.setColor(Qt::blue);
    plotNVH2->graph(upperCurve)->setPen(myPen);
    //3)下界
    plotNVH2->addGraph();
    plotNVH2->graph(lowerCurve)->setPen(myPen);

    plotNVH2->yAxis->setRange(minY, maxY);
}

void PedalRobot::UpdateQCustomPlotNVH()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    elapsedSeconds = GetElapsedMillisecond(actionStartTime, tv)/1000.0 + conf->pedalStartTimeS; // 加上调试开始时间的偏移量

    plotNVH2->graph(actionCurve)->addData(elapsedSeconds, GetCarSpeed());//(s, km/h)
    double lowerX = GetDisplayLowerBound(elapsedSeconds);
    plotNVH2->xAxis->setRange(lowerX, lowerX+30.0);
    plotNVH2->replot();

    if(elapsedSeconds <= actionDurationSecond){
        return;
    }else{
        PRINTF(LOG_DEBUG, "%s: finish this action!. elapsedSeconds/duration=%f/%f\n",
               __func__, elapsedSeconds, actionDurationSecond);
        FinishQCustomPlotNVH(true);
    }
}

void PedalRobot::FinishQCustomPlotNVH(bool showMessageBox)
{
    RobotParams::NVHcurvestate = 9;
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
    CheckIfSaveLogger();

    if(showMessageBox){
        QMessageBox::information(NULL,"提示","该段NVH曲线运行已完成!");
    }
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

void PedalRobot::GetPIDParams(double *params)
{
    return mySysControl->GetPIDParams(params);
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

void PedalRobot::InitParameters()
{
    vp_data.clear();
    vp_data.reserve(64);
    isControlling=false;
    ifFirstToExitNVH = false;
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

            switch(buf[0]){
            case 'R':
                fs.getline(buf,1024);//R段的内容对于UI无用
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
        FinishQCustomPlot();
    }
}

void PedalRobot::FinishQCustomPlot(bool showMessageBox)
{
    isControlling=false;
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

double PedalRobot::GetDisplayUpperBound(double value)
{
    return value>actionDurationSecond-30.0? actionDurationSecond: value+30.0;
}

double PedalRobot::GetCarSpeed()
{
    switch (conf->getSpeedMethod){
    case SettingWidgetPedalRobotGetSpeed::Voltage2Speed:
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

void PedalRobot::SendMoveCommand(double deltaBrake, double deltaAcc, bool *overmax, bool *overmin, bool ifclutchadded, double aimclutch)
{
    std::vector<int> actionMethod;
    std::vector<int> actionAxes;
    std::vector<double> actionTheta;

    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        actionAxes.push_back(i);
    }

    if (overmax[0])
    {
        if (overmax[1])
        {
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->deathPos[0]);
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->deathPos[1]);
        }
        else if (overmin[1])
        {
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->limPos[0]);
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->deathPos[1]);
        }
        else
        {
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->limPos[0]);
            actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
            actionTheta.push_back(deltaAcc);
        }
    }
    else if (overmin[0])
    {
        if (overmax[1])
        {
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->deathPos[0]);
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->limPos[1]);
        }
        else if (overmin[1])
        {
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->deathPos[0]);
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->deathPos[1]);
        }
        else
        {
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->deathPos[0]);
            actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
            actionTheta.push_back(deltaAcc);
        }
    }
    else
    {
        if (overmax[1])
        {
            actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
            actionTheta.push_back(deltaBrake);
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->limPos[1]);
        }
        else if (overmin[1])
        {
            actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
            actionTheta.push_back(deltaBrake);
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionTheta.push_back(conf->deathPos[1]);
        }
        else
        {
            actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
            actionTheta.push_back(deltaBrake);
            actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
            actionTheta.push_back(deltaAcc);
        }
    }

    if (ifclutchadded)
    {
        actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
    }
    else
    {
        actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
    }
    actionTheta.push_back(aimclutch);

    for (unsigned int i=3; i<RobotParams::axisNum; ++i)
    {
        actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
        actionTheta.push_back(0.0);
    }

    AutoDriveRobotApiClient::GetInstance()->Send_SetMonitorActionThetaMsg(actionMethod, actionAxes, actionTheta);
}

void PedalRobot::LoggerStudySamples()
{
    myLogger->Customize(timeStamp).Customize("\t\t\t");
    myLogger->Customize(RobotParams::brakeOpenValue).Customize("\t\t\t");
    myLogger->Customize(RobotParams::accOpenValue).Customize("\t\t\t");
    myLogger->Customize(RobotParams::canCarSpeed).Customize("\t\t\t");
    myLogger->Customize(RobotParams::pulseCarSpeed).Customize("\t\t\t");
    myLogger->Customize(GetCurrentTargetSpeed()).Customize("\t\t\t");
    myLogger->Customize(mySysControl->GetSysControlMethod()).Customize("\t\t\t\t");
    myLogger->Customize(mySysControl->getconBrake()).Customize("\t\t\t");
    myLogger->Customize(mySysControl->getconAcc()).Customize("\t\t\t");

    QString shiftnow = QString::fromStdString(RobotParams::currentshiftvalue);
    if (shiftnow == QString("N_1&2") || shiftnow == QString("N_3&4") || shiftnow == QString("N_5&6"))
    {
        shiftnow = QString("N");
    }
    myLogger->Customize(shiftnow.toStdString().c_str()).Customize("\t\t\t\t");
    myLogger->Customize(RobotParams::currentclutchvalue.c_str()).Customize("\t\t\t");
    myLogger->Customize(RobotParams::angleRealTime[0]).Customize("\t\t\t");
    myLogger->Customize(RobotParams::angleRealTime[1]).Customize("\t\t\t");
    myLogger->Customize(RobotParams::angleRealTime[2]).Customize("\t\t\t");
    myLogger->Customize(RobotParams::angleRealTime[3]).Customize("\t\t\t");
    myLogger->Customize(RobotParams::angleRealTime[4]).Customize("\t\t\t");
    myLogger->Customize(RobotParams::angleRealTime[5]).Customize("\t\t\t");
    myLogger->Customize(QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str()).Customize("\n");
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
    mySysControl->Init(vp_data, conf);
}

void PedalRobot::PedalControl()
{
    if(!isControlling){
        return;
    }

    double deltaBrake, deltaAcc;


    if(conf->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::NedcControl){//NEDC曲线
        mySysControl->calCon(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);
    }else if(conf->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::WltcControl){//WLTC曲线
        mySysControl->calConW(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);
    }

    //获得增量控制(开度)
    deltaBrake = mySysControl->getconBrake();
    deltaAcc   = mySysControl->getconAcc();

    // 软件限位
    bool overmax[2] = {false, false};
    bool overmin[2] = {false, false};
    if ((RobotParams::angleRealTime[0] + deltaBrake) > conf->limPos[0])
    {
        overmax[0] = true;
    }
    else if ((RobotParams::angleRealTime[0] + deltaBrake) < conf->deathPos[0])
    {
        overmin[0] = true;
    }
    else { }

    if ((RobotParams::angleRealTime[1] + deltaAcc) > conf->limPos[1])
    {
        overmax[1] = true;
    }
    else if ((RobotParams::angleRealTime[1] + deltaAcc) < conf->deathPos[1])
    {
        overmin[1] = true;
    }
    else { }

    SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, false);

    LoggerStudySamples();
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

bool PedalRobot::CheckIfAtReady()
{
    if (Configuration::GetInstance()->ifManualShift)
    {
        if (mySysControl->ifreachedshift(false, 1) && mySysControl->ifreachedclutch(false, 1))
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
        if (mySysControl->ifreachedshift(false, 1))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool PedalRobot::GetIsControlling()
{
    return isControlling;
}

bool PedalRobot::isStateIdle()
{
    if ( RobotParams::switchflag[0] || RobotParams::switchflag[1] || RobotParams::switchflag[2] || RobotParams::switchflag[3] || RobotParams::switchflag[4] || RobotParams::isExaming || isControlling)
    {
        return false;
    }
    else
    {
        return true;
    }
}
