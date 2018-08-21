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
    RobotParams::switchflag[5] = false;
    RobotParams::switchflag[6] = false;
    RobotParams::switchflag[7] = false;
    RobotParams::switchflag[8] = false;
    RobotParams::switchflag[9] = false;
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

bool PedalRobot::SelectSpeedCurveACD(const bool selectFile)
{
    int ret = 0;

    ret = QMessageBox::information(
                NULL,"提示", QString(curveFilePath.c_str()) + QObject::tr("请先发动车辆!\n点击确认车辆已发动!"),
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
            // 选择曲线文件
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

            RobotParams::round = 1;
            RobotParams::round2 = 1;

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

    if (conf->ifManualShift && !RobotParams::switchflag[5])
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
    myLogger->Customize("TimeStamp BrakeOpenValue AccOpenValue CANSpeed PULSESpeed TargetSpeed ControlMethod ConBrake ConAcc BrakeAxis AccAxis CurrentTime Shift Clutch ClutchAxis ShiftAxis1 ShiftAxis2 SteeringAxis\n");
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
                    if (fabs(RobotParams::angleRealTime[0] - conf->brakeThetaAfterGoHome) < 1.0)
                    {
                        if (mySysControl->ifreachedshift(false, 2))
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
                        values[0] = conf->brakeThetaAfterGoHome;
                        ifABS[0] = 1;

                        SendMoveCommandAll(values, ifABS);
                    }
                }
            }
            else
            {
                RobotParams::round2++;
                SendMoveCommandAll(values, ifABS);
            }
            return;
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
                if (fabs(RobotParams::angleRealTime[0] - conf->brakeThetaAfterGoHome) < 1.0)
                {
                    if (mySysControl->ifreachedshift(false, 2))
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
                    values[0] = conf->brakeThetaAfterGoHome;
                    ifABS[0] = 1;

                    SendMoveCommandAll(values, ifABS);
                }
            }
            return;
        }
    }

    if (RobotParams::switchflag[3])
    {
        if (!RobotParams::switchflag[4])
        {
            if (RobotParams::round2 > 5)
            {
                RobotParams::switchflag[4] = true;

                if (fabs(RobotParams::angleRealTime[0] - conf->deathPos[0]) < 1.0)
                {
                    RobotParams::currentshiftindex = 2;
                    RobotParams::currentshiftvalue = "D";

                    RobotParams::round = 1;
                    RobotParams::round2 = 1;
                    RobotParams::switchflag[4] = false;
                    RobotParams::switchflag[3] = false;

                    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
                }
                else
                {
                    values[0] = conf->deathPos[0];
                    ifABS[0] = 1;

                    SendMoveCommandAll(values, ifABS);
                }
            }
            else
            {
                RobotParams::round2++;
                SendMoveCommandAll(values, ifABS);
            }
            return;
        }
        else
        {
            if (fabs(RobotParams::angleRealTime[0] - conf->deathPos[0]) < 1.0)
            {
                RobotParams::currentshiftindex = 2;
                RobotParams::currentshiftvalue = "D";

                RobotParams::round = 1;
                RobotParams::round2 = 1;
                RobotParams::switchflag[4] = false;
                RobotParams::switchflag[3] = false;

                AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
            }
            else
            {
                values[0] = conf->deathPos[0];
                ifABS[0] = 1;

                SendMoveCommandAll(values, ifABS);
            }
            return;
        }
    }

    return;
}

void PedalRobot::UpdatePart2()
{
    if (!isControlling || RobotParams::switchflag[0] || RobotParams::switchflag[1] || RobotParams::switchflag[2] || RobotParams::switchflag[3] || RobotParams::switchflag[4] || RobotParams::switchflag[6] || RobotParams::switchflag[7] || RobotParams::switchflag[8] || RobotParams::switchflag[9] || RobotParams::isExaming)
    {
        return;
    }

    // 绘图
    UpdateQCustomPlot();

    //第一次修改

    //第2次修改
    if (modelSelect==0 || controlSelect==0)
    {

        //修改sy//////////////////////////////////////////////////////////////
        double values[RobotParams::axisNum];
        int ifABS[RobotParams::axisNum];
        for (unsigned int i=0; i<RobotParams::axisNum; ++i)
        {
            values[i] = 0; ifABS[i] = 0;
        }

        if (modelSelect == 0)
        {
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

        if (modelSelect == 0)
        {
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
        }
        else
        {
            SendMoveCommand(deltaBrake, deltaAcc, overmax, overmin, false);
        }
        //修改sy//////////////////////////////////////////////////////////////

    }
    //第2次修改

    //修改1//////////////////////////////////////////////////////////////

    LoggerStudySamples();
    timeStamp++;

    return;
}

void PedalRobot::UpdatePart3()
{
    if (isControlling || RobotParams::switchflag[0] || RobotParams::switchflag[1] || RobotParams::switchflag[3] || RobotParams::switchflag[4] || RobotParams::switchflag[5] || RobotParams::switchflag[6] || RobotParams::switchflag[7] || RobotParams::switchflag[8] || RobotParams::switchflag[9] || RobotParams::isExaming)
    {
        return;
    }

    // 是否在NVH曲线运行下
    if (RobotParams::switchflag[2])
    {
        UpdateQCustomPlotSpeed();

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

            mySysControl->calConN(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);

            //获得增量控制(开度)
            deltaBrake = -10;
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

            mySysControl->calConN(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);

            //获得增量控制(开度)
            deltaBrake = -10;
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
            double values[RobotParams::axisNum];
            int ifABS[RobotParams::axisNum];
            for (unsigned int i=0; i<RobotParams::axisNum; ++i)
            {
                values[i] = 0; ifABS[i] = 0;
            }

            if (RobotParams::NVHcurvestate3state == 0)
            {
                // 到速度前踩一半油门
                if ( GetCarSpeed() >= RobotParams::nvh_P1v + 0.5 )
                {
                    if ( fabs(RobotParams::angleRealTime[1] - conf->deathPos[1]) < 1.0 )
                    {
                        SendMoveCommandAll(values, ifABS);
                        RobotParams::NVHcurvestate3state = 1;
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
                    double aim = (conf->limPos[1] + conf->deathPos[1]) / 2;
                    if (fabs(RobotParams::angleRealTime[1] - aim) < 1.0)
                    {
                        SendMoveCommandAll(values, ifABS);
                    }
                    else
                    {
                        values[1] = aim;
                        ifABS[1] = 1;

                        SendMoveCommandAll(values, ifABS);
                    }
                }

            }
            else if (RobotParams::NVHcurvestate3state == 1)
            {
                if ( GetCarSpeed() <= RobotParams::nvh_P1v + 0.5 )
                {
                    size_t sz = vp_data.size();

                    // 运动时间的记录
                    gettimeofday(&actionStartTime, NULL);
                    PRINTF(LOG_DEBUG, "%s: actionStartTime=%ld:%ld\n", __func__, actionStartTime.tv_sec, actionStartTime.tv_usec);
                    actionDurationSecond = vp_data[sz-1].first - vp_data[0].first;
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

                    RobotParams::NVHcurvestate3state = 2;
                }
            }
            else if (RobotParams::NVHcurvestate3state == 2)
            {
                // 绘图
                UpdateQCustomPlotNVH();

                // 非换挡情况下 执行油门刹车踏板计算
                double deltaBrake, deltaAcc;

                mySysControl->calConN(elapsedSeconds, GetCarSpeed(), RobotParams::brakeOpenValue, RobotParams::accOpenValue);

                //获得增量控制(开度)
                deltaBrake = -10;
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

            if (mySysControl->ifreachedshift(false, 1) && fabs(RobotParams::angleRealTime[0] - conf->brakeThetaAfterGoHome) && fabs(RobotParams::angleRealTime[1] - conf->deathPos[1]) < 1.0)
            {
                RobotParams::currentshiftindex = 1;
                RobotParams::currentshiftvalue = "N";

                RobotParams::round = 1;
                RobotParams::round2 = 1;
                RobotParams::switchflag[2] = false;

                FinishQCustomPlotSpeed(false);
                AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
            }
            else
            {
                double *shiftaims = new double[2];
                mySysControl->getconSft(shiftaims, RobotParams::round);

                values[0] = conf->brakeThetaAfterGoHome;
                values[1] = conf->deathPos[1];
                values[3] = *shiftaims;
                values[4] = *(shiftaims + 1);
                ifABS[0] = 1;
                ifABS[1] = 1;
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

void PedalRobot::UpdatePart4()
{
    double values[RobotParams::axisNum];
    int ifABS[RobotParams::axisNum];
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        values[i] = 0; ifABS[i] = 0;
    }

    if (!RobotParams::startchangeshifttimeflag)
    {
        RobotParams::startchangeshifttimeflag = true;
        gettimeofday(&RobotParams::starttime, NULL);
    }

    // 换挡
    if (conf->ifManualShift)
    {
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

        if (RobotParams::changeshiftend)
        {
            if (!RobotParams::iffromNto1)
            {
                if (RobotParams::ifREBA)
                {
                    if (mySysControl->ifreachedclutch(false, 1) && fabs(RobotParams::angleRealTime[0] - RobotParams::tempVars[8]) < 1.0 && fabs(RobotParams::angleRealTime[1] - RobotParams::tempVars[9]) < 1.0)
                    {
                        gettimeofday(&RobotParams::stoptime, NULL);
                        double timeduring = (RobotParams::stoptime.tv_sec-RobotParams::starttime.tv_sec)*1000.0 + (RobotParams::stoptime.tv_usec-RobotParams::starttime.tv_usec)/1000.0;
                        PRINTF(LOG_INFO, "%s: changing shift is finished, totally using time %f ms.\n", __func__, timeduring);

                        RobotParams::currentclutchindex = 1;
                        RobotParams::currentclutchvalue = "松开";

                        RobotParams::startchangeshifttimeflag = false;
                        RobotParams::changeshiftend = false;

                        RobotParams::round = 1;

                        RobotParams::iffromNto1 = false;
                        RobotParams::ifCSACD = false;

                        SendMoveCommandAll(values, ifABS);

                        LoggerStudySamples();
                        timeStamp++;
                        return;
                    }
                    else
                    {
                        RobotParams::currentclutchindex = 2;
                        RobotParams::currentclutchvalue = "抬升";

                        double clutchaim = 0, brkaim = 0, accaim = 0;
                        if ((clutchaim = Configuration::GetInstance()->clutchAngles[0] - RobotParams::round * Configuration::GetInstance()->clutchUpSpeed) < Configuration::GetInstance()->clutchAngles[1])
                        {
                            clutchaim = Configuration::GetInstance()->clutchAngles[1];
                        }
                        values[2] = clutchaim;

                        if ( ( brkaim = Configuration::GetInstance()->deathPos[0] + RobotParams::round * (Configuration::GetInstance()->clutchUpSpeed / (Configuration::GetInstance()->clutchAngles[0] - Configuration::GetInstance()->clutchAngles[1]) * (RobotParams::tempVars[8] - Configuration::GetInstance()->deathPos[0]) ) ) > RobotParams::tempVars[8])
                        {
                            brkaim = RobotParams::tempVars[8];
                        }
                        values[0] = brkaim;

                        if ( ( accaim = Configuration::GetInstance()->deathPos[1] + RobotParams::round * (Configuration::GetInstance()->clutchUpSpeed / (Configuration::GetInstance()->clutchAngles[0] - Configuration::GetInstance()->clutchAngles[1]) * (RobotParams::tempVars[9] - Configuration::GetInstance()->deathPos[1]) ) ) > RobotParams::tempVars[9])
                        {
                            accaim = RobotParams::tempVars[9];
                        }
                        values[1] = accaim;

                        ifABS[0] = 1;
                        ifABS[1] = 1;
                        ifABS[2] = 1;

                        RobotParams::round++;

                        SendMoveCommandAll(values, ifABS);

                        LoggerStudySamples();
                        timeStamp++;
                        return;
                    }
                }
                else
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

                        RobotParams::iffromNto1 = false;
                        RobotParams::ifCSACD = false;

                        SendMoveCommandAll(values, ifABS);

                        LoggerStudySamples();
                        timeStamp++;
                        return;
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

                        SendMoveCommandAll(values, ifABS);

                        LoggerStudySamples();
                        timeStamp++;
                        return;
                    }
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

                    RobotParams::iffromNto1 = false;
                    RobotParams::ifCSACD = false;

                    SendMoveCommandAll(values, ifABS);

                    LoggerStudySamples();
                    timeStamp++;
                    return;
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

                    LoggerStudySamples();
                    timeStamp++;
                    return;
                }
            }
        }
    }
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
              NULL,"提示", QObject::tr("请再次确认挡位离合信息后发动车辆!\n点击确认后开始运行!"),
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
          if (RobotParams::switchflag[0])
          {
              QMessageBox::information(NULL,"警告", QString("按键指令与机械运动冲突，请重试！"));
              PRINTF(LOG_WARNING, "%s: too fast push button.\n", __func__);
              return false;
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
          RobotParams::switchflag[4] = false;
          RobotParams::switchflag[3] = true;

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

    RobotParams::canCarSpeed = 6; // should be solved

    switch (index)
    {
    case 0:
    case 1:
    case 3:
    {
        tempvp_data.clear(); tempvp_data.reserve(sz + 2);
        int lengthsz = sz + 2;

        tempvp_data.push_back( std::make_pair(0.0, vp_data[0].second) );
        for(size_t i=0; i<sz; ++i){
            tempvp_data.push_back( std::make_pair(vp_data[i].first + 2.0, vp_data[i].second) );
        }

        if ( (tempvp_data[lengthsz - 2].first - tempvp_data[lengthsz - 3].first) == 0 )
        {
            tempvp_data.push_back( std::make_pair(tempvp_data[lengthsz - 2].first + 3.0, tempvp_data[lengthsz - 2].second) );
        }
        else
        {
            tempvp_data.push_back( std::make_pair(tempvp_data[lengthsz - 2].first + 3.0, tempvp_data[lengthsz - 2].second + (tempvp_data[lengthsz - 2].second - tempvp_data[lengthsz - 3].second) / (tempvp_data[lengthsz - 2].first - tempvp_data[lengthsz - 3].first) * 3.0) );
        }

        vp_data.clear(); vp_data.reserve(tempvp_data.size());

        for(size_t i=0; i<tempvp_data.size(); ++i){
            vp_data.push_back( std::make_pair(tempvp_data[i].first, tempvp_data[i].second) );
        }
        break;
    }
    case 2:
        break;
    default:
        break;
    }

    sz = vp_data.size();
    QVector<double> time(sz), speed(sz);
    QVector<double> upperTime(sz), upperSpeed(sz);
    QVector<double> lowerTime(sz), lowerSpeed(sz);
    const double maxY = CalculateUpperLowerBound(time, speed, upperTime, upperSpeed, lowerTime, lowerSpeed);

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

    if (RobotParams::NVHcurvestate != 3)
    {
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
    }
    else
    {
        RobotParams::NVHcurvestate3state = 0;
    }

    RobotParams::NVHcurvestate = index;
}

void PedalRobot::StartQCustomPlotSpeed()
{
    InitQCustomPlotNVH1();
    plotNVH1->replot();

    // 运动时间的记录
    gettimeofday(&actionStartTimeSpeed, NULL);
    PRINTF(LOG_DEBUG, "%s: actionStartTimeSpeed=%ld:%ld\n", __func__, actionStartTimeSpeed.tv_sec, actionStartTimeSpeed.tv_usec);
}

void PedalRobot::InitQCustomPlotNVH1()
{
    plotNVH1->clearGraphs();
    plotNVH1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    plotNVH1->xAxis->setLabel(QObject::tr("时间(s)"));
    plotNVH1->yAxis->setLabel(QObject::tr("速度(km/h)"));

    QPen myPen;
    myPen.setWidth(5);

    //1)实际曲线
    plotNVH1->addGraph();
    myPen.setColor(Qt::green);
    plotNVH1->graph(0)->setPen(myPen);
    plotNVH1->yAxis->setRange(GetCarSpeed()-30.0, GetCarSpeed()+30.0);
}

void PedalRobot::UpdateQCustomPlotSpeed()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    elapsedSecondsSpeed = GetElapsedMillisecond(actionStartTimeSpeed, tv)/1000.0;

    plotNVH1->graph(0)->addData(elapsedSecondsSpeed, GetCarSpeed());//(s, km/h)
    double lowerX = GetDisplayLowerBoundSpeed(elapsedSecondsSpeed);
    plotNVH1->xAxis->setRange(lowerX, lowerX+60.0);
    plotNVH1->yAxis->setRange(0.0, GetCarSpeed()+5.0);
    plotNVH1->replot();
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
    RobotParams::NVHcurvestate3state = 0;
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
    CheckIfSaveLogger();

    if(showMessageBox){
        QMessageBox::information(NULL,"提示","该段NVH曲线运行已完成!");
    }
}

void PedalRobot::FinishQCustomPlotSpeed(bool showMessageBox)
{
    if(showMessageBox){
        QMessageBox::information(NULL,"提示","NVH测试已完成!");
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
    /* <ACD> */

    //修改6/////////////////////////////////////////////////////////////////
    lastCmndMd=1;
    lastGetAcclStPos=1.0;//做延时用
    lastGetAcclDlyTmr=1.0;//做延时用
    lastGetBrkPdlStPos=1.0;//做延时用
    lastGetBrkPdlDlyTmr=1.0;//做延时用
    lastGetVehStSpd=1.0;//做online控制 速度初始用

    myGetMTApp=0; //是否手动挡
    myGetTransReqGr=0; //手动档位
    myGetCmndMd=3; //控制模式
    myGetVehStSpd=1.0; //目标速度
    myGetAcclStPos=1.0; //目标油门开度
    myGetAcclDlyTmr=1.0; //油门延迟时间
    myGetBrkPdlStPos=1.0; //目标刹车开度
    myGetBrkPdlDlyTmr=1.0; //刹车延迟时间
    //修改6/////////////////////////////////////////////////////////////////

    /* </ACD> */

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

                /* <ACD> */

                //修改5/////////////////////////////////////////////////////////////////////

                //修改21/////////////////////////////////////////////////////////////////////
                if(modelSelect==1 || modelSelect==2){//ACD的predef模式和online模式
                    //修改21/////////////////////////////////////////////////////////////////////

                    vp_data.clear();
                    //double vpSize=vp_data.size();//这样显示不对，始终0
                    //std::cout<<vpSize<<std::endl;


                    std::cout<<vp_data.size()<<std::endl;

                    vp_data.push_back( std::make_pair(0,0) );
                    vp_data.push_back( std::make_pair(1,0) );
                    vp_data.push_back( std::make_pair(99990,0) );
                    vp_data.push_back( std::make_pair(99999,0) );
                    std::cout<<vp_data.size()<<"   "<<elapsedSeconds<<"   "<<"   "<<vp_data[1].first<<"   "<<vp_data[1].second<<"   "<<std::endl;

                }
                //修改5/////////////////////////////////////////////////////////////////////

                /* </ACD> */

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

/* <ACD> */

//第一次修改
void PedalRobot::RunMyPID() {

    //if(modelSelect!=1){//修改
    //  return;
    //}

    //if(startMyPID2==false){//修改
    //   return;
    //}
    const static QTime t1 = QTime::currentTime();//修改
    QTime t2 = QTime::currentTime();
    int timeElapsed =1000*60*(t2.minute()-t1.minute())+1000*(t2.second()-t1.second())+t2.msec()-t1.msec();

    double values1[RobotParams::axisNum];//栈不能这样赋初值，要像下面一样
    int ifABS1[RobotParams::axisNum];
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        values1[i] = 0; ifABS1[i] = 0;
    }
    double values2[RobotParams::axisNum];//栈不能这样赋初值，要像下面一样
    int ifABS2[RobotParams::axisNum];
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        values2[i] = 0; ifABS2[i] = 0;
    }
    for(int i=0; i<2; ++i){//绝对位置，所以会在0-30间来回运动
        values1[i] = 0.0; ifABS1[i] = 1;
        values2[i] = 80.0; ifABS2[i] = 1;
    }
    //theta2[0] = 0.0;//刹车踏板不动

    timeElapsed %= 6000;
    if(timeElapsed<3000){
        SendMoveCommandAll(values1, ifABS1);
        //std::cout<<"positive"<<timeElapsed<<std::endl;
        //std::cout<<scara_theta[1]<<" InitialMyACD "<<scara_theta[0]<<std::endl;
    }else{
        SendMoveCommandAll(values2, ifABS2);
        //std::cout<<"negative"<<timeElapsed<<std::endl;
        //std::cout<<scara_theta[1]<<" InitialMyACD "<<scara_theta[0]<<std::endl;
    }

}
//第一次修改

//第一次修改
//控制油门开度[1]  ACD模式2 predefined模式 油门延时控制
void PedalRobot::RunAccOpenValue2() {
    if(startAccOpenValue==false){
        return;
    }


    //if(scara_theta[1]>=1)//54ms一个周期足以将就刹车从1退到0
    double values[RobotParams::axisNum];//栈不能这样赋初值，要像下面一样
    int ifABS[RobotParams::axisNum];
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        values[i] = 0; ifABS[i] = 0;
    }
    values[0] = 0.0; ifABS[0] = 1;//刹车踏板不动
    //readAcceleratorOpenValue目标油门开度 ； acceleratorOpenValue当前油门开度
    //theta3[1]设置目标油门踏板位置         ；scara_theta[1] //当前油门踏板位置
    standardAccACD2=1;//设为1，要改改为比1大

    //下面这句是测试用，删除////////////////////////////////////////////////////////
    //acceleratorOpenValue=(scara_theta[1])*0.9;//测试用，删除/////////////////////////////////

    //const static QTime t1 = QTime::currentTime();//修改 pdRobot->initConTime

    QTime t2 = QTime::currentTime();
    int microTimeElapsed =1000*60*(t2.minute()-initConTime.minute())+1000*(t2.second()-initConTime.second())+t2.msec()-initConTime.msec();
    double TimeElapsed=microTimeElapsed/1000.0;
    if(TimeElapsed>=readAccDelayTime)
    {
        TimeElapsed=readAccDelayTime;
    }
    //std::cout<<TimeElapsed<<" time-theta "<<scara_theta[1]<<std::endl;

    //下面这段是加了延时控制  之后那句是没有延时
    if(readAccDelayTime==0)
    {
        values[1]=RobotParams::angleRealTime[1]+(readAccOpenValue-RobotParams::accOpenValue)*standardAccACD2;
    }
    else
    {
        //theta3[1]=scara_theta[1]+(TimeElapsed/readAccDelayTime)*(readAccOpenValue-acceleratorOpenValue)*standardAccACD2;

        //修改11////////////////////////////////////////////////////////////////////
        //theta3[1]=(TimeElapsed/readAccDelayTime)*(scara_theta[1]+(readAccOpenValue-acceleratorOpenValue)*standardAccACD2);
        //theta3[1]=scara_theta[1]+(TimeElapsed/readAccDelayTime)*(TimeElapsed/readAccDelayTime)*(readAccOpenValue-acceleratorOpenValue)*standardAccACD2;
        //theta3[1]=scara1AccStart+(TimeElapsed/readAccDelayTime)*(readAccOpenValue-acceleratorOpenValue)*standardAccACD2;
        values[1]=(1-TimeElapsed/readAccDelayTime)*scara1AccStart+(TimeElapsed/readAccDelayTime)*(RobotParams::angleRealTime[1]+(readAccOpenValue-RobotParams::accOpenValue)*standardAccACD2);
        //修改11////////////////////////////////////////////////////////////////////

    }
    ifABS[1] = 1;

    //std::cout<<TimeElapsed<<" TimeElapsed "<<(TimeElapsed/readAccDelayTime)<<" timeRate-theta3[1] "<<theta3[1]<<" scara "<<scara_theta[1]<<std::endl;
    //theta3[1]=scara_theta[1]+(readAccOpenValue-acceleratorOpenValue)*standardAccACD2;

    SendMoveCommandAll(values, ifABS);
}
//第一次修改

//第一次修改
//控制刹车开度[2]  ACD模式2 predefined模式 刹车延时控制
void PedalRobot::RunBrakeOpenValue2() {
    if(startBrakeOpenValue==false){
        return;
    }

    //if(scara_theta[1]>=1)//54ms一个周期足以将就刹车从1退到0
    double values[RobotParams::axisNum];//栈不能这样赋初值，要像下面一样
    int ifABS[RobotParams::axisNum];
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        values[i] = 0; ifABS[i] = 0;
    }
    values[1] = 0.0; ifABS[1] = 1;//油门踏板不动
    //readAcceleratorOpenValue目标油门开度 ； acceleratorOpenValue当前油门开度
    //theta3[1]设置目标油门踏板位置         ；scara_theta[1] //当前油门踏板位置
    standardAccACD2=1;//设为1，要改改为比1大

    //下面这句是测试用，删除////////////////////////////////////////////////////////
    //brakeOpenValue=(scara_theta[0])*0.9;//测试用，删除/////////////////////////////////

    //const static QTime t1 = QTime::currentTime();//修改 pdRobot->initConTime

    QTime t2 = QTime::currentTime();
    int microTimeElapsed =1000*60*(t2.minute()-initConTime.minute())+1000*(t2.second()-initConTime.second())+t2.msec()-initConTime.msec();
    double TimeElapsed=microTimeElapsed/1000.0;
    if(TimeElapsed>=readBrakeDelayTime)
    {
        TimeElapsed=readBrakeDelayTime;
    }
    //std::cout<<TimeElapsed<<" time-theta "<<scara_theta[0]<<std::endl;

    //下面这段是加了延时控制  之后那句是没有延时
    if(readBrakeDelayTime==0)
    {
        values[0]=RobotParams::angleRealTime[0]+(readBrakeOpenValue-RobotParams::brakeOpenValue)*standardAccACD2;
    }
    else
    {

        //修改11////////////////////////////////////////////////////////////////////
        //theta3[0]=(TimeElapsed/readBrakeDelayTime)*(scara_theta[0]+(readBrakeOpenValue-brakeOpenValue)*standardAccACD2);
        //theta3[0]=scara0BrakeStart+(TimeElapsed/readBrakeDelayTime)*(readBrakeOpenValue-brakeOpenValue)*standardAccACD2;
        values[0]=(1-TimeElapsed/readBrakeDelayTime)*scara0BrakeStart+(TimeElapsed/readBrakeDelayTime)*(RobotParams::angleRealTime[0]+(readBrakeOpenValue-RobotParams::brakeOpenValue)*standardAccACD2);
        //修改11////////////////////////////////////////////////////////////////////

    }
    ifABS[0] = 1;

    //theta3[1]=scara_theta[1]+(readAccOpenValue-acceleratorOpenValue)*standardAccACD2;

    SendMoveCommandAll(values, ifABS);
}
//第一次修改

//第3次修改
void PedalRobot::intialMySysControl(){
    mySysControl->systemModelSelect=modelSelect;
}
//第3次修改

//修改5/////////////////////////////////////////////////////////////////////
void PedalRobot::intialMySysSpeed(){
    mySysControl->sysSpeedStart=speedStart;
    mySysControl->sysReadCarSpeed=readCarSpeed;
    mySysControl->sysCarSelect=carSelect;

    //修改12////////////////////////////////////////////////////////////////////

    //修改21////////////////////////////////////////////////////////////////////
    mySysControl->sysTimeStart=timeStart;
    //修改21////////////////////////////////////////////////////////////////////

    //修改12////////////////////////////////////////////////////////////////////

}

double PedalRobot::GetElapsedSeconds()
{
    return elapsedSeconds;
}

double PedalRobot::GetMyCarSpeed()
{
    return GetCarSpeed();
}

//控制速度[0]  ACD模式2 predefined模式 速度控制
void PedalRobot::RunCarSpeed2(){
    if(startCarSpeed==false){
        return;
    }

    mySysControl->vpdataCarSpeed2();

    //mySysControl->vp_data.clear();
    //std::cout<<elapsedSeconds<<"  "<<vp_data.size()<<"   "<<vp_data[1].first<<"   "<<vp_data[1].second<<std::endl;
    double deltaSpeed=readCarSpeed-speedStart;
    double deltaTen=(deltaSpeed)/(fabs(deltaSpeed))*10.0;

    double acc=1.5;//加速度的定义变量，即10千米用1.5秒到达，折算加速度是6.66>6(wltc最大加速度)
    switch(carSelect)
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
    //std::cout<<"-acc-  "<<acc<<std::endl;

    timeStart=0;//因为每次刷新时elapsedSeconds都归0.且在pedalrobotui里，timeStart的赋值在elapsedSeconds归0之前
    if(timeStart==0)
    {
        if(fabs(deltaSpeed)<10)
        {
            vp_data.clear();
            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart+1,speedStart) );
            vp_data.push_back( std::make_pair(timeStart+1+acc,readCarSpeed) );
            vp_data.push_back( std::make_pair(99999,readCarSpeed) );
        }

        if(fabs(deltaSpeed)>=10)
        {
            double count=floor(fabs(deltaSpeed)/10.0)+1;
            vp_data.clear();

            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart+1,speedStart) );

            for(int i=1;i<count;i++)
            {
                vp_data.push_back( std::make_pair(timeStart+1+acc*i,speedStart+deltaTen*i) );
            }

            vp_data.push_back( std::make_pair(timeStart+1+acc*count,readCarSpeed) );
            vp_data.push_back( std::make_pair(99999,readCarSpeed) );
        }
    }

    if(timeStart!=0)
    {
        if(fabs(deltaSpeed)<10)
        {
            vp_data.clear();
            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart,speedStart) );
            vp_data.push_back( std::make_pair(timeStart+1,speedStart) );
            vp_data.push_back( std::make_pair(timeStart+1+acc,readCarSpeed) );
            vp_data.push_back( std::make_pair(99999,readCarSpeed) );
        }

        if(fabs(deltaSpeed)>=10)
        {
            double count=floor(fabs(deltaSpeed)/10.0)+1;
            vp_data.clear();

            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart,speedStart) );
            vp_data.push_back( std::make_pair(timeStart+1,speedStart) );

            for(int i=1;i<count;i++)
            {
                vp_data.push_back( std::make_pair(timeStart+1+acc*i,speedStart+deltaTen*i) );
            }

            vp_data.push_back( std::make_pair(timeStart+1+acc*count,readCarSpeed) );
            vp_data.push_back( std::make_pair(99999,readCarSpeed) );
        }
    }

}

//修改5/////////////////////////////////////////////////////////////////////
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

//第一次修改
//修改6/////////////////////////////////////////////////////////////////////

//ACD模式1 online模式
void PedalRobot::onlineAccOpenValue1(){
    double values[RobotParams::axisNum];//栈不能这样赋初值，要像下面一样
    int ifABS[RobotParams::axisNum];
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        values[i] = 0; ifABS[i] = 0;
    }
    values[0] = 0.0; ifABS[0] = 1;//刹车踏板不动
    //readAcceleratorOpenValue目标油门开度 ； acceleratorOpenValue当前油门开度
    //theta3[1]设置目标油门踏板位置         ；scara_theta[1] //当前油门踏板位置
    standardAccACD2=1;//设为1，要改改为比1大

    //下面这句是测试用，删除////////////////////////////////////////////////////////
    //acceleratorOpenValue=(scara_theta[1])*0.9;//测试用，删除/////////////////////////////////

    //const static QTime t1 = QTime::currentTime();//修改 pdRobot->initConTime

    QTime t2 = QTime::currentTime();
    int microTimeElapsed =1000*60*(t2.minute()-initConTime1.minute())+1000*(t2.second()-initConTime1.second())+t2.msec()-initConTime1.msec();
    double TimeElapsed=microTimeElapsed/1000.0;
    if(TimeElapsed>=myGetAcclDlyTmr)
    {
        TimeElapsed=myGetAcclDlyTmr;
    }
    //std::cout<<TimeElapsed<<" time-theta "<<scara_theta[1]<<std::endl;

    //下面这段是加了延时控制  之后那句是没有延时
    if(myGetAcclDlyTmr==0)
    {
        values[1]=RobotParams::angleRealTime[1]+(myGetAcclStPos-RobotParams::accOpenValue)*standardAccACD2;
    }
    else
    {
        //theta3[1]=scara_theta[1]+(TimeElapsed/readAccDelayTime)*(readAccOpenValue-acceleratorOpenValue)*standardAccACD2;

        //修改11////////////////////////////////////////////////////////////////////
        //theta3[1]=(TimeElapsed/myGetAcclDlyTmr)*(scara_theta[1]+(myGetAcclStPos-acceleratorOpenValue)*standardAccACD2);
        //theta3[1]=scara1AccStart+(TimeElapsed/myGetAcclDlyTmr)*(myGetAcclStPos-acceleratorOpenValue)*standardAccACD2;
        values[1]=(1-TimeElapsed/myGetAcclDlyTmr)*scara1AccStart+(TimeElapsed/myGetAcclDlyTmr)*(RobotParams::angleRealTime[1]+(myGetAcclStPos-RobotParams::accOpenValue)*standardAccACD2);
        //修改11////////////////////////////////////////////////////////////////////

    }
    ifABS[1] = 1;
    //std::cout<<TimeElapsed<<" TimeElapsed "<<(TimeElapsed/readAccDelayTime)<<" timeRate-theta3[1] "<<theta3[1]<<" scara "<<scara_theta[1]<<std::endl;
    //theta3[1]=scara_theta[1]+(readAccOpenValue-acceleratorOpenValue)*standardAccACD2;

    SendMoveCommandAll(values, ifABS);
}

void PedalRobot::onlineBrakeOpenValue1(){
    double values[RobotParams::axisNum];//栈不能这样赋初值，要像下面一样
    int ifABS[RobotParams::axisNum];
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        values[i] = 0; ifABS[i] = 0;
    }
    values[1] = 0.0; ifABS[1] = 1;//油门踏板不动
    //readAcceleratorOpenValue目标油门开度 ； acceleratorOpenValue当前油门开度
    //theta3[1]设置目标油门踏板位置         ；scara_theta[1] //当前油门踏板位置
    standardAccACD2=1;//设为1，要改改为比1大

    //下面这句是测试用，删除////////////////////////////////////////////////////////
    //brakeOpenValue=(scara_theta[0])*0.9;//测试用，删除/////////////////////////////////

    //const static QTime t1 = QTime::currentTime();//修改 pdRobot->initConTime

    QTime t2 = QTime::currentTime();
    int microTimeElapsed =1000*60*(t2.minute()-initConTime1.minute())+1000*(t2.second()-initConTime1.second())+t2.msec()-initConTime1.msec();
    double TimeElapsed=microTimeElapsed/1000.0;
    if(TimeElapsed>=myGetBrkPdlDlyTmr)
    {
        TimeElapsed=myGetBrkPdlDlyTmr;
    }
    //std::cout<<TimeElapsed<<" time-theta "<<scara_theta[0]<<std::endl;

    //下面这段是加了延时控制  之后那句是没有延时
    if(myGetBrkPdlDlyTmr==0)
    {
        values[0]=RobotParams::angleRealTime[0]+(myGetBrkPdlStPos-RobotParams::brakeOpenValue)*standardAccACD2;
    }
    else
    {

        //修改11////////////////////////////////////////////////////////////////////
        //theta3[0]=(TimeElapsed/myGetBrkPdlDlyTmr)*(scara_theta[0]+(myGetBrkPdlStPos-brakeOpenValue)*standardAccACD2);
        //theta3[0]=scara0BrakeStart+(TimeElapsed/myGetBrkPdlDlyTmr)*(myGetBrkPdlStPos-brakeOpenValue)*standardAccACD2;
        values[0]=(1-TimeElapsed/myGetBrkPdlDlyTmr)*scara0BrakeStart+(TimeElapsed/myGetBrkPdlDlyTmr)*(RobotParams::angleRealTime[0]+(myGetBrkPdlStPos-RobotParams::brakeOpenValue)*standardAccACD2);
        //修改11////////////////////////////////////////////////////////////////////

    }
    ifABS[0] = 1;

    //theta3[1]=scara_theta[1]+(readAccOpenValue-acceleratorOpenValue)*standardAccACD2;

    SendMoveCommandAll(values, ifABS);
}

void PedalRobot::onlineCarSpeed1(){

    //修改12////////////////////////////////////////////////////////////////////
    mySysControl->vpdataCarSpeed1();

    //mySysControl->vp_data.clear();
    //std::cout<<elapsedSeconds<<"  "<<vp_data.size()<<"   "<<vp_data[1].first<<"   "<<vp_data[1].second<<std::endl;
    double deltaSpeed=readCarSpeed-speedStart;
    double deltaTen=(deltaSpeed)/(fabs(deltaSpeed))*10.0;

    double acc=1.5;//加速度的定义变量，即10千米用1.5秒到达，折算加速度是6.66>6(wltc最大加速度)
    switch(carSelect)
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
    //std::cout<<"-acc-  "<<acc<<std::endl;

    //因为本版本online对于elapsedSeconds强制做归零修改，所以下面这句 不用注释掉
    timeStart=0;//因为每次刷新时elapsedSeconds都归0.且在pedalrobotui里，timeStart的赋值在elapsedSeconds归0之前
    if(timeStart==0)
    {
        if(fabs(deltaSpeed)<10)
        {
            vp_data.clear();
            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart+1,speedStart) );
            vp_data.push_back( std::make_pair(timeStart+1+acc,readCarSpeed) );
            vp_data.push_back( std::make_pair(99999,readCarSpeed) );
        }

        if(fabs(deltaSpeed)>=10)
        {
            double count=floor(fabs(deltaSpeed)/10.0)+1;
            vp_data.clear();

            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart+1,speedStart) );

            for(int i=1;i<count;i++)
            {
                vp_data.push_back( std::make_pair(timeStart+1+acc*i,speedStart+deltaTen*i) );
            }

            vp_data.push_back( std::make_pair(timeStart+1+acc*count,readCarSpeed) );
            vp_data.push_back( std::make_pair(99999,readCarSpeed) );
        }
    }

    if(timeStart!=0)
    {
        if(fabs(deltaSpeed)<10)
        {
            vp_data.clear();
            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart,speedStart) );
            vp_data.push_back( std::make_pair(timeStart+1,speedStart) );
            vp_data.push_back( std::make_pair(timeStart+1+acc,readCarSpeed) );
            vp_data.push_back( std::make_pair(99999,readCarSpeed) );
        }

        if(fabs(deltaSpeed)>=10)
        {
            double count=floor(fabs(deltaSpeed)/10.0)+1;
            vp_data.clear();

            vp_data.push_back( std::make_pair(0,0) );
            vp_data.push_back( std::make_pair(timeStart,speedStart) );
            vp_data.push_back( std::make_pair(timeStart+1,speedStart) );

            for(int i=1;i<count;i++)
            {
                vp_data.push_back( std::make_pair(timeStart+1+acc*i,speedStart+deltaTen*i) );
            }

            vp_data.push_back( std::make_pair(timeStart+1+acc*count,readCarSpeed) );
            vp_data.push_back( std::make_pair(99999,readCarSpeed) );
        }
    }

    //修改12////////////////////////////////////////////////////////////////////

}

void PedalRobot::RunOnline1(){
    if(myGetMTApp==0 && myGetCmndMd!=lastCmndMd && myGetCmndMd==1)//油门延时控制用，给初始时间
    {
        initConTime1=QTime::currentTime();

        //修改11////////////////////////////////////////////////////////////////////
        scara1AccStart=RobotParams::angleRealTime[1];
        //修改11////////////////////////////////////////////////////////////////////

    }
    if(myGetMTApp==0 && myGetCmndMd!=lastCmndMd && myGetCmndMd==2)//刹车延时控制用，给初始时间
    {
        initConTime1=QTime::currentTime();

        //修改11////////////////////////////////////////////////////////////////////
        scara0BrakeStart=RobotParams::angleRealTime[0];
        //修改11////////////////////////////////////////////////////////////////////

    }

    if(myGetMTApp==0 && myGetCmndMd==lastCmndMd && myGetCmndMd==1 && lastGetAcclStPos!=myGetAcclStPos)//油门延时控制用，给初始时间
    {
        initConTime1=QTime::currentTime();

        //修改11////////////////////////////////////////////////////////////////////
        scara1AccStart=RobotParams::angleRealTime[1];
        //修改11////////////////////////////////////////////////////////////////////

    }
    if(myGetMTApp==0 && myGetCmndMd==lastCmndMd && myGetCmndMd==1 && lastGetAcclDlyTmr!=myGetAcclDlyTmr)//油门延时控制用，给初始时间
    {
        initConTime1=QTime::currentTime();

        //修改11////////////////////////////////////////////////////////////////////
        scara1AccStart=RobotParams::angleRealTime[1];
        //修改11////////////////////////////////////////////////////////////////////

    }

    if(myGetMTApp==0 && myGetCmndMd==lastCmndMd && myGetCmndMd==2 && lastGetBrkPdlStPos!=myGetBrkPdlStPos)//刹车延时控制用，给初始时间
    {
        initConTime1=QTime::currentTime();

        //修改11////////////////////////////////////////////////////////////////////
        scara0BrakeStart=RobotParams::angleRealTime[0];
        //修改11////////////////////////////////////////////////////////////////////

    }
    if(myGetMTApp==0 && myGetCmndMd==lastCmndMd && myGetCmndMd==2 && lastGetBrkPdlDlyTmr!=myGetBrkPdlDlyTmr)//刹车延时控制用，给初始时间
    {
        initConTime1=QTime::currentTime();

        //修改11////////////////////////////////////////////////////////////////////
        scara0BrakeStart=RobotParams::angleRealTime[0];
        //修改11////////////////////////////////////////////////////////////////////

    }

    //修改12////////////////////////////////////////////////////////////////////
    if(myGetMTApp==0 && myGetCmndMd!=lastCmndMd && myGetCmndMd==0)//速度控制用，用于初始设置
    {
        readCarSpeed=myGetVehStSpd;
        timeStart=floor(GetElapsedSeconds());
        speedStart=GetMyCarSpeed();
        intialMySysSpeed();
        elapsedSeconds=0;
    }

    if(myGetMTApp==0 && myGetCmndMd==lastCmndMd && myGetCmndMd==0 && lastGetVehStSpd!=myGetVehStSpd)//速度控制用，用于初始设置
    {
        readCarSpeed=myGetVehStSpd;
        timeStart=floor(GetElapsedSeconds());
        speedStart=GetMyCarSpeed();
        intialMySysSpeed();
        elapsedSeconds=0;
    }
    //修改12////////////////////////////////////////////////////////////////////

    lastCmndMd=myGetCmndMd;//做延时用
    lastGetAcclStPos=myGetAcclStPos;//做延时用
    lastGetAcclDlyTmr=myGetAcclDlyTmr;//做延时用
    lastGetBrkPdlStPos=myGetBrkPdlStPos;//做延时用
    lastGetBrkPdlDlyTmr=myGetBrkPdlDlyTmr;//做延时用
    lastGetVehStSpd=myGetVehStSpd;//做online控制 速度初始用

    if(myGetMTApp==0 && myGetCmndMd==0)
    {
        onlineCarSpeed1();
    }

    if(myGetMTApp==0 && myGetCmndMd==1)
    {

        //修改11////////////////////////////////////////////////////////////////////
        //scara1AccStart=scara_theta[1];//这么写不对，这样相当于不是给一个初始固定值了，而是实时在给
        //修改11////////////////////////////////////////////////////////////////////

        onlineAccOpenValue1();
    }

    if(myGetMTApp==0 && myGetCmndMd==2)
    {

        //修改11////////////////////////////////////////////////////////////////////
        //scara0BrakeStart=scara_theta[0];//这么写不对，这样相当于不是给一个初始固定值了，而是实时在给
        //修改11////////////////////////////////////////////////////////////////////

        onlineBrakeOpenValue1();
    }
}

//int myGetMTApp; //是否手动挡
//int myGetTransReqGr; //手动档位
//int myGetCmndMd; //控制模式
//double myGetVehStSpd; //目标速度
//double myGetAcclStPos; //目标油门开度
//double myGetAcclDlyTmr; //油门延迟时间
//double myGetBrkPdlStPos; //目标刹车开度
//double myGetBrkPdlDlyTmr; //刹车延迟时间
//修改6/////////////////////////////////////////////////////////////////////

/* </ACD> */















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
    QString shiftnow = QString::fromStdString(RobotParams::currentshiftvalue);
    if (shiftnow == QString("N_1&2") || shiftnow == QString("N_3&4") || shiftnow == QString("N_5&6"))
    {
        shiftnow = QString("N");
    }
    myLogger->Customize(shiftnow.toStdString().c_str()).Customize(" ");
    myLogger->Customize(RobotParams::currentclutchvalue.c_str()).Customize(" ");

    myLogger->Customize(RobotParams::angleRealTime[2]).Customize(" ");
    myLogger->Customize(RobotParams::angleRealTime[3]).Customize(" ");
    myLogger->Customize(RobotParams::angleRealTime[4]).Customize(" ");
    myLogger->Customize(RobotParams::angleRealTime[5]).Customize("\n");
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
    if ( RobotParams::switchflag[0] || RobotParams::switchflag[1] || RobotParams::switchflag[2] || RobotParams::switchflag[3] || RobotParams::switchflag[4] || RobotParams::switchflag[5] || RobotParams::switchflag[6] || RobotParams::switchflag[7] || RobotParams::switchflag[8] || RobotParams::switchflag[9] || RobotParams::isExaming || isControlling)
    {
        return false;
    }
    else
    {
        return true;
    }
}
