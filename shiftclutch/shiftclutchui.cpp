#include "shiftclutchui.h"
#include "ui_shiftclutchui.h"

#include "configuration.h"
#include <QPixmapCache>
#include <QMessageBox>
#include <QTextStream>
#include <QFileDialog>
#include <QInputDialog>
#include <QTimer>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "printf.h"
#include "robotparams.h"
#include "autodriverobotapiclient.h"
#include "robotapi/AssistantFunc/fileassistantfunc.h"
#include "fileoperation/normalfile.h"
#include "pedal/pedalrobot.h"

ShiftClutchUI::ShiftClutchUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ShiftClutchUI),
    startexamtimeflag(false),
    round(1), round2(1),
    haveReadXML(false)
{
    ui->setupUi(this);

    ui->lineEdit_type->installEventFilter(this); // 注册事件过滤

    // 定时器初始化 用来测试
    examtimer = new QTimer(this);
    connect(examtimer, SIGNAL(timeout()), this, SLOT(examtimer_timeout()));

    // 初始化UI
    initialui();

    // 初始化和重置部分控件
    initiallist();
    resetcomboBox();

//    for (unsigned int i=0; i<6; ++i)
//    {
//        actionTheta.push_back(0.0);
//        actionAxes.push_back(i);
//        actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
//    }
}

ShiftClutchUI::~ShiftClutchUI()
{
    delete ui;
    delete examtimer;
}

void ShiftClutchUI::UpdateSC()
{
    ui->lineEdit_motor1->setText( QString::number(RobotParams::angleRealTime[3], 'g', 4) );
    ui->lineEdit_motor2->setText( QString::number(RobotParams::angleRealTime[4], 'g', 4) );
    ui->lineEdit_motor3->setText( QString::number(RobotParams::angleRealTime[2], 'g', 4) );
}

void ShiftClutchUI::UpdateCar()
{
    RobotParams::ifConfirmSC = false;

    resetcomboBox();
    initiallist();

    ui->lineEdit_type->setText( QString::fromStdString( Configuration::GetInstance()->carTypeName.substr(0, Configuration::GetInstance()->carTypeName.length() - 4) ) );
    ifenablewaychangedeventhappen = false;
    ui->comboBox_way->setCurrentIndex(!Configuration::GetInstance()->ifManualShift);
    ifenablewaychangedeventhappen =  true;
    ifenablebackzeroeventhappen = false;
    ui->checkBox_zero->setChecked(Configuration::GetInstance()->ifGoBack);
    ifenablebackzeroeventhappen = true;

    QPixmapCache::clear();

    ui->checkBox_autoset->setEnabled(Configuration::GetInstance()->ifAutoRecordMidN);

    if (Configuration::GetInstance()->ifManualShift)
    {
        const QString picpath = QString::fromStdString(Configuration::mainFolder) + "/manualshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(true);

        ui->tab_exam->setEnabled(true);
        ui->pushButton_run->setEnabled(true);
        ui->pushButton_pause->setEnabled(true);
        ui->pushButton_0to1->setEnabled(true);
        ui->pushButton_1to0->setEnabled(true);
        ui->tab_examclutch->setEnabled(true);

        ui->tab_changeshift->setEnabled(true);
        if (Configuration::GetInstance()->pedalRobotUsage == 0)
        {
            ui->lineEdit_1to2->setEnabled(false);
            ui->lineEdit_2to3->setEnabled(false);
            ui->lineEdit_3to4->setEnabled(false);
            ui->lineEdit_4to5->setEnabled(false);
            ui->lineEdit_5to4->setEnabled(false);
            ui->lineEdit_4to3->setEnabled(false);
            ui->lineEdit_3to2->setEnabled(false);
            ui->lineEdit_atoN->setEnabled(false);

            ui->tabWidget_NW->setCurrentIndex(0);
            ui->plotwidget->clearGraphs();
            ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
            ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
            ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
            ui->plotwidget->xAxis->setRange(0,1800);
            ui->plotwidget->yAxis->setRange(-5, 140);

            RobotParams::ifConfirmCS = false;
        }
        else if (Configuration::GetInstance()->pedalRobotUsage == 1)
        {
            ui->lineEdit_1to2->setEnabled(true);
            ui->lineEdit_2to3->setEnabled(true);
            ui->lineEdit_3to4->setEnabled(true);
            ui->lineEdit_4to5->setEnabled(true);
            ui->lineEdit_5to4->setEnabled(true);
            ui->lineEdit_4to3->setEnabled(true);
            ui->lineEdit_3to2->setEnabled(true);
            ui->lineEdit_atoN->setEnabled(true);

            ui->tabWidget_NW->setCurrentIndex(1);
            ui->plotwidget->clearGraphs();
            ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
            ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
            ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
            ui->plotwidget->xAxis->setRange(0,1800);
            ui->plotwidget->yAxis->setRange(-5, 140);

            RobotParams::ifConfirmCS = false;
        }

        ui->checkBox_autoset->setEnabled(true);
    }
    else
    {
        const QString picpath = QString::fromStdString(Configuration::mainFolder) + "/autoshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(false);

        ui->tab_exam->setEnabled(true);
        ui->pushButton_run->setEnabled(false);
        ui->pushButton_pause->setEnabled(false);
        ui->pushButton_0to1->setEnabled(false);
        ui->pushButton_1to0->setEnabled(false);
        ui->tab_examclutch->setEnabled(false);

        ui->tab_changeshift->setEnabled(false);

        ui->checkBox_autoset->setEnabled(false);
    }
}

void ShiftClutchUI::UpdateUsage()
{
    if (Configuration::GetInstance()->ifManualShift)
    {
        ui->tab_changeshift->setEnabled(true);
        if (Configuration::GetInstance()->pedalRobotUsage == 0)
        {
            ui->lineEdit_1to2->setEnabled(false);
            ui->lineEdit_2to3->setEnabled(false);
            ui->lineEdit_3to4->setEnabled(false);
            ui->lineEdit_4to5->setEnabled(false);
            ui->lineEdit_5to4->setEnabled(false);
            ui->lineEdit_4to3->setEnabled(false);
            ui->lineEdit_3to2->setEnabled(false);
            ui->lineEdit_atoN->setEnabled(false);

            ui->tabWidget_NW->setCurrentIndex(0);
            ui->plotwidget->clearGraphs();
            ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
            ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
            ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
            ui->plotwidget->xAxis->setRange(0,1800);
            ui->plotwidget->yAxis->setRange(-5, 140);

            RobotParams::ifConfirmCS = false;
        }
        else if (Configuration::GetInstance()->pedalRobotUsage == 1)
        {
            ui->lineEdit_1to2->setEnabled(true);
            ui->lineEdit_2to3->setEnabled(true);
            ui->lineEdit_3to4->setEnabled(true);
            ui->lineEdit_4to5->setEnabled(true);
            ui->lineEdit_5to4->setEnabled(true);
            ui->lineEdit_4to3->setEnabled(true);
            ui->lineEdit_3to2->setEnabled(true);
            ui->lineEdit_atoN->setEnabled(true);

            ui->tabWidget_NW->setCurrentIndex(1);
            ui->plotwidget->clearGraphs();
            ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
            ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
            ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
            ui->plotwidget->xAxis->setRange(0,1800);
            ui->plotwidget->yAxis->setRange(-5, 140);

            RobotParams::ifConfirmCS = false;
        }
    }
    else
    {
        ui->tab_changeshift->setEnabled(false);
    }
}

void ShiftClutchUI::initialui()
{
    // 显示图像 修改部分控件使能
    QPixmapCache::clear();
    if (Configuration::GetInstance()->ifManualShift)
    {
        const QString picpath = QString::fromStdString(Configuration::mainFolder) + "/manualshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(true);

        ui->tab_exam->setEnabled(true);
        ui->pushButton_run->setEnabled(true);
        ui->pushButton_pause->setEnabled(true);
        ui->pushButton_0to1->setEnabled(true);
        ui->pushButton_1to0->setEnabled(true);
        ui->tab_examclutch->setEnabled(true);

        ui->tab_changeshift->setEnabled(true);
        if (Configuration::GetInstance()->pedalRobotUsage == 0)
        {
            ui->lineEdit_1to2->setEnabled(false);
            ui->lineEdit_2to3->setEnabled(false);
            ui->lineEdit_3to4->setEnabled(false);
            ui->lineEdit_4to5->setEnabled(false);
            ui->lineEdit_5to4->setEnabled(false);
            ui->lineEdit_4to3->setEnabled(false);
            ui->lineEdit_3to2->setEnabled(false);
            ui->lineEdit_atoN->setEnabled(false);

            ui->tabWidget_NW->setCurrentIndex(0);
            ui->plotwidget->clearGraphs();
            ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
            ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
            ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
            ui->plotwidget->xAxis->setRange(0,1800);
            ui->plotwidget->yAxis->setRange(-5, 140);

            RobotParams::ifConfirmCS = false;
        }
        else if (Configuration::GetInstance()->pedalRobotUsage == 1)
        {
            ui->lineEdit_1to2->setEnabled(true);
            ui->lineEdit_2to3->setEnabled(true);
            ui->lineEdit_3to4->setEnabled(true);
            ui->lineEdit_4to5->setEnabled(true);
            ui->lineEdit_5to4->setEnabled(true);
            ui->lineEdit_4to3->setEnabled(true);
            ui->lineEdit_3to2->setEnabled(true);
            ui->lineEdit_atoN->setEnabled(true);

            ui->tabWidget_NW->setCurrentIndex(1);
            ui->plotwidget->clearGraphs();
            ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
            ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
            ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
            ui->plotwidget->xAxis->setRange(0,1800);
            ui->plotwidget->yAxis->setRange(-5, 140);

            RobotParams::ifConfirmCS = false;
        }
    }
    else
    {
        const QString picpath = QString::fromStdString(Configuration::mainFolder) + "/autoshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(false);

        ui->tab_exam->setEnabled(true);
        ui->pushButton_run->setEnabled(false);
        ui->pushButton_pause->setEnabled(false);
        ui->pushButton_0to1->setEnabled(false);
        ui->pushButton_1to0->setEnabled(false);
        ui->tab_examclutch->setEnabled(false);

        ui->tab_changeshift->setEnabled(false);
    }

    // 初始化变量
    ifenablewaychangedeventhappen = true;
    ifenablebackzeroeventhappen = true;
    ifenablecomboBoxchangedeventhappen = true;

    // 初始化界面
    ui->lineEdit_speed->setEnabled(false);
    ui->pushButton_stopexam->setEnabled(false);
    ui->tabWidget2->setEnabled(false);

    ui->lineEdit_type->setText( QString::fromStdString( Configuration::GetInstance()->carTypeName.substr(0, Configuration::GetInstance()->carTypeName.length() - 4) ) );
    ifenablewaychangedeventhappen = false;
    ui->comboBox_way->setCurrentIndex(!Configuration::GetInstance()->ifManualShift);
    ifenablewaychangedeventhappen =  true;
    ifenablebackzeroeventhappen = false;
    ui->checkBox_zero->setChecked(Configuration::GetInstance()->ifGoBack);
    ifenablebackzeroeventhappen = true;
}

void ShiftClutchUI::initiallist()
{
    ui->list1->clear();
    ui->list2->clear();

    if (Configuration::GetInstance()->ifManualShift)
    {
        ui->list1->addItem(tr("挡位\t\t\t\t角度1/deg\t\t\t角度2/deg"));
        ui->list1->addItem( tr( (QString("N_1&2\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[0], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[0], 'f', 2)).toStdString().c_str() ) );
        ui->list1->addItem( tr( (QString("N_3&4\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[1], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[1], 'f', 2)).toStdString().c_str() ) );
        ui->list1->addItem( tr( (QString("N_5&6\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[2], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[2], 'f', 2)).toStdString().c_str() ) );
        ui->list1->addItem( tr( (QString("1    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[3], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[3], 'f', 2)).toStdString().c_str() ) );
        ui->list1->addItem( tr( (QString("2    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[4], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[4], 'f', 2)).toStdString().c_str() ) );
        ui->list1->addItem( tr( (QString("3    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[5], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[5], 'f', 2)).toStdString().c_str() ) );
        ui->list1->addItem( tr( (QString("4    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[6], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[6], 'f', 2)).toStdString().c_str() ) );
        ui->list1->addItem( tr( (QString("5    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[7], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[7], 'f', 2)).toStdString().c_str() ) );
        ui->list1->addItem( tr( (QString("6    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[8], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[8], 'f', 2)).toStdString().c_str() ) );
    }
    else
    {
        ui->list1->addItem( tr("挡位\t\t\t\t角度1/deg\t\t\t角度2/deg"));
        ui->list1->addItem( tr( (QString("P    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[0], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[0], 'f', 2)).toStdString().c_str() ) );
        ui->list1->addItem( tr( (QString("N    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[1], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[1], 'f', 2)).toStdString().c_str() ) );
        ui->list1->addItem( tr( (QString("D    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[2], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[2], 'f', 2)).toStdString().c_str() ) );
    }

    ui->list2->addItem(tr("位置\t\t\t\t\t\t\t角度/deg\t\t\t\t\t速度"));
    ui->list2->addItem( tr( (QString("踩住位置\t\t\t\t\t\t ") + QString::number(Configuration::GetInstance()->clutchAngles[0], 'f', 2) + QString("\t\t\t\t\t\t / ")).toStdString().c_str() ) );
    ui->list2->addItem( tr( (QString("松开位置\t\t\t\t\t\t ") + QString::number(Configuration::GetInstance()->clutchAngles[1], 'f', 2) + QString("\t\t\t\t\t\t / ")).toStdString().c_str() ) );
    ui->list2->addItem( tr( (QString("松开速度\t\t\t\t\t\t /    \t\t\t\t\t\t ") + QString::number(Configuration::GetInstance()->clutchUpSpeed, 'f', 2)).toStdString().c_str() ) );
}

void ShiftClutchUI::resetlist()
{
    ui->list1->clear();
    ui->list2->clear();

    if (Configuration::GetInstance()->ifManualShift)
    {
        ui->list1->addItem(tr("挡位\t\t\t\t角度1/deg\t\t\t角度2/deg"));
        ui->list1->addItem(tr("N_1&2\t\t\t\t 00.00\t\t\t\t 00.00"));
        ui->list1->addItem(tr("N_3&4\t\t\t\t 00.00\t\t\t\t 00.00"));
        ui->list1->addItem(tr("N_5&6\t\t\t\t 00.00\t\t\t\t 00.00"));
        ui->list1->addItem(tr("1    \t\t\t\t 00.00\t\t\t\t 00.00"));
        ui->list1->addItem(tr("2    \t\t\t\t 00.00\t\t\t\t 00.00"));
        ui->list1->addItem(tr("3    \t\t\t\t 00.00\t\t\t\t 00.00"));
        ui->list1->addItem(tr("4    \t\t\t\t 00.00\t\t\t\t 00.00"));
        ui->list1->addItem(tr("5    \t\t\t\t 00.00\t\t\t\t 00.00"));
        ui->list1->addItem(tr("6    \t\t\t\t 00.00\t\t\t\t 00.00"));
    }
    else
    {
        ui->list1->addItem(tr("挡位\t\t\t\t角度1/deg\t\t\t角度2/deg"));
        ui->list1->addItem(tr("P    \t\t\t\t 00.00\t\t\t\t 00.00"));
        ui->list1->addItem(tr("N    \t\t\t\t 00.00\t\t\t\t 00.00"));
        ui->list1->addItem(tr("D    \t\t\t\t 00.00\t\t\t\t 00.00"));
    }

    ui->list2->addItem(tr("位置\t\t\t\t\t\t\t角度/deg\t\t\t\t\t速度"));
    ui->list2->addItem(tr("踩住位置\t\t\t\t\t\t 00.00\t\t\t\t\t\t / "));
    ui->list2->addItem(tr("松开位置\t\t\t\t\t\t 00.00\t\t\t\t\t\t / "));
    ui->list2->addItem(tr("松开速度\t\t\t\t\t\t /    \t\t\t\t\t\t 0.00"));

    for (int i=0; i<9; ++i)
    {
        Configuration::GetInstance()->shiftAxisAngles1[i] = 0.0;
        Configuration::GetInstance()->shiftAxisAngles2[i] = 0.0;
    }
    for (int i=0; i<2; ++i)
    {
        Configuration::GetInstance()->clutchAngles[i] = 0.0;
    }
    Configuration::GetInstance()->clutchUpSpeed = 0.0;
}

void ShiftClutchUI::resetcomboBox()
{
    ifenablecomboBoxchangedeventhappen = false;
    ui->comboBox_shift->clear();
    ui->comboBox_shiftaim->clear();

    if (Configuration::GetInstance()->ifManualShift)
    {
        ui->comboBox_shift->insertItem(0, "N_1&2");
        ui->comboBox_shift->insertItem(1, "N_3&4");
        ui->comboBox_shift->insertItem(2, "N_5&6");
        ui->comboBox_shift->insertItem(3, "1");
        ui->comboBox_shift->insertItem(4, "2");
        ui->comboBox_shift->insertItem(5, "3");
        ui->comboBox_shift->insertItem(6, "4");
        ui->comboBox_shift->insertItem(7, "5");
        ui->comboBox_shift->insertItem(8, "6");

        ui->comboBox_shiftaim->insertItem(0, "N_1&2");
        ui->comboBox_shiftaim->insertItem(1, "N_3&4");
        ui->comboBox_shiftaim->insertItem(2, "N_5&6");
        ui->comboBox_shiftaim->insertItem(3, "1");
        ui->comboBox_shiftaim->insertItem(4, "2");
        ui->comboBox_shiftaim->insertItem(5, "3");
        ui->comboBox_shiftaim->insertItem(6, "4");
        ui->comboBox_shiftaim->insertItem(7, "5");
        ui->comboBox_shiftaim->insertItem(8, "6");
    }
    else
    {
        ui->comboBox_shift->insertItem(0, "P");
        ui->comboBox_shift->insertItem(1, "N");
        ui->comboBox_shift->insertItem(2, "D");

        ui->comboBox_shiftaim->insertItem(0, "P");
        ui->comboBox_shiftaim->insertItem(1, "N");
        ui->comboBox_shiftaim->insertItem(2, "D");
    }

    ifenablecomboBoxchangedeventhappen = true;
}

void ShiftClutchUI::examtimer_timeout()
{
    //软件倍频
    if(examtimer->isActive()){
        static int cnt = 0;
        if(++cnt%RobotParams::UITimerMultiplier != 0){
//            AutoDriveRobotApiClient::GetInstance()->Send_SetMonitorActionThetaMsg(actionMethod, actionAxes, actionTheta);
            return;
        }
    }

//    actionMethod.clear();
//    actionAxes.clear();
//    actionTheta.clear();

    ui->lineEdit_shiftnow->setText( QString::fromStdString( RobotParams::currentshiftvalue ) );

    if (examflag == 0)
    {
        SendMoveCommand(0,0,0,false,true,false);
        PRINTF(LOG_ERR, "%s: stamp = %d; mode = %d; %s\n", __func__, timestamp_test, examflag, QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
        timestamp_test++;
    }
    else if (examflag == 1)
    {
        if (Configuration::GetInstance()->ifManualShift)
        {
            if (mySCControl->ifreachedatinitial(0.2))
            {
                examflag = 0;
                round = 1;
                PRINTF(LOG_INFO, "%s: be ready to exam.\n", __func__);

                // 界面设置
                ui->pushButton_startexam->setEnabled(false);
                ui->pushButton_stopexam->setEnabled(true);
                ui->tabWidget2->setEnabled(true);
                ui->tab_examshift->setEnabled(true);
                ui->tab_examclutch->setEnabled(true);

                ui->pushButton_shiftrun->setEnabled(true);
                ui->pushButton_shiftpause->setEnabled(false);
                ui->pushButton_run->setEnabled(true);
                ui->pushButton_pause->setEnabled(false);
                ui->pushButton_0to1->setEnabled(true);
                ui->pushButton_1to0->setEnabled(false);
                ui->pushButton_bottom->setEnabled(true);
                ui->pushButton_top->setEnabled(false);
                ui->pushButton_speed->setEnabled(false);
                ui->pushButton_clutchpause->setEnabled(false);
            }
            else
            {
                double brakeaim = 0, accaim = 0;
                double brakespeed = 2, accspeed = 2;
                int brakesgn = 0, accsgn = 0;

                brakesgn = mySCControl->sgn(Configuration::GetInstance()->deathPos[0] - RobotParams::tempVars[5]);
                accsgn = mySCControl->sgn(Configuration::GetInstance()->deathPos[1] - RobotParams::tempVars[6]);

                if (brakesgn > 0)
                {
                    if ((brakeaim = RobotParams::tempVars[5] + round * brakespeed) > Configuration::GetInstance()->deathPos[0])
                    {
                        brakeaim = Configuration::GetInstance()->deathPos[0];
                    }
                }
                else if (brakesgn < 0)
                {
                    if ((brakeaim = RobotParams::tempVars[5] - round * brakespeed) < Configuration::GetInstance()->deathPos[0])
                    {
                        brakeaim = Configuration::GetInstance()->deathPos[0];
                    }
                }
                else
                {
                    brakeaim = Configuration::GetInstance()->deathPos[0];
                }

                if (accsgn > 0)
                {
                    if ((accaim = RobotParams::tempVars[6] + round * accspeed) > Configuration::GetInstance()->deathPos[1])
                    {
                        accaim = Configuration::GetInstance()->deathPos[1];
                    }
                }
                else if (accsgn < 0)
                {
                    if ((accaim = RobotParams::tempVars[6] - round * accspeed) < Configuration::GetInstance()->deathPos[1])
                    {
                        accaim = Configuration::GetInstance()->deathPos[1];
                    }
                }
                else
                {
                    accaim = Configuration::GetInstance()->deathPos[1];
                }

                double shiftaim1 = 0, shiftaim2 = 0, clutchaim = 0;
                double shiftspeed1 = 0.2, shiftspeed2 = 0.2, clutchspeed = 1.0;
                int shiftsgn1 = 0, shiftsgn2 = 0, clutchsgn = 0;

                clutchsgn = mySCControl->sgn(Configuration::GetInstance()->clutchAngles[1] - RobotParams::tempVars[0]);
                shiftsgn1 = mySCControl->sgn(Configuration::GetInstance()->shiftAxisAngles1[1] - RobotParams::tempVars[1]);
                shiftsgn2 = mySCControl->sgn(Configuration::GetInstance()->shiftAxisAngles2[1] - RobotParams::tempVars[2]);

                if (clutchsgn > 0)
                {
                    if ((clutchaim = RobotParams::tempVars[0] + round * clutchspeed) > Configuration::GetInstance()->clutchAngles[1])
                    {
                        clutchaim = Configuration::GetInstance()->clutchAngles[1];
                    }
                }
                else if (clutchsgn < 0)
                {
                    if ((clutchaim = RobotParams::tempVars[0] - round * clutchspeed) < Configuration::GetInstance()->clutchAngles[1])
                    {
                        clutchaim = Configuration::GetInstance()->clutchAngles[1];
                    }
                }
                else
                {
                    clutchaim = Configuration::GetInstance()->clutchAngles[1];
                }

                if (shiftsgn1 > 0)
                {
                    if ((shiftaim1 = RobotParams::tempVars[1] + round * shiftspeed1) > Configuration::GetInstance()->shiftAxisAngles1[1])
                    {
                        shiftaim1 = Configuration::GetInstance()->shiftAxisAngles1[1];
                    }
                }
                else if (shiftsgn1 < 0)
                {
                    if ((shiftaim1 = RobotParams::tempVars[1] - round * shiftspeed1) < Configuration::GetInstance()->shiftAxisAngles1[1])
                    {
                        shiftaim1 = Configuration::GetInstance()->shiftAxisAngles1[1];
                    }
                }
                else
                {
                    shiftaim1 = Configuration::GetInstance()->shiftAxisAngles1[1];
                }

                if (shiftsgn2 > 0)
                {
                    if ((shiftaim2 = RobotParams::tempVars[2] + round * shiftspeed2) > Configuration::GetInstance()->shiftAxisAngles2[1])
                    {
                        shiftaim2 = Configuration::GetInstance()->shiftAxisAngles2[1];
                    }
                }
                else if (shiftsgn2 < 0)
                {
                    if ((shiftaim2 = RobotParams::tempVars[2] - round * shiftspeed2) < Configuration::GetInstance()->shiftAxisAngles2[1])
                    {
                        shiftaim2 = Configuration::GetInstance()->shiftAxisAngles2[1];
                    }
                }
                else
                {
                    shiftaim2 = Configuration::GetInstance()->shiftAxisAngles2[1];
                }

                double values[RobotParams::axisNum];
                int ifABS[RobotParams::axisNum];
                for (unsigned int i=0; i<RobotParams::axisNum; ++i)
                {
                    values[i] = 0; ifABS[i] = 0;
                }

                values[0] = brakeaim;
                values[1] = accaim;
                values[2] = clutchaim;
                values[3] = shiftaim1;
                values[4] = shiftaim2;
                ifABS[0] = 1;
                ifABS[1] = 1;
                ifABS[2] = 1;
                ifABS[3] = 1;
                ifABS[4] = 1;

                SendMoveCommandAll(values, ifABS);

                round++;

                PRINTF(LOG_ERR, "%s: stamp = %d; mode = %d; %s\n", __func__, timestamp_test, examflag, QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
                timestamp_test++;
            }
        }
        else
        {
            if (mySCControl->ifreachedatinitial(0.2))
            {
                examflag = 0;
                round = 1;
                PRINTF(LOG_INFO, "%s: be ready to exam.\n", __func__);

                // 界面设置
                ui->pushButton_startexam->setEnabled(false);
                ui->pushButton_stopexam->setEnabled(true);
                ui->tabWidget2->setEnabled(true);
                ui->tab_examshift->setEnabled(true);
                ui->tab_examclutch->setEnabled(false);

                ui->pushButton_shiftrun->setEnabled(true);
                ui->pushButton_shiftpause->setEnabled(false);

            }
            else
            {
                double brakeaim = 0, accaim = 0;
                double brakespeed = 2, accspeed = 2;
                int brakesgn = 0, accsgn = 0;

                brakesgn = mySCControl->sgn(Configuration::GetInstance()->brakeThetaAfterGoHome - RobotParams::tempVars[5]);
                accsgn = mySCControl->sgn(Configuration::GetInstance()->deathPos[1] - RobotParams::tempVars[6]);

                if (brakesgn > 0)
                {
                    if ((brakeaim = RobotParams::tempVars[5] + round * brakespeed) > Configuration::GetInstance()->brakeThetaAfterGoHome)
                    {
                        brakeaim = Configuration::GetInstance()->brakeThetaAfterGoHome;
                    }
                }
                else if (brakesgn < 0)
                {
                    if ((brakeaim = RobotParams::tempVars[5] - round * brakespeed) < Configuration::GetInstance()->brakeThetaAfterGoHome)
                    {
                        brakeaim = Configuration::GetInstance()->brakeThetaAfterGoHome;
                    }
                }
                else
                {
                    brakeaim = Configuration::GetInstance()->brakeThetaAfterGoHome;
                }

                if (accsgn > 0)
                {
                    if ((accaim = RobotParams::tempVars[6] + round * accspeed) > Configuration::GetInstance()->deathPos[1])
                    {
                        accaim = Configuration::GetInstance()->deathPos[1];
                    }
                }
                else if (accsgn < 0)
                {
                    if ((accaim = RobotParams::tempVars[6] - round * accspeed) < Configuration::GetInstance()->deathPos[1])
                    {
                        accaim = Configuration::GetInstance()->deathPos[1];
                    }
                }
                else
                {
                    accaim = Configuration::GetInstance()->deathPos[1];
                }

                double shiftaim1 = 0, shiftaim2 = 0, clutchaim = 0;
                double shiftspeed1 = 0.2, shiftspeed2 = 0.2, clutchspeed = 1.0;
                int shiftsgn1 = 0, shiftsgn2 = 0, clutchsgn = 0;
                Q_UNUSED(clutchaim);Q_UNUSED(clutchspeed);Q_UNUSED(clutchsgn);

                shiftsgn1 = mySCControl->sgn(Configuration::GetInstance()->shiftAxisAngles1[1] - RobotParams::tempVars[1]);
                shiftsgn2 = mySCControl->sgn(Configuration::GetInstance()->shiftAxisAngles2[1] - RobotParams::tempVars[2]);

                if (shiftsgn1 > 0)
                {
                    if ((shiftaim1 = RobotParams::tempVars[1] + round * shiftspeed1) > Configuration::GetInstance()->shiftAxisAngles1[1])
                    {
                        shiftaim1 = Configuration::GetInstance()->shiftAxisAngles1[1];
                    }
                }
                else if (shiftsgn1 < 0)
                {
                    if ((shiftaim1 = RobotParams::tempVars[1] - round * shiftspeed1) < Configuration::GetInstance()->shiftAxisAngles1[1])
                    {
                        shiftaim1 = Configuration::GetInstance()->shiftAxisAngles1[1];
                    }
                }
                else
                {
                    shiftaim1 = Configuration::GetInstance()->shiftAxisAngles1[1];
                }

                if (shiftsgn2 > 0)
                {
                    if ((shiftaim2 = RobotParams::tempVars[2] + round * shiftspeed2) > Configuration::GetInstance()->shiftAxisAngles2[1])
                    {
                        shiftaim2 = Configuration::GetInstance()->shiftAxisAngles2[1];
                    }
                }
                else if (shiftsgn2 < 0)
                {
                    if ((shiftaim2 = RobotParams::tempVars[2] - round * shiftspeed2) < Configuration::GetInstance()->shiftAxisAngles2[1])
                    {
                        shiftaim2 = Configuration::GetInstance()->shiftAxisAngles2[1];
                    }
                }
                else
                {
                    shiftaim2 = Configuration::GetInstance()->shiftAxisAngles2[1];
                }

                double values[RobotParams::axisNum];
                int ifABS[RobotParams::axisNum];
                for (unsigned int i=0; i<RobotParams::axisNum; ++i)
                {
                    values[i] = 0; ifABS[i] = 0;
                }

                values[0] = brakeaim;
                values[1] = accaim;
                values[2] = 0;
                values[3] = shiftaim1;
                values[4] = shiftaim2;
                ifABS[0] = 1;
                ifABS[1] = 1;
                ifABS[2] = 0;
                ifABS[3] = 1;
                ifABS[4] = 1;

                SendMoveCommandAll(values, ifABS);

                round++;
            }
        }
    }
    else if (examflag == 2)
    {
        if (!startexamtimeflag)
        {
            startexamtimeflag = true;
            gettimeofday(&starttime, NULL);
        }

        if (shiftexampause)
        {
            SendMoveCommand(0,0,0,false,true,false);
            return;
        }

        if (Configuration::GetInstance()->ifManualShift)
        {
            if (RobotParams::currentshiftindex == RobotParams::aimshiftindex)
            {
                examflag = 0;
                startexamtimeflag = false;
                ui->pushButton_shiftrun->setEnabled(true);
                ui->pushButton_shiftpause->setEnabled(false);
                ui->pushButton_run->setEnabled(true);
                ui->pushButton_pause->setEnabled(false);
                ui->pushButton_0to1->setEnabled(true);
                ui->pushButton_1to0->setEnabled(false);
                ui->tab_examclutch->setEnabled(true);
                PRINTF(LOG_INFO, "%s: this shift exam alreay done.\n", __func__);
                SendMoveCommand(0,0,0,false,true,false);

                PRINTF(LOG_ERR, "%s: stamp = %d; mode = %d; %s\n", __func__, timestamp_test, 0, QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
                timestamp_test++;
                return;
            }

            if (RobotParams::shiftrunpointer + 1 == RobotParams::shiftrunlength - 1)
            {
                if (mySCControl->ifreachedshift(false,RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                {
                    RobotParams::shiftrunpointer++;

                    round = 1;

                    gettimeofday(&stoptime, NULL);
                    double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                    ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0));

                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::lastshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer - 1];
                    RobotParams::shiftrunpointer = 0;
                    examflag = 0;
                    startexamtimeflag = false;
                    ui->pushButton_shiftrun->setEnabled(true);
                    ui->pushButton_shiftpause->setEnabled(false);
                    ui->pushButton_run->setEnabled(true);
                    ui->pushButton_pause->setEnabled(false);
                    ui->pushButton_0to1->setEnabled(true);
                    ui->pushButton_1to0->setEnabled(false);
                    ui->tab_examclutch->setEnabled(true);
                    PRINTF(LOG_INFO, "%s: this shift exam finishes.\n", __func__);
                    SendMoveCommand(0,0,0,false,true,false);
                    PRINTF(LOG_ERR, "%s: stamp = %d; mode = %d; %s\n", __func__, timestamp_test, 0, QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
                    timestamp_test++;
                }
                else
                {
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);

                    SendMoveCommand(0,*shiftaims,*(shiftaims + 1),true,false,false);
                    PRINTF(LOG_ERR, "%s: stamp = %d; c#3 = %f; c#4 = %f; a#3 = %f; a#4 = %f; %s\n", __func__, timestamp_test, *shiftaims, *(shiftaims+1),RobotParams::angleRealTime[3], RobotParams::angleRealTime[4], QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
                    timestamp_test++;
                    delete shiftaims;

                    round++;                   
                }
            }
            else
            {
                if (mySCControl->ifreachedshiftprocess(RobotParams::shiftrunpath[RobotParams::shiftrunpointer], RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                {
                    RobotParams::shiftrunpointer++;

                    round = 1;

                    gettimeofday(&stoptime, NULL);
                    double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                    ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0) + QString("|"));

                    RobotParams::lastshiftindex = RobotParams::currentshiftindex;
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);

                    SendMoveCommand(0,*shiftaims,*(shiftaims + 1),true,false,false);
                    PRINTF(LOG_ERR, "%s: stamp = %d; c#3 = %f; c#4 = %f; a#3 = %f; a#4 = %f; %s\n", __func__, timestamp_test, *shiftaims, *(shiftaims+1),RobotParams::angleRealTime[3], RobotParams::angleRealTime[4], QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
                    timestamp_test++;
                    delete shiftaims;

                    round++;
                }
                else
                {
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);

                    SendMoveCommand(0,*shiftaims,*(shiftaims + 1),true,false,false);
                    PRINTF(LOG_ERR, "%s: stamp = %d; c#3 = %f; c#4 = %f; a#3 = %f; a#4 = %f; %s\n", __func__, timestamp_test, *shiftaims, *(shiftaims+1),RobotParams::angleRealTime[3], RobotParams::angleRealTime[4], QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
                    timestamp_test++;
                    delete shiftaims;

                    round++;
                }
            }
        }
        else
        {
            if (RobotParams::currentshiftindex == RobotParams::aimshiftindex)
            {
                examflag = 0;
                startexamtimeflag = false;
                ui->pushButton_shiftrun->setEnabled(true);
                ui->pushButton_shiftpause->setEnabled(false);
                ui->pushButton_run->setEnabled(false);
                ui->pushButton_pause->setEnabled(false);
                ui->pushButton_0to1->setEnabled(false);
                ui->pushButton_1to0->setEnabled(false);
                ui->tab_examclutch->setEnabled(false);
                PRINTF(LOG_INFO, "%s: this shift exam alreay done.\n", __func__);
                SendMoveCommand(0,0,0,false,true,false);
                return;
            }

            if (mySCControl->ifreachedshift(false, RobotParams::aimshiftindex))
            {
                round = 1;

                gettimeofday(&stoptime, NULL);
                double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0));

                RobotParams::lastshiftindex = RobotParams::currentshiftindex;
                RobotParams::currentshiftindex = RobotParams::aimshiftindex;
                RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();

                examflag = 0;
                startexamtimeflag = false;
                ui->pushButton_shiftrun->setEnabled(true);
                ui->pushButton_shiftpause->setEnabled(false);

                PRINTF(LOG_INFO, "%s: this shift exam finishes.\n", __func__);
                SendMoveCommand(0,0,0,false,true,false);
            }
            else
            {
                double *shiftaims = new double[2];
                mySCControl->getconSft(shiftaims, round);

                SendMoveCommand(0,*shiftaims,*(shiftaims + 1),true,false,false);
                delete shiftaims;

                round++;
            }
        }
    }
    else if (examflag == 3)
    {
        if (!startexamtimeflag)
        {
            startexamtimeflag = true;
            gettimeofday(&starttime, NULL);
        }

        if (clutchexampause)
        {
            SendMoveCommand(0,0,0,false,true,false);
            return;
        }

        if (mySCControl->ifreachedclutch(false, 0))
        {
            gettimeofday(&stoptime, NULL);
            double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
            ui->lineEdit_clutchtime->setText(QString::number(timeduring, 'f', 0));

            RobotParams::currentclutchindex = 0;
            RobotParams::currentclutchvalue = "踩下";

            examflag = 0;
            startexamtimeflag = false;
            ui->pushButton_bottom->setEnabled(false);
            ui->pushButton_top->setEnabled(true);
            ui->pushButton_speed->setEnabled(true);
            ui->pushButton_clutchpause->setEnabled(false);
            ui->tab_examshift->setEnabled(true);
            round = 1;
            PRINTF(LOG_INFO, "%s: clutch bottom exam finishes.\n", __func__);
            SendMoveCommand(0,0,0,false,true,false);
            PRINTF(LOG_ERR, "%s: stamp = %d; mode = %d; %s\n", __func__, timestamp_test, 0, QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
            timestamp_test++;
        }
        else
        {
            double *clutchaim = new double();
            mySCControl->getconClh(clutchaim, round);

            SendMoveCommand(*clutchaim,0,0,true,false,true);
            delete clutchaim;

            round++;
            PRINTF(LOG_ERR, "%s: stamp = %d; mode = %d; %s\n", __func__, timestamp_test, examflag, QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
            timestamp_test++;
        }
    }
    else if (examflag == 4)
    {
        if (!startexamtimeflag)
        {
            startexamtimeflag = true;
            gettimeofday(&starttime, NULL);
        }

        if (clutchexampause)
        {
            SendMoveCommand(0,0,0,false,true,false);
            return;
        }

        if (mySCControl->ifreachedclutch(false, 1))
        {
            gettimeofday(&stoptime, NULL);
            double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
            ui->lineEdit_clutchtime->setText(QString::number(timeduring, 'f', 0));

            RobotParams::currentclutchindex = 1;
            RobotParams::currentclutchvalue = "松开";

            examflag = 0;
            startexamtimeflag = false;
            ui->pushButton_bottom->setEnabled(true);
            ui->pushButton_top->setEnabled(false);
            ui->pushButton_speed->setEnabled(false);
            ui->pushButton_clutchpause->setEnabled(false);
            ui->tab_examshift->setEnabled(true);
            round = 1;
            PRINTF(LOG_INFO, "%s: clutch top exam finishes.\n", __func__);
            SendMoveCommand(0,0,0,false,true,false);
            PRINTF(LOG_ERR, "%s: stamp = %d; mode = %d; %s\n", __func__, timestamp_test, 0, QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
            timestamp_test++;
        }
        else
        {
            double *clutchaim = new double();
            mySCControl->getconClh(clutchaim, round);

            SendMoveCommand(*clutchaim,0,0,true,false,true);
            delete clutchaim;

            round++;
            PRINTF(LOG_ERR, "%s: stamp = %d; mode = %d; %s\n", __func__, timestamp_test, examflag, QTime::currentTime().toString("hh:mm:ss:zzz").toStdString().c_str());
            timestamp_test++;
        }
    }
    else if (examflag == 5)
    {
        if (!startexamtimeflag)
        {
            startexamtimeflag = true;
            gettimeofday(&starttime, NULL);
        }

        if (clutchexampause)
        {
            SendMoveCommand(0,0,0,false,true,false);
            return;
        }

        if (mySCControl->ifreachedclutch(false, 1))
        {
            gettimeofday(&stoptime, NULL);
            double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
            ui->lineEdit_clutchtime->setText(QString::number(timeduring, 'f', 0));

            RobotParams::currentclutchindex = 1;
            RobotParams::currentclutchvalue = "松开";

            examflag = 0;
            startexamtimeflag = false;
            ui->pushButton_bottom->setEnabled(true);
            ui->pushButton_top->setEnabled(false);
            ui->pushButton_speed->setEnabled(false);
            ui->pushButton_clutchpause->setEnabled(false);
            ui->tab_examshift->setEnabled(true);
            round = 1;
            PRINTF(LOG_INFO, "%s: clutch speed exam finishes.\n", __func__);
            SendMoveCommand(0,0,0,false,true,false);
        }
        else
        {
            RobotParams::currentclutchindex = 2;
            RobotParams::currentclutchvalue = "抬升";

            double clutchaim = 0;
            if ((clutchaim = Configuration::GetInstance()->clutchAngles[0] - round * Configuration::GetInstance()->clutchUpSpeed) < Configuration::GetInstance()->clutchAngles[1])
            {
                clutchaim = Configuration::GetInstance()->clutchAngles[1];
            }

            SendMoveCommand(clutchaim,0,0,true,false,true);

            round++;
        }

    }
    else if (examflag == 6)
    {
        double values[RobotParams::axisNum];
        int ifABS[RobotParams::axisNum];
        for (unsigned int i=0; i<RobotParams::axisNum; ++i)
        {
            values[i] = 0; ifABS[i] = 0;
        }

        if (exampause)
        {
            SendMoveCommandAll(values, ifABS);
            return;
        }

        if (changeshiftprocess == 0 && RobotParams::currentshiftindex == RobotParams::aimshiftindex)
        {
            examflag = 0;
            startexamtimeflag = false;
            ui->pushButton_shiftrun->setEnabled(true);
            ui->pushButton_shiftpause->setEnabled(false);
            ui->pushButton_run->setEnabled(true);
            ui->pushButton_pause->setEnabled(false);
            ui->pushButton_0to1->setEnabled(true);
            ui->pushButton_1to0->setEnabled(false);
            ui->tab_examclutch->setEnabled(true);
            PRINTF(LOG_INFO, "%s: this exam alreay done.\n", __func__);
            SendMoveCommandAll(values, ifABS);
            return;
        }

        if (changeshiftprocess == 0)
        {
            if (fabs(RobotParams::angleRealTime[0]-Configuration::GetInstance()->limPos[0]) < 1.0 && fabs(RobotParams::angleRealTime[1]-Configuration::GetInstance()->limPos[1]) < 1.0)
            {
                if (!startexamtimeflag)
                {
                    startexamtimeflag = true;
                    gettimeofday(&starttime, NULL);
                }

                round = 1;
                changeshiftprocess = 1;
                values[0] = Configuration::GetInstance()->deathPos[0];
                values[1] = Configuration::GetInstance()->deathPos[1];

                RobotParams::aimclutchindex = 0;
                double *clutchaim = new double();
                mySCControl->getconClh(clutchaim, round);
                values[2] = *clutchaim;
                delete clutchaim;
                round++;

                ifABS[0] = 1; ifABS[1] = 1; ifABS[2] = 1;

                PRINTF(LOG_INFO, "%s: ready to exam.\n", __func__);
                SendMoveCommandAll(values, ifABS);
                return;
            }
            else
            {
                values[0] = Configuration::GetInstance()->limPos[0];
                values[1] = Configuration::GetInstance()->limPos[1];
                ifABS[0] = 1; ifABS[1] = 1;
                SendMoveCommandAll(values, ifABS);
                return;
            }
        }
        else if (changeshiftprocess == 1)
        {
            if (mySCControl->ifreachedclutch(false, 0)&& fabs(RobotParams::angleRealTime[0]-Configuration::GetInstance()->deathPos[0]) < 1.0 && fabs(RobotParams::angleRealTime[1]-Configuration::GetInstance()->deathPos[1]) < 1.0)
            {
                gettimeofday(&stoptime, NULL);
                double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0) + QString("|"));

                RobotParams::currentclutchindex = 0;
                RobotParams::currentclutchvalue = "踩下";

                round = 1;
                changeshiftprocess = 2;

                // 规划路径
                RobotParams::aimshiftindex = ui->comboBox_shiftaim->currentIndex();
                mySCControl->plantrace();

                RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                double *shiftaims = new double[2];
                mySCControl->getconSft(shiftaims, round);
                values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                delete shiftaims;
                round++;

                ifABS[3] = 1; ifABS[4] = 1;

                PRINTF(LOG_INFO, "%s: clutch has been trodden.\n", __func__);
                SendMoveCommandAll(values, ifABS);
                return;
            }
            else
            {
                values[0] = Configuration::GetInstance()->deathPos[0];
                values[1] = Configuration::GetInstance()->deathPos[1];

                RobotParams::aimclutchindex = 0;
                double *clutchaim = new double();
                mySCControl->getconClh(clutchaim, round);
                values[2] = *clutchaim;
                delete clutchaim;
                round++;

                ifABS[0] = 1; ifABS[1] = 1; ifABS[2] = 1;

                SendMoveCommandAll(values, ifABS);
                return;
            }
        }
        else if (changeshiftprocess == 2)
        {
            if (RobotParams::shiftrunpointer + 1 == RobotParams::shiftrunlength - 1)
            {
                if (mySCControl->ifreachedshift(false,RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                {
                    RobotParams::shiftrunpointer++;

                    round = 1;

                    gettimeofday(&stoptime, NULL);
                    double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                    ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0) + QString("|"));

                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::lastshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer - 1];
                    RobotParams::shiftrunpointer = 0;

                    changeshiftprocess = 3;

                    RobotParams::currentclutchindex = 2;
                    RobotParams::currentclutchvalue = "抬升";

                    double clutchaim = 0;
                    if ((clutchaim = Configuration::GetInstance()->clutchAngles[0] - round * Configuration::GetInstance()->clutchUpSpeed) < Configuration::GetInstance()->clutchAngles[1])
                    {
                        clutchaim = Configuration::GetInstance()->clutchAngles[1];
                    }
                    values[2] = clutchaim;
                    round++;

                    ifABS[2] = 1;

                    PRINTF(LOG_INFO, "%s: shift change has been done.\n", __func__);
                    SendMoveCommandAll(values, ifABS);
                    return;
                }
                else
                {
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);
                    values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                    delete shiftaims;
                    round++;

                    ifABS[3] = 1; ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);
                    return;
                }
            }
            else
            {
                if (mySCControl->ifreachedshiftprocess(RobotParams::shiftrunpath[RobotParams::shiftrunpointer], RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                {
                    RobotParams::shiftrunpointer++;

                    round = 1;

                    gettimeofday(&stoptime, NULL);
                    double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                    ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0) + QString("|"));

                    RobotParams::lastshiftindex = RobotParams::currentshiftindex;
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);
                    values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                    delete shiftaims;
                    round++;

                    ifABS[3] = 1; ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);
                    return;
                }
                else
                {
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);
                    values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                    delete shiftaims;
                    round++;

                    ifABS[3] = 1; ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);
                    return;
                }
            }
        }
        else if (changeshiftprocess == 3)
        {
            if (mySCControl->ifreachedclutch(false, 1))
            {
                gettimeofday(&stoptime, NULL);
                double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0));

                RobotParams::currentclutchindex = 1;
                RobotParams::currentclutchvalue = "松开";

                examflag = 0;
                startexamtimeflag = false;
                changeshiftprocess = 0;
                ui->pushButton_shiftrun->setEnabled(true);
                ui->pushButton_shiftpause->setEnabled(false);
                ui->pushButton_run->setEnabled(true);
                ui->pushButton_pause->setEnabled(false);
                ui->pushButton_0to1->setEnabled(true);
                ui->pushButton_1to0->setEnabled(false);
                ui->tab_examclutch->setEnabled(true);
                round = 1;

                PRINTF(LOG_INFO, "%s: exam finishes.\n", __func__);
                SendMoveCommandAll(values, ifABS);
                return;
            }
            else
            {
                RobotParams::currentclutchindex = 2;
                RobotParams::currentclutchvalue = "抬升";

                double clutchaim = 0;
                if ((clutchaim = Configuration::GetInstance()->clutchAngles[0] - round * Configuration::GetInstance()->clutchUpSpeed) < Configuration::GetInstance()->clutchAngles[1])
                {
                    clutchaim = Configuration::GetInstance()->clutchAngles[1];
                }
                values[2] = clutchaim;
                round++;

                ifABS[2] = 1;

                SendMoveCommandAll(values, ifABS);
                return;
            }
        }
        else
        {
            PRINTF(LOG_WARNING, "%s: no such exam process.\n", __func__);
            SendMoveCommandAll(values, ifABS);
            return;
        }
    }
    else if (examflag == 7)
    {
        double values[RobotParams::axisNum];
        int ifABS[RobotParams::axisNum];
        for (unsigned int i=0; i<RobotParams::axisNum; ++i)
        {
            values[i] = 0; ifABS[i] = 0;
        }

        if (changeshiftprocess == 0)
        {
            if (!startexamtimeflag)
            {
                startexamtimeflag = true;
                gettimeofday(&starttime, NULL);
            }

            if (mySCControl->ifreachedclutch(false, 0)&& fabs(RobotParams::angleRealTime[0]-Configuration::GetInstance()->deathPos[0]) < 1.0 && fabs(RobotParams::angleRealTime[1]-Configuration::GetInstance()->deathPos[1]) < 1.0)
            {
                gettimeofday(&stoptime, NULL);
                double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0) + QString("|"));

                RobotParams::currentclutchindex = 0;
                RobotParams::currentclutchvalue = "踩下";

                round = 1;
                changeshiftprocess = 1;

                // 规划路径 切到1挡
                RobotParams::aimshiftindex = 3;
                mySCControl->plantrace();

                RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                double *shiftaims = new double[2];
                mySCControl->getconSft(shiftaims, round);
                values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                delete shiftaims;
                round++;

                ifABS[3] = 1; ifABS[4] = 1;

                PRINTF(LOG_INFO, "%s: clutch has been trodden.\n", __func__);
                SendMoveCommandAll(values, ifABS);
                return;
            }
            else
            {
                values[0] = Configuration::GetInstance()->deathPos[0];
                values[1] = Configuration::GetInstance()->deathPos[1];

                RobotParams::aimclutchindex = 0;
                double *clutchaim = new double();
                mySCControl->getconClh(clutchaim, round);
                values[2] = *clutchaim;
                delete clutchaim;
                round++;

                ifABS[0] = 1; ifABS[1] = 1; ifABS[2] = 1;

                SendMoveCommandAll(values, ifABS);
                return;
            }
        }
        else if (changeshiftprocess == 1)
        {
            if (RobotParams::shiftrunpointer + 1 == RobotParams::shiftrunlength - 1)
            {
                if (mySCControl->ifreachedshift(false,RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                {
                    RobotParams::shiftrunpointer++;

                    round = 1;

                    gettimeofday(&stoptime, NULL);
                    double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                    ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0) + QString("|"));

                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::lastshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer - 1];
                    RobotParams::shiftrunpointer = 0;

                    changeshiftprocess = 2;

                    RobotParams::currentclutchindex = 2;
                    RobotParams::currentclutchvalue = "抬升";

                    double clutchaim = 0, accaim = 0;
                    if ((clutchaim = Configuration::GetInstance()->clutchAngles[0] - round * Configuration::GetInstance()->clutchUpSpeed) < Configuration::GetInstance()->clutchAngles[1])
                    {
                        clutchaim = Configuration::GetInstance()->clutchAngles[1];
                    }
                    values[2] = clutchaim;

                    if ( ( accaim = Configuration::GetInstance()->deathPos[1] + round * ( Configuration::GetInstance()->clutchUpSpeed / (Configuration::GetInstance()->clutchAngles[0] - Configuration::GetInstance()->clutchAngles[1]) * (Configuration::GetInstance()->startAccAngleValue - Configuration::GetInstance()->deathPos[1]) ) ) > Configuration::GetInstance()->startAccAngleValue)
                    {
                        accaim = Configuration::GetInstance()->startAccAngleValue;
                    }
                    values[1] = accaim;
                    ifABS[1] = 1;
                    ifABS[2] = 1;

                    round++;

                    PRINTF(LOG_INFO, "%s: shift change has been done.\n", __func__);
                    SendMoveCommandAll(values, ifABS);
                    return;
                }
                else
                {
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);
                    values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                    delete shiftaims;
                    round++;

                    ifABS[3] = 1; ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);
                    return;
                }
            }
            else
            {
                if (mySCControl->ifreachedshiftprocess(RobotParams::shiftrunpath[RobotParams::shiftrunpointer], RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                {
                    RobotParams::shiftrunpointer++;

                    round = 1;

                    gettimeofday(&stoptime, NULL);
                    double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                    ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0) + QString("|"));

                    RobotParams::lastshiftindex = RobotParams::currentshiftindex;
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);
                    values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                    delete shiftaims;
                    round++;

                    ifABS[3] = 1; ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);
                    return;
                }
                else
                {
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);
                    values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                    delete shiftaims;
                    round++;

                    ifABS[3] = 1; ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);
                    return;
                }
            }
        }
        else if (changeshiftprocess == 2)
        {
            if (mySCControl->ifreachedclutch(false, 1)&& fabs(RobotParams::angleRealTime[1] - Configuration::GetInstance()->startAccAngleValue) < 1.0)
            {
                gettimeofday(&stoptime, NULL);
                double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0));

                RobotParams::currentclutchindex = 1;
                RobotParams::currentclutchvalue = "松开";

                examflag = 0;
                startexamtimeflag = false;
                changeshiftprocess = 0;
                ui->pushButton_shiftrun->setEnabled(false);
                ui->pushButton_shiftpause->setEnabled(false);
                ui->pushButton_run->setEnabled(false);
                ui->pushButton_pause->setEnabled(false);
                ui->pushButton_0to1->setEnabled(false);
                ui->pushButton_1to0->setEnabled(true);
                ui->tab_examclutch->setEnabled(false);
                round = 1;

                PRINTF(LOG_INFO, "%s: car start finishes.\n", __func__);
                SendMoveCommandAll(values, ifABS);
                return;
            }
            else
            {
                RobotParams::currentclutchindex = 2;
                RobotParams::currentclutchvalue = "抬升";

                double clutchaim = 0, accaim = 0;
                if ((clutchaim = Configuration::GetInstance()->clutchAngles[0] - round * Configuration::GetInstance()->clutchUpSpeed) < Configuration::GetInstance()->clutchAngles[1])
                {
                    clutchaim = Configuration::GetInstance()->clutchAngles[1];
                }
                values[2] = clutchaim;

                if ( ( accaim = Configuration::GetInstance()->deathPos[1] + round * (Configuration::GetInstance()->clutchUpSpeed / (Configuration::GetInstance()->clutchAngles[0] - Configuration::GetInstance()->clutchAngles[1]) * (Configuration::GetInstance()->startAccAngleValue - Configuration::GetInstance()->deathPos[1]) ) ) > Configuration::GetInstance()->startAccAngleValue)
                {
                    accaim = Configuration::GetInstance()->startAccAngleValue;
                }
                values[1] = accaim;
                ifABS[1] = 1;
                ifABS[2] = 1;

                round++;

                SendMoveCommandAll(values, ifABS);
                return;
            }
        }
        else
        {
            PRINTF(LOG_WARNING, "%s: no such exam process.\n", __func__);
            SendMoveCommandAll(values, ifABS);
            return;
        }
    }
    else if (examflag == 8)
    {
        double values[RobotParams::axisNum];
        int ifABS[RobotParams::axisNum];
        for (unsigned int i=0; i<RobotParams::axisNum; ++i)
        {
            values[i] = 0; ifABS[i] = 0;
        }

        if (changeshiftprocess == 0)
        {
            if (!startexamtimeflag)
            {
                startexamtimeflag = true;
                gettimeofday(&starttime, NULL);
            }

            if (mySCControl->ifreachedclutch(false, 0)&& fabs(RobotParams::angleRealTime[0]-Configuration::GetInstance()->deathPos[0]) < 1.0 && fabs(RobotParams::angleRealTime[1]-Configuration::GetInstance()->deathPos[1]) < 1.0)
            {
                gettimeofday(&stoptime, NULL);
                double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0) + QString("|"));

                RobotParams::currentclutchindex = 0;
                RobotParams::currentclutchvalue = "踩下";

                round = 1;
                changeshiftprocess = 1;

                // 规划路径 切到N挡
                RobotParams::aimshiftindex = 1;
                mySCControl->plantrace();

                RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                double *shiftaims = new double[2];
                mySCControl->getconSft(shiftaims, round);
                values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                delete shiftaims;
                round++;

                ifABS[3] = 1; ifABS[4] = 1;

                PRINTF(LOG_INFO, "%s: clutch has been trodden.\n", __func__);
                SendMoveCommandAll(values, ifABS);
                return;
            }
            else
            {
                values[0] = Configuration::GetInstance()->deathPos[0];
                values[1] = Configuration::GetInstance()->deathPos[1];

                RobotParams::aimclutchindex = 0;
                double *clutchaim = new double();
                mySCControl->getconClh(clutchaim, round);
                values[2] = *clutchaim;
                delete clutchaim;
                round++;

                ifABS[0] = 1; ifABS[1] = 1; ifABS[2] = 1;

                SendMoveCommandAll(values, ifABS);
                return;
            }
        }
        else if (changeshiftprocess == 1)
        {
            if (RobotParams::shiftrunpointer + 1 == RobotParams::shiftrunlength - 1)
            {
                if (mySCControl->ifreachedshift(false,RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                {
                    RobotParams::shiftrunpointer++;

                    round = 1;

                    gettimeofday(&stoptime, NULL);
                    double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                    ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0) + QString("|"));

                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::lastshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer - 1];
                    RobotParams::shiftrunpointer = 0;

                    changeshiftprocess = 2;

                    RobotParams::currentclutchindex = 2;
                    RobotParams::currentclutchvalue = "抬升";

                    double clutchaim = 0, brakeaim = 0;
                    if ((clutchaim = Configuration::GetInstance()->clutchAngles[0] - round * Configuration::GetInstance()->clutchUpSpeed) < Configuration::GetInstance()->clutchAngles[1])
                    {
                        clutchaim = Configuration::GetInstance()->clutchAngles[1];
                    }
                    values[2] = clutchaim;

                    double brakespeed = 1.0;
                    if ( ( brakeaim = Configuration::GetInstance()->deathPos[0] + round * brakespeed ) > Configuration::GetInstance()->brakeThetaAfterGoHome)
                    {
                        brakeaim = Configuration::GetInstance()->brakeThetaAfterGoHome;
                    }
                    values[0] = brakeaim;
                    ifABS[0] = 1;
                    ifABS[2] = 1;

                    round++;

                    PRINTF(LOG_INFO, "%s: shift change has been done.\n", __func__);
                    SendMoveCommandAll(values, ifABS);
                    return;
                }
                else
                {
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);
                    values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                    delete shiftaims;
                    round++;

                    ifABS[3] = 1; ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);
                    return;
                }
            }
            else
            {
                if (mySCControl->ifreachedshiftprocess(RobotParams::shiftrunpath[RobotParams::shiftrunpointer], RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                {
                    RobotParams::shiftrunpointer++;

                    round = 1;

                    gettimeofday(&stoptime, NULL);
                    double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                    ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0) + QString("|"));

                    RobotParams::lastshiftindex = RobotParams::currentshiftindex;
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);
                    values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                    delete shiftaims;
                    round++;

                    ifABS[3] = 1; ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);
                    return;
                }
                else
                {
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);
                    values[3] = *shiftaims; values[4] = *(shiftaims + 1);
                    delete shiftaims;
                    round++;

                    ifABS[3] = 1; ifABS[4] = 1;

                    SendMoveCommandAll(values, ifABS);
                    return;
                }
            }
        }
        else if (changeshiftprocess == 2)
        {
            if (mySCControl->ifreachedclutch(false, 1)&& fabs(RobotParams::angleRealTime[0] - Configuration::GetInstance()->brakeThetaAfterGoHome) < 1.0)
            {
                gettimeofday(&stoptime, NULL);
                double timeduring = (stoptime.tv_sec-starttime.tv_sec)*1000.0 + (stoptime.tv_usec-starttime.tv_usec)/1000.0;
                ui->lineEdit_shifttime->setText(ui->lineEdit_shifttime->text() + QString::number(timeduring, 'f', 0));

                RobotParams::currentclutchindex = 1;
                RobotParams::currentclutchvalue = "松开";

                examflag = 0;
                startexamtimeflag = false;
                changeshiftprocess = 0;
                ui->pushButton_shiftrun->setEnabled(true);
                ui->pushButton_shiftpause->setEnabled(false);
                ui->pushButton_run->setEnabled(true);
                ui->pushButton_pause->setEnabled(false);
                ui->pushButton_0to1->setEnabled(true);
                ui->pushButton_1to0->setEnabled(false);
                ui->tab_examclutch->setEnabled(true);
                round = 1;

                PRINTF(LOG_INFO, "%s: car start stops.\n", __func__);
                SendMoveCommandAll(values, ifABS);
                return;
            }
            else
            {
                RobotParams::currentclutchindex = 2;
                RobotParams::currentclutchvalue = "抬升";

                double clutchaim = 0, brakeaim = 0;
                if ((clutchaim = Configuration::GetInstance()->clutchAngles[0] - round * Configuration::GetInstance()->clutchUpSpeed) < Configuration::GetInstance()->clutchAngles[1])
                {
                    clutchaim = Configuration::GetInstance()->clutchAngles[1];
                }
                values[2] = clutchaim;

                double brakespeed = 1.0;
                if ( ( brakeaim = Configuration::GetInstance()->deathPos[0] + round * brakespeed ) > Configuration::GetInstance()->brakeThetaAfterGoHome)
                {
                    brakeaim = Configuration::GetInstance()->brakeThetaAfterGoHome;
                }
                values[0] = brakeaim;
                ifABS[0] = 1;
                ifABS[2] = 1;

                round++;

                SendMoveCommandAll(values, ifABS);
                return;
            }
        }
        else
        {
            PRINTF(LOG_WARNING, "%s: no such exam process.\n", __func__);
            SendMoveCommandAll(values, ifABS);
            return;
        }
    }
    else if (examflag == 9)
    {
        if (Configuration::GetInstance()->ifManualShift)
        {
            if (RobotParams::shiftrunpointer + 1 == RobotParams::shiftrunlength - 1)
            {
                if (mySCControl->ifreachedshift(false,RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                {
                    RobotParams::shiftrunpointer++;

                    round = 1;

                    if (mySCControl->ifreachedclutch(false, 1))
                    {
                        RobotParams::currentshiftindex = 1;
                        RobotParams::currentshiftvalue = "N_3&4";
                        RobotParams::currentclutchindex = 1;
                        RobotParams::currentclutchvalue = "松开";

                        shiftexampause = false;
                        clutchexampause = false;
                        exampause = false;
                        examflag = 0;
                        AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
                        round = 1;
                        round2 = 1;
                        PRINTF(LOG_INFO, "%s: exam stops.\n", __func__);
                        examtimer->stop();

                        // 退出测试标志位
                        RobotParams::isExaming = false;
                    }
                    else
                    {
                        RobotParams::shiftrunpointer--;

                        double *clutchaim = new double();
                        mySCControl->getconClh(clutchaim, round2);

                        SendMoveCommand(*clutchaim,0,0,true,false,true);
                        delete clutchaim;

                        round2++;
                    }
                }
                else
                {
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);

                    double *clutchaim = new double();
                    mySCControl->getconClh(clutchaim, round2);

                    SendMoveCommand(*clutchaim,*shiftaims,*(shiftaims + 1),true,true,false);
                    delete clutchaim;
                    delete shiftaims;

                    round++;
                    round2++;
                }
            }
            else
            {
                if (mySCControl->ifreachedshiftprocess(RobotParams::shiftrunpath[RobotParams::shiftrunpointer], RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1]))
                {
                    RobotParams::shiftrunpointer++;

                    round = 1;

                    RobotParams::lastshiftindex = RobotParams::currentshiftindex;
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);

                    double *clutchaim = new double();
                    mySCControl->getconClh(clutchaim, round2);

                    SendMoveCommand(*clutchaim,*shiftaims,*(shiftaims + 1),true,true,false);
                    delete clutchaim;
                    delete shiftaims;

                    round++;
                    round2++;
                }
                else
                {
                    RobotParams::currentshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer];
                    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
                    RobotParams::aimshiftindex = RobotParams::shiftrunpath[RobotParams::shiftrunpointer + 1];

                    double *shiftaims = new double[2];
                    mySCControl->getconSft(shiftaims, round);

                    double *clutchaim = new double();
                    mySCControl->getconClh(clutchaim, round2);

                    SendMoveCommand(*clutchaim,*shiftaims,*(shiftaims + 1),true,true,false);
                    delete clutchaim;
                    delete shiftaims;

                    round++;
                    round2++;
                }
            }
        }
        else
        {
            if (mySCControl->ifreachedshift(false,RobotParams::aimshiftindex))
            {
                round = 1;

                RobotParams::currentshiftindex = 1;
                RobotParams::currentshiftvalue = "N";
                RobotParams::currentclutchindex = 1;
                RobotParams::currentclutchvalue = "自动";

                shiftexampause = false;
                clutchexampause = false;
                exampause = false;
                examflag = 0;
                AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
                round2 = 1;
                PRINTF(LOG_INFO, "%s: exam stops.\n", __func__);
                examtimer->stop();

                // 退出测试标志位
                RobotParams::isExaming = false;
            }
            else
            {
                double *shiftaims = new double[2];
                mySCControl->getconSft(shiftaims, round);

                SendMoveCommand(0,*shiftaims,*(shiftaims + 1),true,false,false);
                delete shiftaims;

                round++;
            }
        }
    }
    else
    {
        PRINTF(LOG_WARNING, "%s: (%d) state does not exist in exam.\n", __func__, examflag);
    }
}

void ShiftClutchUI::on_comboBox_way_currentIndexChanged(int index)
{
    if (ifenablewaychangedeventhappen)
    {
        ui->checkBox_autoset->setChecked(false);
        RobotParams::ifConfirmSC = false;
        Configuration::GetInstance()->ifAutoRecordMidN = false;

        if (index == 0)
        {
            Configuration::GetInstance()->ifManualShift = true;

            // 改变图像
            QPixmapCache::clear();
            const QString picpath = QString::fromStdString(Configuration::mainFolder) + "/manualshift.png";
            pic.load(picpath);
            ui->label_pic->setPixmap(pic);

            // 设定离合界面
            ui->tab_clutch->setEnabled(true);

            // 设定测试界面
            ui->tab_exam->setEnabled(true);
            ui->pushButton_run->setEnabled(true);
            ui->pushButton_pause->setEnabled(true);
            ui->pushButton_0to1->setEnabled(true);
            ui->pushButton_1to0->setEnabled(true);
            ui->tab_examclutch->setEnabled(true);

            // 设定换挡时刻界面
            ui->tab_changeshift->setEnabled(true);
            if (Configuration::GetInstance()->pedalRobotUsage == 0)
            {
                ui->lineEdit_1to2->setEnabled(false);
                ui->lineEdit_2to3->setEnabled(false);
                ui->lineEdit_3to4->setEnabled(false);
                ui->lineEdit_4to5->setEnabled(false);
                ui->lineEdit_5to4->setEnabled(false);
                ui->lineEdit_4to3->setEnabled(false);
                ui->lineEdit_3to2->setEnabled(false);
                ui->lineEdit_atoN->setEnabled(false);

                ui->tabWidget_NW->setCurrentIndex(0);
                ui->plotwidget->clearGraphs();
                ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
                ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
                ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
                ui->plotwidget->xAxis->setRange(0,1800);
                ui->plotwidget->yAxis->setRange(-5, 140);

                RobotParams::ifConfirmCS = false;
            }
            else if (Configuration::GetInstance()->pedalRobotUsage == 1)
            {
                ui->lineEdit_1to2->setEnabled(true);
                ui->lineEdit_2to3->setEnabled(true);
                ui->lineEdit_3to4->setEnabled(true);
                ui->lineEdit_4to5->setEnabled(true);
                ui->lineEdit_5to4->setEnabled(true);
                ui->lineEdit_4to3->setEnabled(true);
                ui->lineEdit_3to2->setEnabled(true);
                ui->lineEdit_atoN->setEnabled(true);

                ui->tabWidget_NW->setCurrentIndex(1);
                ui->plotwidget->clearGraphs();
                ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
                ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
                ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
                ui->plotwidget->xAxis->setRange(0,1800);
                ui->plotwidget->yAxis->setRange(-5, 140);

                RobotParams::ifConfirmCS = false;
            }

            // 设置自动补齐可用性
            ui->checkBox_autoset->setEnabled(true);
        }
        else
        {
            Configuration::GetInstance()->ifManualShift = false;

            // 改变图像
            QPixmapCache::clear();
            const QString picpath = QString::fromStdString(Configuration::mainFolder) + "/autoshift.png";
            pic.load(picpath);
            ui->label_pic->setPixmap(pic);

            // 设定离合界面
            ui->tab_clutch->setEnabled(false);

            // 设定测试界面
            ui->tab_exam->setEnabled(true);
            ui->pushButton_run->setEnabled(false);
            ui->pushButton_pause->setEnabled(false);
            ui->pushButton_0to1->setEnabled(false);
            ui->pushButton_1to0->setEnabled(false);
            ui->tab_examclutch->setEnabled(false);

            // 设定换挡时刻界面
            ui->tab_changeshift->setEnabled(false);

            // 设置自动补齐可用性
            ui->checkBox_autoset->setEnabled(false);
        }

        // 重置comboBox
        resetcomboBox();

        // 重置list
        resetlist();
    }
}

void ShiftClutchUI::on_checkBox_zero_stateChanged(int arg1)
{
    if (ifenablebackzeroeventhappen)
    {
        RobotParams::ifConfirmSC = false;

        if (arg1)
        {
            Configuration::GetInstance()->ifGoBack = true;
        }
        else
        {
            Configuration::GetInstance()->ifGoBack = false;
        }
    }
}

void ShiftClutchUI::on_pushButton_save_clicked()
{
    if(Configuration::GetInstance()->SaveToFile()==0){
        QMessageBox::information(this,"提示",QObject::tr("设置保存成功"));
    }else{
        QMessageBox::information(this,"提示",QObject::tr("!!!设置保存失败!!!"));
    }
}

void ShiftClutchUI::on_pushButton_read_clicked()
{
    const QString txtpath = QString::fromStdString(Configuration::carTypeFilePath);
    QString fileName = QFileDialog::getOpenFileName(this, tr("读取"), txtpath, tr("XML Files(*.xml)"));
    if (fileName == "") return;
    std::string fn = NormalFile::GetFileName(fileName.toStdString().c_str());

    std::string tempcarname = Configuration::GetInstance()->carTypeName;
    Configuration::GetInstance()->carTypeName = fn;

    if(Configuration::GetInstance()->ReadFromFile() == 0){
        QMessageBox::information( this,"提示", tr( (QString("读取").toStdString() + Configuration::GetInstance()->carTypeName + QString("成功").toStdString()).c_str() ) );
        RobotParams::ifConfirmSC = false;
        haveReadXML = true;
    }else{
        QMessageBox::information(this,"提示", tr( (QString("!!!读取").toStdString() + Configuration::GetInstance()->carTypeName + QString("失败!!!").toStdString()).c_str() ) );
        Configuration::GetInstance()->carTypeName = tempcarname;
        return;
    }

    resetcomboBox();
    initiallist();

    ui->lineEdit_type->setText( QString::fromStdString( Configuration::GetInstance()->carTypeName.substr(0, Configuration::GetInstance()->carTypeName.length() - 4) ) );
    ifenablewaychangedeventhappen = false;
    ui->comboBox_way->setCurrentIndex(!Configuration::GetInstance()->ifManualShift);
    ifenablewaychangedeventhappen =  true;
    ifenablebackzeroeventhappen = false;
    ui->checkBox_zero->setChecked(Configuration::GetInstance()->ifGoBack);
    ifenablebackzeroeventhappen = true;

    QPixmapCache::clear();

    ui->checkBox_autoset->setEnabled(Configuration::GetInstance()->ifAutoRecordMidN);

    if (Configuration::GetInstance()->ifManualShift)
    {
        const QString picpath = QString::fromStdString(Configuration::mainFolder) + "/manualshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(true);

        ui->tab_exam->setEnabled(true);
        ui->pushButton_run->setEnabled(true);
        ui->pushButton_pause->setEnabled(true);
        ui->pushButton_0to1->setEnabled(true);
        ui->pushButton_1to0->setEnabled(true);
        ui->tab_examclutch->setEnabled(true);

        ui->tab_changeshift->setEnabled(true);
        if (Configuration::GetInstance()->pedalRobotUsage == 0)
        {
            ui->lineEdit_1to2->setEnabled(false);
            ui->lineEdit_2to3->setEnabled(false);
            ui->lineEdit_3to4->setEnabled(false);
            ui->lineEdit_4to5->setEnabled(false);
            ui->lineEdit_5to4->setEnabled(false);
            ui->lineEdit_4to3->setEnabled(false);
            ui->lineEdit_3to2->setEnabled(false);
            ui->lineEdit_atoN->setEnabled(false);

            ui->tabWidget_NW->setCurrentIndex(0);
            ui->plotwidget->clearGraphs();
            ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
            ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
            ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
            ui->plotwidget->xAxis->setRange(0,1800);
            ui->plotwidget->yAxis->setRange(-5, 140);

            RobotParams::ifConfirmCS = false;
        }
        else if (Configuration::GetInstance()->pedalRobotUsage == 1)
        {
            ui->lineEdit_1to2->setEnabled(true);
            ui->lineEdit_2to3->setEnabled(true);
            ui->lineEdit_3to4->setEnabled(true);
            ui->lineEdit_4to5->setEnabled(true);
            ui->lineEdit_5to4->setEnabled(true);
            ui->lineEdit_4to3->setEnabled(true);
            ui->lineEdit_3to2->setEnabled(true);
            ui->lineEdit_atoN->setEnabled(true);

            ui->tabWidget_NW->setCurrentIndex(1);
            ui->plotwidget->clearGraphs();
            ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
            ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
            ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
            ui->plotwidget->xAxis->setRange(0,1800);
            ui->plotwidget->yAxis->setRange(-5, 140);

            RobotParams::ifConfirmCS = false;
        }

        ui->checkBox_autoset->setEnabled(true);
    }
    else
    {
        const QString picpath = QString::fromStdString(Configuration::mainFolder) + "/autoshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(false);

        ui->tab_exam->setEnabled(true);
        ui->pushButton_run->setEnabled(false);
        ui->pushButton_pause->setEnabled(false);
        ui->pushButton_0to1->setEnabled(false);
        ui->pushButton_1to0->setEnabled(false);
        ui->tab_examclutch->setEnabled(false);

        ui->tab_changeshift->setEnabled(false);

        ui->checkBox_autoset->setEnabled(false);
    }
}

void ShiftClutchUI::on_pushButton_record1_clicked()
{
    RobotParams::ifConfirmSC = false;

    const int jg = ui->comboBox_shift->currentIndex();
    Configuration::GetInstance()->shiftAxisAngles1[jg] = RobotParams::angleRealTime[3];
    Configuration::GetInstance()->shiftAxisAngles2[jg] = RobotParams::angleRealTime[4];

    if (Configuration::GetInstance()->ifManualShift)
    {
        switch (jg)
        {
        case 0:
            if (Configuration::GetInstance()->ifAutoRecordMidN)
            {
                QMessageBox::information(NULL, tr("提示"), tr("该挡位会自动补齐！"));
            }
            else
            {
                ui->list1->item(jg + 1)->setText("N_1&2\t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            }
            break;
        case 1:
            ui->list1->item(jg + 1)->setText("N_3&4\t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        case 2:
            if (Configuration::GetInstance()->ifAutoRecordMidN)
            {
                QMessageBox::information(NULL, tr("提示"), tr("该挡位会自动补齐！"));
            }
            else
            {
                ui->list1->item(jg + 1)->setText("N_5&6\t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            }
            break;
        case 3:
            ui->list1->item(jg + 1)->setText("1    \t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        case 4:
            ui->list1->item(jg + 1)->setText("2    \t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        case 5:
            ui->list1->item(jg + 1)->setText("3    \t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        case 6:
            ui->list1->item(jg + 1)->setText("4    \t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        case 7:
            ui->list1->item(jg + 1)->setText("5    \t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        case 8:
            ui->list1->item(jg + 1)->setText("6    \t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        default:
            break;
        }

        // 自动补齐
        if (Configuration::GetInstance()->ifAutoRecordMidN)
        {
            Configuration::GetInstance()->shiftAxisAngles1[0] = 0.5 * ( Configuration::GetInstance()->shiftAxisAngles1[3] + Configuration::GetInstance()->shiftAxisAngles1[4] );
            Configuration::GetInstance()->shiftAxisAngles2[0] = Configuration::GetInstance()->shiftAxisAngles2[1];
            Configuration::GetInstance()->shiftAxisAngles1[2] = Configuration::GetInstance()->shiftAxisAngles1[7];
            Configuration::GetInstance()->shiftAxisAngles2[2] = Configuration::GetInstance()->shiftAxisAngles2[1];

            ui->list1->item(1)->setText("N_1&2\t\t\t\t " + QString::number(Configuration::GetInstance()->shiftAxisAngles1[0], 'f', 2) + "\t\t\t\t " + QString::number(Configuration::GetInstance()->shiftAxisAngles2[0], 'f', 2));
            ui->list1->item(3)->setText("N_5&6\t\t\t\t " + QString::number(Configuration::GetInstance()->shiftAxisAngles1[2], 'f', 2) + "\t\t\t\t " + QString::number(Configuration::GetInstance()->shiftAxisAngles2[2], 'f', 2));
        }
    }
    else
    {
        switch (jg)
        {
        case 0:
            ui->list1->item(jg + 1)->setText("P    \t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        case 1:
            ui->list1->item(jg + 1)->setText("N    \t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        case 2:
            ui->list1->item(jg + 1)->setText("D    \t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        default:
            break;
        }
    }
}

void ShiftClutchUI::on_pushButton_reset1_clicked()
{
    RobotParams::ifConfirmSC = false;

    const int jg = ui->comboBox_shift->currentIndex();
    Configuration::GetInstance()->shiftAxisAngles1[jg] = 0.0;
    Configuration::GetInstance()->shiftAxisAngles2[jg] = 0.0;

    if (Configuration::GetInstance()->ifManualShift)
    {
        switch (jg)
        {
        case 0:
            ui->list1->item(jg + 1)->setText("N_1&2\t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        case 1:
            ui->list1->item(jg + 1)->setText("N_3&4\t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        case 2:
            ui->list1->item(jg + 1)->setText("N_5&6\t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        case 3:
            ui->list1->item(jg + 1)->setText("1    \t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        case 4:
            ui->list1->item(jg + 1)->setText("2    \t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        case 5:
            ui->list1->item(jg + 1)->setText("3    \t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        case 6:
            ui->list1->item(jg + 1)->setText("4    \t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        case 7:
            ui->list1->item(jg + 1)->setText("5    \t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        case 8:
            ui->list1->item(jg + 1)->setText("6    \t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        default:
            break;
        }
    }
    else
    {
        switch (jg)
        {
        case 0:
            ui->list1->item(jg + 1)->setText("P    \t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        case 1:
            ui->list1->item(jg + 1)->setText("N    \t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        case 2:
            ui->list1->item(jg + 1)->setText("D    \t\t\t\t 00.00\t\t\t\t 00.00");
            break;
        default:
            break;
        }
    }
}

void ShiftClutchUI::on_comboBox_clutch_currentIndexChanged(int index)
{
    if (index == 2)
    {
        ui->lineEdit_speed->setEnabled(true);
    }
    else
    {
        ui->lineEdit_speed->setEnabled(false);
    }
}

void ShiftClutchUI::on_pushButton_record2_clicked()
{
    RobotParams::ifConfirmSC = false;

    const int jg = ui->comboBox_clutch->currentIndex();

    switch (jg)
    {
    case 0:
        Configuration::GetInstance()->clutchAngles[0] = RobotParams::angleRealTime[2];
        ui->list2->item(jg + 1)->setText("踩住位置\t\t\t\t\t\t " + QString::number(RobotParams::angleRealTime[2], 'f', 2) + "\t\t\t\t\t\t / ");
        break;
    case 1:
        Configuration::GetInstance()->clutchAngles[1] = RobotParams::angleRealTime[2];
        ui->list2->item(jg + 1)->setText("松开位置\t\t\t\t\t\t " + QString::number(RobotParams::angleRealTime[2], 'f', 2) + "\t\t\t\t\t\t / ");
        break;
    case 2:
        Configuration::GetInstance()->clutchUpSpeed = ui->lineEdit_speed->text().trimmed().toDouble();
        ui->list2->item(jg + 1)->setText("松开速度\t\t\t\t\t\t /    \t\t\t\t\t\t " + QString::number(Configuration::GetInstance()->clutchUpSpeed, 'f', 2));
        break;
    default:
        break;
    }
}

void ShiftClutchUI::on_pushButton_reset2_clicked()
{
    RobotParams::ifConfirmSC = false;

    const int jg = ui->comboBox_clutch->currentIndex();

    switch (jg)
    {
    case 0:
        Configuration::GetInstance()->clutchAngles[0] = 0.0;
        ui->list2->item(jg + 1)->setText("踩住位置\t\t\t\t\t\t 00.00\t\t\t\t\t\t / ");
        break;
    case 1:
        Configuration::GetInstance()->clutchAngles[1] = 0.0;
        ui->list2->item(jg + 1)->setText("松开位置\t\t\t\t\t\t 00.00\t\t\t\t\t\t / ");
        break;
    case 2:
        Configuration::GetInstance()->clutchUpSpeed = 0.0;
        ui->list2->item(jg + 1)->setText("松开速度\t\t\t\t\t\t /    \t\t\t\t\t\t 0.00");
        break;
    default:
        break;
    }
}

void ShiftClutchUI::on_pushButton_startexam_clicked()
{
    // 检查挡位规范性
    bool ifshiftcanbeused = true;

    if (Configuration::GetInstance()->ifManualShift)
    {
        if ( fabs(Configuration::GetInstance()->shiftAxisAngles1[0] - Configuration::GetInstance()->shiftAxisAngles1[3]) > Configuration::GetInstance()->angleErr_A[1] )
        {
            ifshiftcanbeused = false;
        }
        if ( fabs(Configuration::GetInstance()->shiftAxisAngles1[0] - Configuration::GetInstance()->shiftAxisAngles1[4]) > Configuration::GetInstance()->angleErr_A[1] )
        {
            ifshiftcanbeused = false;
        }

        if ( fabs(Configuration::GetInstance()->shiftAxisAngles1[1] - Configuration::GetInstance()->shiftAxisAngles1[5]) > Configuration::GetInstance()->angleErr_A[1] )
        {
            ifshiftcanbeused = false;
        }
        if ( fabs(Configuration::GetInstance()->shiftAxisAngles1[1] - Configuration::GetInstance()->shiftAxisAngles1[6]) > Configuration::GetInstance()->angleErr_A[1] )
        {
            ifshiftcanbeused = false;
        }

        if ( fabs(Configuration::GetInstance()->shiftAxisAngles1[2] - Configuration::GetInstance()->shiftAxisAngles1[7]) > Configuration::GetInstance()->angleErr_A[1] )
        {
            ifshiftcanbeused = false;
        }

        if ( fabs(Configuration::GetInstance()->shiftAxisAngles2[1] - Configuration::GetInstance()->shiftAxisAngles2[0]) > Configuration::GetInstance()->angleErr_A[2] )
        {
            ifshiftcanbeused = false;
        }
        if ( fabs(Configuration::GetInstance()->shiftAxisAngles2[1] - Configuration::GetInstance()->shiftAxisAngles2[2]) > Configuration::GetInstance()->angleErr_A[2] )
        {
            ifshiftcanbeused = false;
        }
    }
    else
    {
        if ( fabs(Configuration::GetInstance()->shiftAxisAngles1[1] - Configuration::GetInstance()->shiftAxisAngles1[0]) > Configuration::GetInstance()->angleErr_A[1] )
        {
            ifshiftcanbeused = false;
        }
        if ( fabs(Configuration::GetInstance()->shiftAxisAngles1[1] - Configuration::GetInstance()->shiftAxisAngles1[2]) > Configuration::GetInstance()->angleErr_A[1] )
        {
            ifshiftcanbeused = false;
        }
    }

    if (!ifshiftcanbeused)
    {
        QMessageBox::warning(NULL, tr("警告"), tr("挡位之间差别过大\r\n请手动调整！"));
        return;
    }

    if (!RobotParams::ifConfirmSC)
    {
        QMessageBox::warning(NULL, tr("警告"), tr("请确认挡位信息后再进行测试！"));
        return;
    }

    int ret = QMessageBox::information(NULL, tr("提示"), tr("请确认所有挡位离合信息采集完毕，\r\n再把挡位拨到空挡附近！"), tr("确认"), tr("取消"));
    if(ret == 1)
    {
        return;
    }

    // 保证在空挡 手动和自动的空挡index相同
    if (!mySCControl->ifreachedshift(true, 1))
    {
        QMessageBox::warning(NULL, tr("警告"), tr("挡位不在空挡附近\r\n请手动调整！"));
        return;
    }

    if (Configuration::GetInstance()->ifManualShift)
    {
        RobotParams::currentshiftindex = 1;
        RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
        RobotParams::lastshiftindex = 1;
        RobotParams::aimshiftindex = 3;

        RobotParams::shiftrunpath[0] = 1;
        RobotParams::shiftrunpath[1] = 0;
        RobotParams::shiftrunpath[2] = 3;
        RobotParams::shiftrunlength = 3;
        RobotParams::shiftrunpointer = 0;

        RobotParams::currentclutchindex = 1;
        RobotParams::currentclutchvalue = "松开";

        RobotParams::aimclutchindex = 0;

        shiftexampause = false;
        clutchexampause = false;
        exampause = false;

        ui->pushButton_shiftpause->setText(" 暂 停 ");
        ui->pushButton_pause->setText(" 暂 停 ");
        ui->pushButton_clutchpause->setText(" 暂 停 ");

        ui->lineEdit_shiftnow->setText("N_3&4");
        ui->comboBox_shiftaim->setCurrentIndex(3);
        ui->lineEdit_shifttrace->setText("N_3&4 ---> N_1&2 ---> 1");
    }
    else
    {
        RobotParams::currentshiftindex = 1;
        RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
        RobotParams::lastshiftindex = 1;
        RobotParams::aimshiftindex = 2;

        RobotParams::currentclutchindex = 1;
        RobotParams::currentclutchvalue = "自动";

        RobotParams::aimclutchindex = 1;

        shiftexampause = false;
        clutchexampause = false;
        exampause = false;

        ui->pushButton_shiftpause->setText(" 暂 停 ");
        ui->pushButton_pause->setText(" 暂 停 ");
        ui->pushButton_clutchpause->setText(" 暂 停 ");

        ui->lineEdit_shiftnow->setText("N");
        ui->comboBox_shiftaim->setCurrentIndex(2);
        ui->lineEdit_shifttrace->setText("N ---> D");
    }

    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

    // 开始监听模式
    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::examFilePath + "SC_ARM");
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

    // 将当前的离合和挡位保存到预留变量中
    RobotParams::tempVars[0] = RobotParams::angleRealTime[2];
    RobotParams::tempVars[1] = RobotParams::angleRealTime[3];
    RobotParams::tempVars[2] = RobotParams::angleRealTime[4];

    // 将当前的刹车和油门保存到预留变量中
    RobotParams::tempVars[5] = RobotParams::angleRealTime[0];
    RobotParams::tempVars[6] = RobotParams::angleRealTime[1];

    // 定时器打开
    examflag = 0;
    examtimer->start(RobotParams::UITimerMs);

    // 纠正到空挡 抬起离合
    // 开始测试
    examflag = 1;
    RobotParams::isExaming = true;
}

void ShiftClutchUI::on_pushButton_stopexam_clicked()
{
    if (Configuration::GetInstance()->ifManualShift)
    {
        if (RobotParams::currentshiftindex == 1 && RobotParams::currentclutchindex == 1) // 在空挡 离合松开 即刻退出测试
        {
            RobotParams::currentshiftindex = 1;
            RobotParams::currentshiftvalue = "N_3&4";
            RobotParams::currentclutchindex = 1;
            RobotParams::currentclutchvalue = "松开";

            shiftexampause = false;
            clutchexampause = false;
            exampause = false;
            examflag = 0;
            round = 1;
            round2 = 1;

            // 停止
            AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
            PRINTF(LOG_INFO, "%s: exam stops.\n", __func__);
            examtimer->stop();

            // 退出测试标志位
            RobotParams::isExaming = false;
        }
        else
        {
            // 规划路径
            RobotParams::aimshiftindex = 1;
            mySCControl->plantrace();

            // 离合目标
            RobotParams::aimclutchindex = 1;

            // 退出测试
            examflag = 9;
        }
    }
    else
    {
        if (RobotParams::currentshiftindex == 1) // 在空挡 即刻退出测试
        {
            RobotParams::currentshiftindex = 1;
            RobotParams::currentshiftvalue = "N";
            RobotParams::currentclutchindex = 1;
            RobotParams::currentclutchvalue = "自动";

            shiftexampause = false;
            clutchexampause = false;
            exampause = false;
            examflag = 0;
            round = 1;
            round2 = 1;

            // 停止
            AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
            PRINTF(LOG_INFO, "%s: exam stops.\n", __func__);
            examtimer->stop();

            // 退出测试标志位
            RobotParams::isExaming = false;
        }
        else
        {
            // 目标空挡
            RobotParams::aimshiftindex = 1;

            // 退出测试
            examflag = 9;
        }
    }

    // 界面设置
    ui->pushButton_startexam->setEnabled(true);
    ui->pushButton_stopexam->setEnabled(false);
    ui->tabWidget2->setEnabled(false);
}

void ShiftClutchUI::on_comboBox_shiftaim_currentIndexChanged(int index)
{
    if (ifenablecomboBoxchangedeventhappen)
    {
        if (Configuration::GetInstance()->ifManualShift)
        {
            // 规划路径
            RobotParams::aimshiftindex = index;
            if (RobotParams::aimshiftindex == RobotParams::currentshiftindex)
            {
                ui->lineEdit_shifttrace->setText(" ----- ----- ");
                ui->pushButton_shiftrun->setEnabled(false);
                ui->pushButton_run->setEnabled(false);
                QMessageBox::information(NULL, tr("提示"), tr("已经到达位置！"));
                return;
            }
            if (RobotParams::aimshiftindex == 0 || RobotParams::aimshiftindex ==2)
            {
                ui->lineEdit_shifttrace->setText(" ----- ----- ");
                ui->pushButton_shiftrun->setEnabled(false);
                ui->pushButton_run->setEnabled(false);
                QMessageBox::information(NULL, tr("提示"), tr("不能选择中间挡位！"));
                return;
            }
            mySCControl->plantrace();

            QString str = "";
            for (int i = 0; i < RobotParams::shiftrunlength - 1; i++)
            {
                str += ui->comboBox_shiftaim->itemText(RobotParams::shiftrunpath[i]) + " ---> ";
            }
            str += ui->comboBox_shiftaim->itemText(RobotParams::shiftrunpath[RobotParams::shiftrunlength - 1]);

            ui->lineEdit_shifttrace->setText(str);
            ui->pushButton_shiftrun->setEnabled(true);
            ui->pushButton_run->setEnabled(true);
            ui->pushButton_0to1->setEnabled(true);
        }
        else
        {
            // 规划路径
            RobotParams::aimshiftindex = index;
            if (RobotParams::aimshiftindex == RobotParams::currentshiftindex)
            {
                ui->lineEdit_shifttrace->setText(" ----- ----- ");
                ui->pushButton_shiftrun->setEnabled(false);

                QMessageBox::information(NULL, tr("提示"), tr("已经到达位置！"));
                return;
            }

            QString str = "";
            str += ui->comboBox_shiftaim->itemText(RobotParams::currentshiftindex) + " ---> ";
            str += ui->comboBox_shiftaim->itemText(RobotParams::aimshiftindex);

            ui->lineEdit_shifttrace->setText(str);
            ui->pushButton_shiftrun->setEnabled(true);
        }
    }
}

void ShiftClutchUI::on_pushButton_shiftrun_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Off || RobotParams::powerMode == PedalRobot::Accessory)
    {
        // 测试生成的换挡路径
        examflag = 2;

        // 设置界面
        ui->pushButton_shiftrun->setEnabled(false);
        ui->pushButton_shiftpause->setEnabled(true);
        ui->pushButton_run->setEnabled(false);
        ui->pushButton_pause->setEnabled(false);
        ui->pushButton_0to1->setEnabled(false);
        ui->pushButton_1to0->setEnabled(false);
        ui->tab_examclutch->setEnabled(false);
        ui->lineEdit_shifttime->setText("");
    }
    else
    {
        if (Configuration::GetInstance()->ifManualShift)
        {
            if (RobotParams::currentclutchindex == 0)
            {
                // 测试生成的换挡路径
                examflag = 2;

                // 设置界面
                ui->pushButton_shiftrun->setEnabled(false);
                ui->pushButton_shiftpause->setEnabled(true);
                ui->pushButton_run->setEnabled(false);
                ui->pushButton_pause->setEnabled(false);
                ui->pushButton_0to1->setEnabled(false);
                ui->pushButton_1to0->setEnabled(false);
                ui->tab_examclutch->setEnabled(false);
                ui->lineEdit_shifttime->setText("");
            }
            else
            {
                QMessageBox::information(NULL, tr("提示"), tr("车辆已发动，必须踩下离合才能测试挡位！"));
            }
        }
        else
        {
            // 测试生成的换挡路径
            examflag = 2;

            // 设置界面
            ui->pushButton_shiftrun->setEnabled(false);
            ui->pushButton_shiftpause->setEnabled(true);
            ui->pushButton_run->setEnabled(false);
            ui->pushButton_pause->setEnabled(false);
            ui->pushButton_0to1->setEnabled(false);
            ui->pushButton_1to0->setEnabled(false);
            ui->tab_examclutch->setEnabled(false);
            ui->lineEdit_shifttime->setText("");
        }
    }
}

void ShiftClutchUI::on_pushButton_shiftpause_clicked()
{
    if (shiftexampause)
    {
        shiftexampause = false;
        ui->pushButton_shiftpause->setText(tr("暂停"));
    }
    else
    {
        shiftexampause = true;
        ui->pushButton_shiftpause->setText(tr("继续"));
    }
}

void ShiftClutchUI::on_pushButton_bottom_clicked()
{
    // 离合目标
    RobotParams::aimclutchindex = 0;

    // 测试离合位置 踩住
    examflag = 3;

    // 设置界面
    ui->pushButton_bottom->setEnabled(false);
    ui->pushButton_top->setEnabled(false);
    ui->pushButton_speed->setEnabled(false);
    ui->pushButton_clutchpause->setEnabled(true);
    ui->tab_examshift->setEnabled(false);
}

void ShiftClutchUI::on_pushButton_top_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Off || RobotParams::powerMode == PedalRobot::Accessory)
    {
        // 离合目标
        RobotParams::aimclutchindex = 1;

        // 测试离合位置 松开
        examflag = 4;

        // 设置界面
        ui->pushButton_bottom->setEnabled(false);
        ui->pushButton_top->setEnabled(false);
        ui->pushButton_speed->setEnabled(false);
        ui->pushButton_clutchpause->setEnabled(true);
        ui->tab_examshift->setEnabled(false);
    }
    else
    {
        QMessageBox::information(NULL, tr("提示"), tr("车辆已发动，不能直接抬起离合踏板！"));
    }
}

void ShiftClutchUI::on_pushButton_speed_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Off || RobotParams::powerMode == PedalRobot::Accessory)
    {
        // 测试离合速度
        examflag = 5;

        // 设置界面
        ui->pushButton_bottom->setEnabled(false);
        ui->pushButton_top->setEnabled(false);
        ui->pushButton_speed->setEnabled(false);
        ui->pushButton_clutchpause->setEnabled(true);
        ui->tab_examshift->setEnabled(false);
    }
    else
    {
        if (RobotParams::currentshiftindex == 1)
        {
            // 测试离合速度
            examflag = 5;

            // 设置界面
            ui->pushButton_bottom->setEnabled(false);
            ui->pushButton_top->setEnabled(false);
            ui->pushButton_speed->setEnabled(false);
            ui->pushButton_clutchpause->setEnabled(true);
            ui->tab_examshift->setEnabled(false);
        }
        else
        {
            QMessageBox::information(NULL, tr("提示"), tr("车辆已发动，只有在空挡才能缓慢抬起离合踏板！"));
        }
    }
}

void ShiftClutchUI::on_pushButton_clutchpause_clicked()
{
    if (clutchexampause)
    {
        clutchexampause = false;
        ui->pushButton_clutchpause->setText(tr(" 暂 停 "));
    }
    else
    {
        clutchexampause = true;
        ui->pushButton_clutchpause->setText(tr(" 继 续 "));
    }
}

bool ShiftClutchUI::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->lineEdit_type && event->type() == QEvent::MouseButtonRelease)
    {
        bool ok;
        QString cartype = QInputDialog::getText(this,QObject::tr("提示"),QObject::tr("请输入更改的车型"),
                                               QLineEdit::Normal,"",&ok);
        if(ok){
            if(cartype.isEmpty()){
                QMessageBox::information(this,"提示",QObject::tr("车型必须非空"));
                return true;
            }

            Configuration::GetInstance()->carTypeName = ( cartype + QString(".xml") ).toStdString();
            if(Configuration::GetInstance()->SaveToFile() == 0){
                ui->lineEdit_type->setText( QString::fromStdString( Configuration::GetInstance()->carTypeName.substr(0, Configuration::GetInstance()->carTypeName.length() - 4) ) );
                QMessageBox::information(this,"提示",QObject::tr("车型修改成功"));
            }else{
                QMessageBox::information(this,"提示",QObject::tr("!!!车型修改失败!!!"));
            }
        }

        return true;
    }

    return QWidget::eventFilter(watched, event);
}

void ShiftClutchUI::SendMoveCommand(double clutch, double shift1, double shift2, bool run, bool ifboth, bool ifclutch)
{
    std::vector<int> actionMethod;
    std::vector<int> actionAxes;
    std::vector<double> actionTheta;

    for (unsigned int i=0; i<2; ++i)
    {
        actionAxes.push_back(i);
        actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
    }
    actionTheta.push_back(0.0); actionTheta.push_back(0.0);

    if (run)
    {
        if (ifboth)
        {
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
            actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
        }
        else
        {
            if (ifclutch)
            {
                actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
                actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
                actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
                actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
            }
            else
            {
                actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
                actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
                actionMethod.push_back(AutoDriveRobotApiClient::AbsControlMethod);
                actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
            }
        }
    }
    else
    {
        for (unsigned int i=2; i<RobotParams::axisNum; ++i)
        {
            actionMethod.push_back(AutoDriveRobotApiClient::DeltaControlMethod);
        }
    }

    actionAxes.push_back(2); actionAxes.push_back(3);
    actionAxes.push_back(4); actionAxes.push_back(5);
    actionTheta.push_back(clutch);
    actionTheta.push_back(shift1); actionTheta.push_back(shift2);
    actionTheta.push_back(0.0);

    AutoDriveRobotApiClient::GetInstance()->Send_SetMonitorActionThetaMsg(actionMethod, actionAxes, actionTheta);
}

void ShiftClutchUI::on_pushButton_motor3minus_pressed()
{
    const double speed = -1 * RobotParams::singleAxisBtnRatioC;

    std::vector<int> moveAxes;
    std::vector<double> moveSpeed;
    moveAxes.push_back(2);
    moveSpeed.push_back(speed);

    AutoDriveRobotApiClient::GetInstance()->Send_MoveSingleAxisMsg(moveAxes, moveSpeed);
}

void ShiftClutchUI::on_pushButton_motor3minus_released()
{
    std::vector<int> stopAxes;
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        stopAxes.push_back(i);
    }

    AutoDriveRobotApiClient::GetInstance()->Send_StopSingleAxisMsg(stopAxes);
}

void ShiftClutchUI::on_pushButton_motor3plus_pressed()
{
    const double speed = 1 * RobotParams::singleAxisBtnRatioC;

    std::vector<int> moveAxes;
    std::vector<double> moveSpeed;
    moveAxes.push_back(2);
    moveSpeed.push_back(speed);

    AutoDriveRobotApiClient::GetInstance()->Send_MoveSingleAxisMsg(moveAxes, moveSpeed);
}


void ShiftClutchUI::on_pushButton_motor3plus_released()
{
    std::vector<int> stopAxes;
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        stopAxes.push_back(i);
    }

    AutoDriveRobotApiClient::GetInstance()->Send_StopSingleAxisMsg(stopAxes);
}

void ShiftClutchUI::on_pushButton_run_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Off || RobotParams::powerMode == PedalRobot::Accessory)
    {
        // 测试生成的换挡路径
        examflag = 6;

        // 设置界面
        ui->pushButton_shiftrun->setEnabled(false);
        ui->pushButton_shiftpause->setEnabled(false);
        ui->pushButton_run->setEnabled(false);
        ui->pushButton_pause->setEnabled(true);
        ui->pushButton_0to1->setEnabled(false);
        ui->pushButton_1to0->setEnabled(false);
        ui->tab_examclutch->setEnabled(false);
        ui->lineEdit_shifttime->setText("");
    }
    else
    {
        QMessageBox::information(NULL, tr("提示"), tr("车辆已发动，不能进行换挡测试！"));
    }
}

void ShiftClutchUI::on_pushButton_pause_clicked()
{
    if (exampause)
    {
        exampause = false;
        ui->pushButton_pause->setText(tr(" 暂 停 "));
    }
    else
    {
        exampause = true;
        ui->pushButton_pause->setText(tr(" 继 续 "));
    }
}

void ShiftClutchUI::SendMoveCommandAll(double *values, int *ifABS)
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

void ShiftClutchUI::on_pushButton_stopnow_clicked()
{
    shiftexampause = false;
    clutchexampause = false;
    exampause = false;
    examflag = 0;
    round = 1;
    round2 = 1;

    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
    PRINTF(LOG_WARNING, "%s: immediately stop.\n", __func__);
    examtimer->stop();

    // 退出测试标志位
    RobotParams::isExaming = false;

    // 界面设置
    ui->pushButton_startexam->setEnabled(true);
    ui->pushButton_stopexam->setEnabled(false);
    ui->tabWidget2->setEnabled(false);

    // 按照车型配置文件更新examsoftStop.txt
    std::fstream essf(Configuration::examsoftStopFilePath.c_str(), std::fstream::out | std::fstream::binary);
    if(essf.fail()){
        PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, Configuration::examsoftStopFilePath.c_str());
        return;
    }
    essf << RobotParams::robotType << "\n";
    essf << 'R' << "\n";
    essf << std::right << std::setw(15) << 0;
    essf << std::right << std::setw(15) << 0;
    essf << std::right << std::setw(15) << 2000;
    essf << std::right << std::setw(15) << 0;
    essf << std::right << std::setw(15) << 0 << "\n";
    essf << 'T' << "\n";
    essf << Configuration::GetInstance()->translateSpeed << "\n";
    essf << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[0];
    essf << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];

    if (Configuration::GetInstance()->ifManualShift)
    {
        essf << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[0];
        essf << std::right << std::setw(15) << RobotParams::angleRealTime[3];
        essf << std::right << std::setw(15) << RobotParams::angleRealTime[4];
        essf << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }
    else
    {
        essf << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
        essf << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[1];
        essf << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[1];
        essf << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }

    essf.close();

    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::examsoftStopFilePath);
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
}

void ShiftClutchUI::on_pushButton_brakeslowly_clicked()
{
    shiftexampause = false;
    clutchexampause = false;
    exampause = false;
    examflag = 0;
    round = 1;
    round2 = 1;

    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
    PRINTF(LOG_WARNING, "%s: immediately stop.\n", __func__);
    examtimer->stop();

    // 退出测试标志位
    RobotParams::isExaming = false;

    // 界面设置
    ui->pushButton_startexam->setEnabled(true);
    ui->pushButton_stopexam->setEnabled(false);
    ui->tabWidget2->setEnabled(false);

    // 按照车型配置文件更新examslowlyBrake.txt
    std::fstream esb(Configuration::examslowlyBrakeFilePath.c_str(), std::fstream::out | std::fstream::binary);
    if(esb.fail()){
        PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, Configuration::examslowlyBrakeFilePath.c_str());
        return;
    }
    esb << RobotParams::robotType << "\n";
    esb << 'R' << "\n";
    esb << std::right << std::setw(15) << 0;
    esb << std::right << std::setw(15) << 0;
    esb << std::right << std::setw(15) << 2000;
    esb << std::right << std::setw(15) << 0;
    esb << std::right << std::setw(15) << 0 << "\n";
    esb << 'T' << "\n";
    esb << Configuration::GetInstance()->translateSpeed << "\n";
    esb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[0];
    esb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];

    if (Configuration::GetInstance()->ifManualShift)
    {
        esb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[0];
        esb << std::right << std::setw(15) << RobotParams::angleRealTime[3];
        esb << std::right << std::setw(15) << RobotParams::angleRealTime[4];
        esb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

        // 踩刹车
        esb << 'T' << "\n";
        esb << Configuration::GetInstance()->translateSpeed/4 << "\n";
        esb << std::right << std::setw(15) << Configuration::GetInstance()->brakeThetaAfterGoHome;
        esb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
        esb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[0];
        esb << std::right << std::setw(15) << RobotParams::angleRealTime[3];
        esb << std::right << std::setw(15) << RobotParams::angleRealTime[4];
        esb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }
    else
    {
        esb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
        esb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[1];;
        esb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[1];;
        esb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

        // 踩刹车
        esb << 'T' << "\n";
        esb << Configuration::GetInstance()->translateSpeed/4 << "\n";
        esb << std::right << std::setw(15) << Configuration::GetInstance()->brakeThetaAfterGoHome;
        esb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
        esb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
        esb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[1];;
        esb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[1];;
        esb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }

    esb.close();

    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::examslowlyBrakeFilePath);
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
}

void ShiftClutchUI::on_pushButton_confirmSC_clicked()
{
    int ret = QMessageBox::information(NULL, tr("提示"), tr("请确认挡位离合信息！"), tr("确认"), tr("取消"));
    if(ret == 1)
    {
        RobotParams::ifConfirmSC = false;
    }
    else
    {
        RobotParams::ifConfirmSC = true;

        int emergencyStopType = !Configuration::GetInstance()->ifManualShift;

        std::vector<double> emergencyStopTheta;
        emergencyStopTheta.push_back( Configuration::GetInstance()->deathPos[0] );
        emergencyStopTheta.push_back( Configuration::GetInstance()->deathPos[1] );

        if (Configuration::GetInstance()->ifManualShift)
        {
            emergencyStopTheta.push_back( Configuration::GetInstance()->clutchAngles[0] );
            emergencyStopTheta.push_back( 0.0 );
            emergencyStopTheta.push_back( 0.0 );
        }
        else
        {
            emergencyStopTheta.push_back( 0.0 );
            emergencyStopTheta.push_back( Configuration::GetInstance()->shiftAxisAngles1[1] );
            emergencyStopTheta.push_back( Configuration::GetInstance()->shiftAxisAngles2[1] );
        }

        emergencyStopTheta.push_back( RobotParams::angleRealTime[5] );

        AutoDriveRobotApiClient::GetInstance()->Send_SetPedalRobotEmergencyStopThetaMsg(emergencyStopType, emergencyStopTheta);
    }
}

void ShiftClutchUI::on_checkBox_autoset_stateChanged(int arg1)
{
    if (arg1)
    {
        Configuration::GetInstance()->ifAutoRecordMidN = true;
    }
    else
    {
        Configuration::GetInstance()->ifAutoRecordMidN = false;
    }
}

void ShiftClutchUI::on_pushButton_0to1_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Run)
    {
        if (RobotParams::currentshiftindex == 1)
        {
            if (RobotParams::currentclutchindex == 1)
            {
                // 测试起步
                examflag = 7;

                // 设置界面
                ui->pushButton_shiftrun->setEnabled(false);
                ui->pushButton_shiftpause->setEnabled(false);
                ui->pushButton_run->setEnabled(false);
                ui->pushButton_pause->setEnabled(false);
                ui->pushButton_0to1->setEnabled(false);
                ui->pushButton_1to0->setEnabled(false);
                ui->tab_examclutch->setEnabled(false);
                ui->lineEdit_shifttime->setText("");
            }
            else
            {
                QMessageBox::information(NULL, tr("提示"), tr("起步测试前请保证离合踏板抬起！"));
            }
        }
        else
        {
            QMessageBox::information(NULL, tr("提示"), tr("起步测试前请保证挡位在空挡！"));
        }
    }
    else
    {
        QMessageBox::information(NULL, tr("提示"), tr("起步测试前请先发动车辆！"));
    }
}

void ShiftClutchUI::on_pushButton_1to0_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Run)
    {
        if (RobotParams::currentshiftindex == 3)
        {
            if (RobotParams::currentclutchindex == 1)
            {
                // 测试起步
                examflag = 8;

                // 设置界面
                ui->lineEdit_shifttime->setText("");
            }
            else
            {
                QMessageBox::information(NULL, tr("提示"), tr("起步测试前请保证离合踏板抬起！"));
            }
        }
        else
        {
            QMessageBox::information(NULL, tr("提示"), tr("起步停止测试前请保证挡位在1挡！"));
        }
    }
    else
    {
        QMessageBox::information(NULL, tr("提示"), tr("起步停止测试前请先发动车辆！"));
    }
}

void ShiftClutchUI::on_tabWidget_NW_currentChanged(int index)
{
    Q_UNUSED(index);
    ui->tabWidget_NW->setCurrentIndex(Configuration::GetInstance()->pedalRobotUsage);
}

void ShiftClutchUI::on_pushButton_generate_clicked()
{
    if (Configuration::GetInstance()->pedalRobotUsage == 0)
    {
        // 读取NEDC文件 获得曲线数据
        SysControl::PairData datas;

        if(ReadDatas(datas) == -1){
            QMessageBox::information(NULL,"warning",QString("无法读取该NEDC曲线数据!"));
            return;
        }

        // 换挡时刻
        RobotParams::changeshiftlist.clear();
        RobotParams::changeshiftlist.reserve(128);

        RobotParams::changeshiftlist.push_back( std::make_pair(9, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(25, 1) ); // 1=>N
        RobotParams::changeshiftlist.push_back( std::make_pair(47, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(54, 4) ); // 1=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(93, 1) ); // 2=>N
        RobotParams::changeshiftlist.push_back( std::make_pair(115, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(122, 4) ); // 1=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(133, 5) ); // 2=>3
        RobotParams::changeshiftlist.push_back( std::make_pair(176, 4) ); // 3=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(185, 1) ); // 2=>N

        RobotParams::changeshiftlist.push_back( std::make_pair(195 + 9, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(195 + 25, 1) ); // 1=>N
        RobotParams::changeshiftlist.push_back( std::make_pair(195 + 47, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(195 + 54, 4) ); // 1=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(195 + 93, 1) ); // 2=>N
        RobotParams::changeshiftlist.push_back( std::make_pair(195 + 115, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(195 + 122, 4) ); // 1=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(195 + 133, 5) ); // 2=>3
        RobotParams::changeshiftlist.push_back( std::make_pair(195 + 176, 4) ); // 3=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(195 + 185, 1) ); // 2=>N

        RobotParams::changeshiftlist.push_back( std::make_pair(2 * 195 + 9, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(2 * 195 + 25, 1) ); // 1=>N
        RobotParams::changeshiftlist.push_back( std::make_pair(2 * 195 + 47, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(2 * 195 + 54, 4) ); // 1=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(2 * 195 + 93, 1) ); // 2=>N
        RobotParams::changeshiftlist.push_back( std::make_pair(2 * 195 + 115, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(2 * 195 + 122, 4) ); // 1=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(2 * 195 + 133, 5) ); // 2=>3
        RobotParams::changeshiftlist.push_back( std::make_pair(2 * 195 + 176, 4) ); // 3=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(2 * 195 + 185, 1) ); // 2=>N

        RobotParams::changeshiftlist.push_back( std::make_pair(3 * 195 + 9, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(3 * 195 + 25, 1) ); // 1=>N
        RobotParams::changeshiftlist.push_back( std::make_pair(3 * 195 + 47, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(3 * 195 + 54, 4) ); // 1=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(3 * 195 + 93, 1) ); // 2=>N
        RobotParams::changeshiftlist.push_back( std::make_pair(3 * 195 + 115, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(3 * 195 + 122, 4) ); // 1=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(3 * 195 + 133, 5) ); // 2=>3
        RobotParams::changeshiftlist.push_back( std::make_pair(3 * 195 + 176, 4) ); // 3=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(3 * 195 + 185, 1) ); // 2=>N

        RobotParams::changeshiftlist.push_back( std::make_pair(798, 3) ); // N=>1
        RobotParams::changeshiftlist.push_back( std::make_pair(805, 4) ); // 1=>2
        RobotParams::changeshiftlist.push_back( std::make_pair(816, 5) ); // 2=>3
        RobotParams::changeshiftlist.push_back( std::make_pair(826, 6) ); // 3=>4
        RobotParams::changeshiftlist.push_back( std::make_pair(841, 7) ); // 4=>5
        RobotParams::changeshiftlist.push_back( std::make_pair(889, 6) ); // 5=>4
        RobotParams::changeshiftlist.push_back( std::make_pair(981, 7) ); // 4=>5
        RobotParams::changeshiftlist.push_back( std::make_pair(1150, 1) ); // 5=>N

        // 绘制换挡时刻
        PlotCST(datas);
    }
    else if (Configuration::GetInstance()->pedalRobotUsage == 1)
    {

    }

}

int ShiftClutchUI::ReadDatas(SysControl::PairData& datas)
{
    std::string fileName;
    if (Configuration::GetInstance()->pedalRobotUsage == 0)
    {
        // 读取NEDC文件 获得曲线数据
        fileName = Configuration::examFilePath + "NEDC";
    }
    else if (Configuration::GetInstance()->pedalRobotUsage == 1)
    {
        // 读取WLTC文件 获得曲线数据
        fileName = Configuration::examFilePath + "WLTC";
    }

    std::ifstream fs(fileName.c_str(), std::fstream::binary);
    PRINTF(LOG_DEBUG, "%s: read file = %s\n",__func__, fileName.c_str());
    if(fs.fail()){
        PRINTF(LOG_DEBUG, "%s: fail to open teach file = %s\n",__func__, fileName.c_str());
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
                datas.clear();
                fs>>pointsNum;
                datas.reserve(pointsNum);

                while(!fs.eof()){
                    fs>>time>>speed;
                    datas.push_back( std::make_pair(time,speed) );
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

void ShiftClutchUI::PlotCST(SysControl::PairData &data_o)
{
    ui->plotwidget->clearGraphs();
    ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
    ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));

    RobotParams::ifConfirmCS = false;

    QPen myPen;
    myPen.setWidth(3);

    // 目标曲线 0
    ui->plotwidget->addGraph();
    myPen.setColor(Qt::red);
    ui->plotwidget->graph(0)->setPen(myPen);

    size_t num_cs = RobotParams::changeshiftlist.size();

    // 换挡时刻 1 ~ num_cs
    for (unsigned int i=1; i<= num_cs; ++i)
    {
        ui->plotwidget->addGraph();
        myPen.setColor(Qt::blue);
        ui->plotwidget->graph(i)->setPen(myPen);
    }

    ui->plotwidget->xAxis->setRange(0, 180);

    // 加载数据
    double maxY = 0.0;
    size_t sz = data_o.size();
    QVector<double> time(sz), speed(sz);
    time.clear(); time.resize(sz);
    speed.clear(); speed.resize(sz);
    for(size_t i=0; i<sz; ++i){
        time[i] = data_o[i].first;
        speed[i]= data_o[i].second;
        maxY = std::max(maxY, speed[i]+5.0);
    }
    ui->plotwidget->yAxis->setRange(-15, maxY);
    ui->plotwidget->graph(0)->setData(time, speed);

    for (unsigned int i=1; i<= num_cs; ++i)
    {
        QVector<double> time_cs(2), lim_cs(2);
        time_cs.clear(); time_cs.resize(2);
        lim_cs.clear(); lim_cs.resize(2);
        time_cs[0] = RobotParams::changeshiftlist[i-1].first;
        time_cs[1] = RobotParams::changeshiftlist[i-1].first;
        lim_cs[0] = -500;
        lim_cs[1] = 500;
        ui->plotwidget->graph(i)->setData(time_cs, lim_cs);

        QCPItemText *csText = new QCPItemText(ui->plotwidget);
        csText->setPositionAlignment(Qt::AlignLeft|Qt::AlignTop);
        csText->position->setCoords(time_cs[0], 0);
        csText->setTextAlignment(Qt::AlignLeft);
        csText->setFont(QFont("KaiTi",16));

        if (i == 1)
        {
           csText->setText("N=>1");
        }
        else
        {

            QString targetshift, currentshift;
            if (RobotParams::changeshiftlist[i-1].second == 1)
            {
                targetshift = "N";
            }
            else
            {
                targetshift = QString::fromStdString(RobotParams::manulShiftValues[RobotParams::changeshiftlist[i-1].second]);
            }
            if (RobotParams::changeshiftlist[i-2].second == 1)
            {
                currentshift = "N";
            }
            else
            {
                currentshift = QString::fromStdString(RobotParams::manulShiftValues[RobotParams::changeshiftlist[i-2].second]);
            }
            csText->setText(currentshift + QString("=>") + targetshift);
        }
    }

    // 绘制
    ui->plotwidget->replot();
}

void ShiftClutchUI::on_pushButton_confirmCS_clicked()
{
    int ret = QMessageBox::information(NULL, tr("提示"), tr("请确认换挡时刻信息！"), tr("确认"), tr("取消"));
    if(ret == 1)
    {
        RobotParams::ifConfirmCS = false;
    }
    else
    {
        RobotParams::ifConfirmCS = true;
    }
}
