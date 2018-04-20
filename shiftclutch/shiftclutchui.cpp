#include "shiftclutchui.h"
#include "ui_shiftclutchui.h"

#include "configuration.h"
#include <QPixmapCache>
#include <QMessageBox>
#include <QTextStream>
#include <QFileDialog>
#include <QInputDialog>
#include <QTimer>
#include "printf.h"
#include "robotparams.h"
#include "autodriverobotapiclient.h"
#include "robotapi/AssistantFunc/fileassistantfunc.h"
#include "fileoperation/normalfile.h"

ShiftClutchUI::ShiftClutchUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ShiftClutchUI),
    haveReadXML(false)
{
    ui->setupUi(this);

    // 定时器初始化 用来测试
    examtimer = new QTimer(this);
    connect(examtimer, SIGNAL(timeout()), this, SLOT(examtimer_timeout()));

    // 初始化UI
    initialui();

    // 初始化和重置部分控件
    initiallist();
    resetcomboBox();
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
    if (Configuration::GetInstance()->ifManualShift)
    {
        const QString picpath = QString::fromStdString(Configuration::GetInstance()->mainFolder) + "/manualshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(true);
        ui->tab_exam->setEnabled(true);
    }
    else
    {
        const QString picpath = QString::fromStdString(Configuration::GetInstance()->mainFolder) + "/autoshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(false);
        ui->tab_exam->setEnabled(false);
    }
}

void ShiftClutchUI::initialui()
{
    // 显示图像 修改部分控件使能
    QPixmapCache::clear();
    if (Configuration::GetInstance()->ifManualShift)
    {
        const QString picpath = QString::fromStdString(Configuration::GetInstance()->mainFolder) + "/manualshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(true);
        ui->tab_exam->setEnabled(true);
    }
    else
    {
        const QString picpath = QString::fromStdString(Configuration::GetInstance()->mainFolder) + "/autoshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(false);
        ui->tab_exam->setEnabled(false);
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
            return;
        }
    }

    ui->lineEdit_shiftnow->setText( QString::fromStdString( RobotParams::currentshiftvalue ) );

//    double theta[RobotParams::axisNum] = {0.0};
//    if (examflag == 0)
//    {
//        MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::DeltaControl, 0, theta, RobotParams::axisNum);
//        CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//    }
//    else if (examflag == 1)
//    {
//        if (shiftexampause)
//        {
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::DeltaControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            return;
//        }

//        if (ifreached(false, true, shifttrace[shifttracelength - 1]))
//        {
//            currentshift = shifttrace[shifttracelength - 1];
//            shifttracepointer = 0;
//            examflag = 0;
//            ui->pushButton_shiftrun->setEnabled(true);
//            ui->pushButton_shiftpause->setEnabled(false);
//            ui->tab_examclutch->setEnabled(true);
//            PRINTF(LOG_INFO, "%s: this shift exam finishes.\n", __func__);
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::DeltaControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            return;
//        }

//        if (ifreached(false, true, shifttrace[shifttracepointer + 1]))
//        {
//            shifttracepointer++;

//            if (shifttracepointer == shifttracelength - 1)
//            {
//                currentshift = shifttrace[shifttracepointer];
//                shifttracepointer = 0;
//                examflag = 0;
//                ui->pushButton_shiftrun->setEnabled(true);
//                ui->pushButton_shiftpause->setEnabled(false);
//                ui->tab_examclutch->setEnabled(true);
//                PRINTF(LOG_INFO, "%s: this shift exam finishes.\n", __func__);
//                MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::DeltaControl, 0, theta, RobotParams::axisNum);
//                CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            }
//            else
//            {
//                currentshift = shifttrace[shifttracepointer];
//                aimshift = shifttrace[shifttracepointer + 1];
//                for (int i = 0; i < RobotParams::axisNum; i++)
//                {
//                    theta[i] = scara_theta[i];
//                }
//                theta[shiftaxis1] = shiftangles[shifttrace[shifttracepointer + 1]][0];
//                theta[shiftaxis2] = shiftangles[shifttrace[shifttracepointer + 1]][1];
//                MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::AbsControl, 0, theta, RobotParams::axisNum);
//                CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            }
//        }
//        else
//        {
//            currentshift = shifttrace[shifttracepointer];
//            aimshift = shifttrace[shifttracepointer + 1];
//            for (int i = 0; i < RobotParams::axisNum; i++)
//            {
//                theta[i] = scara_theta[i];
//            }
//            theta[shiftaxis1] = shiftangles[shifttrace[shifttracepointer + 1]][0];
//            theta[shiftaxis2] = shiftangles[shifttrace[shifttracepointer + 1]][1];
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::AbsControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//        }
//    }
//    else if (examflag == 2)
//    {
//        if (clutchexampause)
//        {
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::DeltaControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            return;
//        }

//        if (ifreached(false, false, 0))
//        {
//            examflag = 0;
//            ui->pushButton_bottom->setEnabled(true);
//            ui->pushButton_top->setEnabled(true);
//            ui->pushButton_speed->setEnabled(true);
//            ui->pushButton_clutchpause->setEnabled(false);
//            ui->tab_examshift->setEnabled(true);
//            PRINTF(LOG_INFO, "%s: clutch bottom exam finishes.\n", __func__);
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::DeltaControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//        }
//        else
//        {
//            for (int i = 0; i < RobotParams::axisNum; i++)
//            {
//                theta[i] = scara_theta[i];
//            }
//            theta[clutchaxis] = clutchangles[0];
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::AbsControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//        }
//    }
//    else if (examflag == 21)
//    {
//        if (clutchexampause)
//        {
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::DeltaControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            return;
//        }

//        if (ifreached(false, false, 1))
//        {
//            examflag = 0;
//            ui->pushButton_bottom->setEnabled(true);
//            ui->pushButton_top->setEnabled(true);
//            ui->pushButton_speed->setEnabled(true);
//            ui->pushButton_clutchpause->setEnabled(false);
//            ui->tab_examshift->setEnabled(true);
//            PRINTF(LOG_INFO, "%s: clutch top exam finishes.\n", __func__);
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::DeltaControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//        }
//        else
//        {
//            for (int i = 0; i < RobotParams::axisNum; i++)
//            {
//                theta[i] = scara_theta[i];
//            }
//            theta[clutchaxis] = clutchangles[1];
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::AbsControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//        }
//    }
//    else if (examflag == 3)
//    {
//        if (clutchexampause)
//        {
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::DeltaControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            return;
//        }

//        if (clutchspeedstate == 0)
//        {
//            if (ifreached(false, false, 0))
//            {
//                clutchspeedstate = 1;
//                PRINTF(LOG_INFO, "%s: clutch speed exam, reaches bottom.\n", __func__);

//                for (int i = 0; i < RobotParams::axisNum; i++)
//                {
//                    theta[i] = scara_theta[i];
//                }
//                theta[clutchaxis] = clutchangles[0] - clutchspeed;

//                MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::AbsControl, 0, theta, RobotParams::axisNum);
//                CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            }
//            else
//            {
//                for (int i = 0; i < RobotParams::axisNum; i++)
//                {
//                    theta[i] = scara_theta[i];
//                }
//                theta[clutchaxis] = clutchangles[0];
//                MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::AbsControl, 0, theta, RobotParams::axisNum);
//                CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            }
//        }
//        else if (clutchspeedstate == 1)
//        {
//            static int round = 1;
//            if (ifreached(false, false, 1))
//            {
//                examflag = 0;
//                clutchspeedstate = 0;
//                ui->pushButton_bottom->setEnabled(true);
//                ui->pushButton_top->setEnabled(true);
//                ui->pushButton_speed->setEnabled(true);
//                ui->pushButton_clutchpause->setEnabled(false);
//                ui->tab_examshift->setEnabled(true);
//                round = 1;
//                PRINTF(LOG_INFO, "%s: clutch speed exam finishes.\n", __func__);
//                MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::DeltaControl, 0, theta, RobotParams::axisNum);
//                CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            }
//            else
//            {
//                for (int i = 0; i < RobotParams::axisNum; i++)
//                {
//                    theta[i] = scara_theta[i];
//                }

//                round++;
//                if ((theta[clutchaxis] = clutchangles[0] - round * clutchspeed) < clutchangles[1])
//                {
//                    theta[clutchaxis] = clutchangles[1];
//                }

//                MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::AbsControl, 0, theta, RobotParams::axisNum);
//                CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            }
//        }
//    }
//    else if (examflag == 4)
//    {
//        if (ifreached(false, true, shifttrace[shifttracepointer + 1]))
//        {
//            shifttracepointer++;

//            if (shifttracepointer == shifttracelength - 1)
//            {
//                currentshift = 1;
//                aimshift = 3;
//                shifttrace[0] = 1;
//                shifttrace[1] = 0;
//                shifttrace[2] = 3;
//                shifttracelength = 3;
//                shifttracepointer = 0;
//                clutchspeedstate = 0;
//                shiftexampause = false;
//                clutchexampause = false;
//                examflag = 0;
//                CommunicationMediator::GetInstance()->SendMsg(ENTER_IDLE_STATE_2A, CommunicationMediator::DownlinkMsg);
//                PRINTF(LOG_INFO, "%s: exam stops.\n", __func__);
//                examtimer->stop();
//            }
//            else
//            {
//                currentshift = shifttrace[shifttracepointer];
//                aimshift = shifttrace[shifttracepointer + 1];
//                for (int i = 0; i < RobotParams::axisNum; i++)
//                {
//                    theta[i] = scara_theta[i];
//                }
//                theta[shiftaxis1] = shiftangles[shifttrace[shifttracepointer + 1]][0];
//                theta[shiftaxis2] = shiftangles[shifttrace[shifttracepointer + 1]][1];
//                MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::AbsControl, 0, theta, RobotParams::axisNum);
//                CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//            }
//        }
//        else
//        {
//            currentshift = shifttrace[shifttracepointer];
//            aimshift = shifttrace[shifttracepointer + 1];
//            for (int i = 0; i < RobotParams::axisNum; i++)
//            {
//                theta[i] = scara_theta[i];
//            }
//            theta[shiftaxis1] = shiftangles[shifttrace[shifttracepointer + 1]][0];
//            theta[shiftaxis2] = shiftangles[shifttrace[shifttracepointer + 1]][1];
//            MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::AbsControl, 0, theta, RobotParams::axisNum);
//            CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//        }
//    }
//    else
//    {
//        PRINTF(LOG_WARNING, "%s: (%d) does not exist.\n", __func__, examflag);
//    }
}

void ShiftClutchUI::on_comboBox_way_currentIndexChanged(int index)
{
    if (ifenablewaychangedeventhappen)
    {
        if (index == 0)
        {
            Configuration::GetInstance()->ifManualShift = true;

            // 改变图像
            QPixmapCache::clear();
            const QString picpath = QString::fromStdString(Configuration::GetInstance()->mainFolder) + "/manualshift.png";
            pic.load(picpath);
            ui->label_pic->setPixmap(pic);

            // 设定离合界面
            ui->tab_clutch->setEnabled(true);

            // 设定测试界面
            ui->tab_exam->setEnabled(true);
        }
        else
        {
            Configuration::GetInstance()->ifManualShift = false;

            // 改变图像
            QPixmapCache::clear();
            const QString picpath = QString::fromStdString(Configuration::GetInstance()->mainFolder) + "/autoshift.png";
            pic.load(picpath);
            ui->label_pic->setPixmap(pic);

            // 设定离合界面
            ui->tab_clutch->setEnabled(false);

            // 设定测试界面
            ui->tab_exam->setEnabled(false);
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
    const QString txtpath = QString::fromStdString(Configuration::GetInstance()->carTypeFilePath);
    QString fileName = QFileDialog::getOpenFileName(this, tr("读取"), txtpath);
    std::string fn = NormalFile::GetFileName(fileName.toStdString().c_str());

    Configuration::GetInstance()->carTypeName = fn.substr(0, fn.length() - 4);

    if(Configuration::GetInstance()->ReadFromFile() == 0){
        QMessageBox::information( this,"提示", tr( (QString("读取").toStdString() + Configuration::GetInstance()->carTypeName + QString("成功").toStdString()).c_str() ) );
        haveReadXML = true;
    }else{
        QMessageBox::information(this,"提示", tr( (QString("!!!读取").toStdString() + Configuration::GetInstance()->carTypeName + QString("失败!!!").toStdString()).c_str() ) );
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
    if (Configuration::GetInstance()->ifManualShift)
    {
        const QString picpath = QString::fromStdString(Configuration::GetInstance()->mainFolder) + "/manualshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(true);
        ui->tab_exam->setEnabled(true);
    }
    else
    {
        const QString picpath = QString::fromStdString(Configuration::GetInstance()->mainFolder) + "/autoshift.png";
        pic.load(picpath);
        ui->label_pic->setPixmap(pic);

        ui->tab_clutch->setEnabled(false);
        ui->tab_exam->setEnabled(false);
    }
}

void ShiftClutchUI::on_pushButton_record1_clicked()
{
    const int jg = ui->comboBox_shift->currentIndex();
    Configuration::GetInstance()->shiftAxisAngles1[jg] = RobotParams::angleRealTime[3];
    Configuration::GetInstance()->shiftAxisAngles2[jg] = RobotParams::angleRealTime[4];

    if (Configuration::GetInstance()->ifManualShift)
    {
        switch (jg)
        {
        case 0:
            ui->list1->item(jg + 1)->setText("N_1&2\t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        case 1:
            ui->list1->item(jg + 1)->setText("N_3&4\t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
            break;
        case 2:
            ui->list1->item(jg + 1)->setText("N_5&6\t\t\t\t " + QString::number(RobotParams::angleRealTime[3], 'f', 2) + "\t\t\t\t " + QString::number(RobotParams::angleRealTime[4], 'f', 2));
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
    QMessageBox::information(NULL, tr("提示"), tr("请确认所有挡位离合信息采集完毕，\r\n再把挡位拨到空挡附近！"));

    // 保证在空挡
    if (!mySCControl->ifreachedshift(true, 1))
    {
        QMessageBox::warning(NULL, tr("警告"), tr("挡位不在空挡附加，请把挡位拨到空挡附近！"));
        return;
    }

    RobotParams::currentshiftindex = 1;
    RobotParams::currentshiftvalue = ui->comboBox_shift->itemText(RobotParams::currentshiftindex).toStdString();
    RobotParams::aimshiftindex = 3;

    RobotParams::shiftrunpath[0] = 1;
    RobotParams::shiftrunpath[1] = 0;
    RobotParams::shiftrunpath[2] = 3;
    RobotParams::shiftrunlength = 3;
    RobotParams::shiftrunpointer = 0;

    RobotParams::currentclutchindex = 0;
    switch (RobotParams::currentclutchindex)
    {
    case 0:
        RobotParams::currentclutchvalue = "踩下";
        break;
    case 1:
        RobotParams::currentclutchvalue = "松开";
        break;
    case 2:
        RobotParams::currentclutchvalue = "抬升";
        break;
    default:
        break;
    }

    shiftexampause = false;
    clutchexampause = false;

    ui->lineEdit_shiftnow->setText("N_3&4");
    ui->comboBox_shiftaim->setCurrentIndex(3);
    ui->lineEdit_shifttrace->setText("N_3&4 ---> N_1&2 ---> 1");

    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

    // 开始监听模式
    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::examFilePath + "SC_ARM");
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

    // 纠正到空挡 抬起离合 sbs
//    double theta[RobotParams::axisNum] = {0.0};
//    for (int i = 0; i < RobotParams::axisNum; i++)
//    {
//        theta[i] = scara_theta[i];
//    }
//    theta[shiftaxis1] = shiftangles[1][0];
//    theta[shiftaxis2] = shiftangles[1][1];
//    MsgStructure::MonitorCommand mc(MsgStructure::MonitorCommand::AbsControl, 0, theta, RobotParams::axisNum);
//    CommunicationMediator::GetInstance()->SendMsg(mc, CommunicationMediator::DownlinkMsg);
//    Sleep(50);

    // 定时器打开
    examflag = 0;
    examtimer->start(RobotParams::UITimerMs);

    // 界面设置
    ui->pushButton_startexam->setEnabled(false);
    ui->pushButton_stopexam->setEnabled(true);
    ui->tabWidget2->setEnabled(true);

    ui->pushButton_shiftrun->setEnabled(true);
    ui->pushButton_shiftpause->setEnabled(false);
    ui->pushButton_bottom->setEnabled(true);
    ui->pushButton_top->setEnabled(true);
    ui->pushButton_speed->setEnabled(true);
    ui->pushButton_clutchpause->setEnabled(false);
}

void ShiftClutchUI::on_pushButton_stopexam_clicked()
{
    if (RobotParams::currentshiftindex == 1) // 在空挡即刻退出测试
    {
        RobotParams::aimshiftindex = 3;
        RobotParams::shiftrunpath[0] = 1;
        RobotParams::shiftrunpath[1] = 0;
        RobotParams::shiftrunpath[2] = 3;
        RobotParams::shiftrunlength = 3;
        RobotParams::shiftrunpointer = 0;
        examflag = 0;

        // 停止
        AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
        PRINTF(LOG_INFO, "%s: exam stops.\n", __func__);
        examtimer->stop();
    }
    else
    {
        // 规划路径 sbs
//        aimshift = 1;
//        plantrace(currentshift, aimshift, shifttrace, &shifttracelength);

        // 退出测试
        examflag = 6;
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
        // 规划路径
        RobotParams::aimshiftindex = index;
        if (RobotParams::aimshiftindex == RobotParams::currentshiftindex)
        {
            ui->lineEdit_shifttrace->setText(" ----- ----- ");
            ui->pushButton_shiftrun->setEnabled(false);
            QMessageBox::information(NULL, tr("提示"), tr("已经到达位置！"));
            return;
        }
        if (RobotParams::aimshiftindex == 0 || RobotParams::aimshiftindex ==2)
        {
            ui->lineEdit_shifttrace->setText(" ----- ----- ");
            ui->pushButton_shiftrun->setEnabled(false);
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
    }
}

void ShiftClutchUI::on_pushButton_shiftrun_clicked()
{
    // 测试生成的换挡路径
    examflag = 2;

    // 设置界面
    ui->pushButton_shiftrun->setEnabled(false);
    ui->pushButton_shiftpause->setEnabled(true);
    ui->tab_examclutch->setEnabled(false);
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
    // 测试离合位置 松开
    examflag = 4;

    // 设置界面
    ui->pushButton_bottom->setEnabled(false);
    ui->pushButton_top->setEnabled(false);
    ui->pushButton_speed->setEnabled(false);
    ui->pushButton_clutchpause->setEnabled(true);
    ui->tab_examshift->setEnabled(false);
}

void ShiftClutchUI::on_pushButton_speed_clicked()
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

void ShiftClutchUI::on_pushButton_clutchpause_clicked()
{
    if (clutchexampause)
    {
        clutchexampause = false;
        ui->pushButton_clutchpause->setText(tr("暂停"));
    }
    else
    {
        clutchexampause = true;
        ui->pushButton_clutchpause->setText(tr("继续"));
    }
}

void ShiftClutchUI::on_lineEdit_type_returnPressed()
{
    bool ok;
    QString cartype = QInputDialog::getText(this,QObject::tr("提示"),QObject::tr("请输入更改的车型"),
                                           QLineEdit::Normal,"",&ok);
    if(ok){
        if(cartype.isEmpty()){
            QMessageBox::information(this,"提示",QObject::tr("车型必须非空"));
            return;
        }

        Configuration::GetInstance()->carTypeName = ( cartype + QString(".xml") ).toStdString();
        if(Configuration::GetInstance()->SaveToFile() == 0){
            QMessageBox::information(this,"提示",QObject::tr("车型修改成功"));
        }else{
            QMessageBox::information(this,"提示",QObject::tr("!!!车型修改失败!!!"));
        }
    }
}