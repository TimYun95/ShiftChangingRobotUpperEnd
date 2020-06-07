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
#include "shiftclutchparams.h"
#include "robotparams.h"
#include "autodriverobotapiclient.h"
#include "robotapi/AssistantFunc/fileassistantfunc.h"
#include "fileoperation/normalfile.h"
#include "pedal/pedalrobot.h"

ShiftClutchUI::ShiftClutchUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ShiftClutchUI)
//    haveReadXML(false)
{
    ui->setupUi(this);

    ui->lineEdit_type->installEventFilter(this); // 注册事件过滤

    // 定时器初始化 用来测试
    examtimer = new QTimer(this);
    connect(examtimer, SIGNAL(timeout()), this, SLOT(examtimer_timeout()));

    // 初始化UI
    InitialUI();
}

ShiftClutchUI::~ShiftClutchUI()
{
    delete ui;
    delete examtimer;
}

void ShiftClutchUI::ConnectXMLSignalWithSlot(QWidget *stui)
{
    connect( stui, SIGNAL(ReadXMLFromSetting()), this, SLOT(UpdateShowingDatas()) );
    connect( stui, SIGNAL(SaveXMLFromSetting()), this, SLOT(SetFalseSCConfirm()) );
}

void ShiftClutchUI::UpdateAnglesForShiftClutch()
{
    ui->lineEdit_motor1->setText( QString::number(RobotParams::angleRealTime[3], 'f', 2) );
    ui->lineEdit_motor2->setText( QString::number(RobotParams::angleRealTime[4], 'f', 2) );
    ui->lineEdit_motor3->setText( QString::number(RobotParams::angleRealTime[2], 'f', 2) );
}

QString ShiftClutchUI::GetCurrentShiftString(int chooseIndex)
{
    if (chooseIndex < 0)
    {
        if (Configuration::GetInstance()->ifManualShift)
        {
            return ManualShiftStateString[RobotParams::shiftIndex];
        }
        else
        {
            return AutoShiftStateString[RobotParams::shiftIndex];
        }
    }
    else
    {
        if (Configuration::GetInstance()->ifManualShift)
        {
            return ManualShiftStateString[chooseIndex];
        }
        else
        {
            return AutoShiftStateString[chooseIndex];
        }
    }
}

QString ShiftClutchUI::GetCurrentClutchString(int chooseIndex)
{
    if (chooseIndex < 0)
    {
        return ClutchStateString[RobotParams::clutchIndex];
    }
    else
    {
        return ClutchStateString[chooseIndex];
    }
}

bool ShiftClutchUI::GetIfAtNullShift()
{
    if (Configuration::GetInstance()->ifManualShift)
    {
        return ifReachedPositionbyHand((int)ManualShiftState::Gear_N);
    }
    else
    {
        return ifReachedPositionbyHand((int)AutoShiftState::Gear_N);
    }
}

void ShiftClutchUI::UpdateShowingDatas()
{
    RefreshShiftMap();
    RefreshMainControlStrip();
    RefreshShiftClutchTeachPanel();
    RefreshExamPanel();
    RefreshChangeShiftPanel();
}

void ShiftClutchUI::SetFalseSCConfirm()
{
    ifConfirmShiftClutchInfo = false;
    RefreshConfirmShiftClutchInfoState();
}

bool ShiftClutchUI::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->lineEdit_type && event->type() == QEvent::MouseButtonRelease)
    {
        bool isinputfinished;
        QString cartype = QInputDialog::getText(this,QObject::tr("提示"),QObject::tr("请输入更改的车型"),
                                               QLineEdit::Normal,"",&isinputfinished);
        if (isinputfinished)
        {
            if (cartype.isEmpty())
            {
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


void ShiftClutchUI::examtimer_timeout()
{
    //软件倍频
    if(examtimer->isActive()){
        static int cnt = 0;
        if(++cnt%RobotParams::UITimerMultiplier != 0){
            return;
        }
    }
    else return;

    if (ShiftClutchParams::shiftclutchState < 0)
    {
        PRINTF(LOG_ERR, "%s: error in shift changing.\n", __func__);

        // 停止
        AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

        examflag = -1;
        isExaming = false;
        examtimer->stop();

        ui->pushButton_startexam->setEnabled(true);
        return;
    }

    currentclutchindex = ShiftClutchParams::clutchState;
    currentshiftindex = ShiftClutchParams::shiftState;
    if (Configuration::GetInstance()->ifManualShift)
    {
        ui->lineEdit_shiftnow->setText(ManualShiftStateString[currentshiftindex]);
    }
    else
    {
        ui->lineEdit_shiftnow->setText(AutoShiftStateString[currentshiftindex]);
    }

    if (examflag == -1)
    {
        PRINTF(LOG_DEBUG, "%s: now at no command state.\n", __func__);
    }
    else if (examflag == 0)
    {
        SendMotionCmd(relativebrk, relativeacc);
    }
    else if (examflag == 1)
    {
        if (pauseflag == 0)
        {
            ui->lineEdit_shifttime->setText("");

            bool ifsuccesschangefile = false;
            if (Configuration::GetInstance()->ifManualShift)
            {
                QVector<QVector<double>> anglelist;
                QVector<double> liftpedalandclutch = {
                    Configuration::GetInstance()->deathPos[0],
                    Configuration::GetInstance()->deathPos[1],
                    Configuration::GetInstance()->clutchAngles[(int)ClutchState::Released],
                    RobotParams::angleRealTime[3],
                    RobotParams::angleRealTime[4],
                    RobotParams::angleRealTime[5]};
                QVector<double> dragtoNshift = {
                    Configuration::GetInstance()->deathPos[0],
                    Configuration::GetInstance()->deathPos[1],
                    Configuration::GetInstance()->clutchAngles[(int)ClutchState::Released],
                    Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_N],
                    Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_N],
                    RobotParams::angleRealTime[5]};
                anglelist.append(liftpedalandclutch);
                anglelist.append(dragtoNshift);
                ifsuccesschangefile = SetTempArrivalFile(anglelist, 2);
            }
            else
            {
                QVector<QVector<double>> anglelist;
                QVector<double> pressbrkanddragtoNshift = {
                    Configuration::GetInstance()->limPos[0],
                    Configuration::GetInstance()->deathPos[1],
                    Configuration::GetInstance()->clutchAngles[(int)ClutchState::Released],
                    Configuration::GetInstance()->shiftAxisAngles1[(int)AutoShiftState::Gear_N],
                    Configuration::GetInstance()->shiftAxisAngles2[(int)AutoShiftState::Gear_N],
                    RobotParams::angleRealTime[5]};
                anglelist.append(pressbrkanddragtoNshift);
                ifsuccesschangefile = SetTempArrivalFile(anglelist, 1);
            }

            if (!ifsuccesschangefile)
            {
                examflag = -1;
                examtimer->stop();
                PRINTF(LOG_WARNING, "%s: exam initial file refresh failed.\n", __func__);
            }
            else
            {
                std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::temparrivalFilePath);
                if(fileContent.empty()){
                    PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
                    return;
                }
                AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

                pauseflag++;
            }
        }

        if (pauseflag <= 8)
        {
            pauseflag++;
            return;
        }

        if (RobotParams::statusStrIndex != 28)
        {
            if (pauseflag > 150)
            {
                pauseflag = 0;
                examflag = -1;
                examtimer->stop();
                PRINTF(LOG_WARNING, "%s: exam initial failed.\n", __func__);
            }
            else
            {
                pauseflag++;
                return;
            }
        }
        else
        {
            pauseflag = 0;

            // 停止
            AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

            // 开始监听模式
            std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::stdFilePath + "SC_ARM");
            if(fileContent.empty()){
                PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
                return;
            }
            AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

            ui->pushButton_stopexam->setEnabled(true);
            ui->tabWidget2->setEnabled(true);

            examflag = 0;
            isExaming = true;
            PRINTF(LOG_INFO, "%s: be ready to exam.\n", __func__);
        }
    }
    else if (examflag == 2)
    {
        if (shiftexampause)
        {
            SendMotionCmd(0, 0, false, true);
            return;
        }

        if (currentshiftindex == nextshiftindex)
        {
            examflag = 0;
            pauseflag = 0;
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime);
            PRINTF(LOG_INFO, "%s: only shift change finished.\n", __func__);
            return;
        }

        if (pauseflag == 0)
        {
            if (Configuration::GetInstance()->ifManualShift)
            {
                SendMotionCmd(0, 0, true, false,
                              nextshiftindex,
                              (int)ClutchState::Released,
                              (int)ShiftChangingMode::OnlyShift);
            }
            else
            {
                SendMotionCmd(0, 0, true, false,
                              nextshiftindex,
                              (int)ClutchState::Released,
                              (int)ShiftChangingMode::AutoShift);
            }
        }
        else
        {
            SendMotionCmd(0, 0);
        }
        pauseflag++;

        if (pauseflag > 7)
        {
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime);
        }
    }
    else if (examflag == 3)
    {
        if (clutchexampause)
        {
            SendMotionCmd(0, 0, false, true);
            return;
        }

        if (currentclutchindex == (int)ClutchState::Pressed)
        {
            examflag = 0;
            pauseflag = 0;
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime, true);
            PRINTF(LOG_INFO, "%s: only clutch pressed finished.\n", __func__);
            return;
        }

        if (pauseflag == 0)
        {
            SendMotionCmd(0, 0, true, false,
                          (int)ManualShiftState::Gear_N,
                          (int)ClutchState::Pressed,
                          (int)ShiftChangingMode::OnlyClutch);
        }
        else
        {
            SendMotionCmd(0, 0);
        }
        pauseflag++;

        if (pauseflag > 7)
        {
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime, true);
        }
    }
    else if (examflag == 4)
    {
        if (clutchexampause)
        {
            SendMotionCmd(0, 0, false, true);
            return;
        }

        if (currentclutchindex == (int)ClutchState::Released)
        {
            examflag = 0;
            pauseflag = 0;
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime, true);
            PRINTF(LOG_INFO, "%s: only clutch released finished.\n", __func__);
            return;
        }

        if (pauseflag == 0)
        {
            SendMotionCmd(0, 0, true, false,
                          (int)ManualShiftState::Gear_N,
                          (int)ClutchState::Released,
                          (int)ShiftChangingMode::OnlyClutch);
        }
        else
        {
            SendMotionCmd(0, 0);
        }
        pauseflag++;

        if (pauseflag > 7)
        {
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime, true);
        }
    }
    else if (examflag == 5)
    {
        int clutchaim;
        if (ui->checkBox_departure->isChecked())
        {
            clutchaim = (int)ClutchState::SlowlyReleasingAtDeparture;
        }
        else
        {
            clutchaim = (int)ClutchState::SlowlyReleasing;
        }

        if (clutchexampause)
        {
            SendMotionCmd(0, 0, false, true);
            return;
        }

        if (currentclutchindex == (int)ClutchState::Released)
        {
            examflag = 0;
            pauseflag = 0;
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime, true);

            ui->checkBox_departure->setEnabled(true);
            PRINTF(LOG_INFO, "%s: only clutch slowly releasing finished.\n", __func__);
            return;
        }

        if (pauseflag == 0)
        {
            SendMotionCmd(0, 0, true, false,
                          (int)ManualShiftState::Gear_N,
                          clutchaim,
                          (int)ShiftChangingMode::OnlyClutch);
        }
        else
        {
            SendMotionCmd(0, 0);
        }
        pauseflag++;

        if (pauseflag > 7)
        {
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime, true);
        }
    }
    else if (examflag == 6)
    {
        if (exampause)
        {
            SendMotionCmd(0, 0, false, true);
            return;
        }

        if (currentshiftindex == nextshiftindex && currentclutchindex == (int)ClutchState::Released)
        {
            examflag = 0;
            pauseflag = 0;
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime);
            PRINTF(LOG_INFO, "%s: 3-axis shift change finished.\n", __func__);
            return;
        }

        if (pauseflag == 0)
        {
            SendMotionCmd(0, 0, true, false,
                          nextshiftindex,
                          (int)ClutchState::Released,
                          (int)ShiftChangingMode::ThreeAxis);
        }
        else
        {
            SendMotionCmd(0, 0);
        }
        pauseflag++;

        if (pauseflag > 7)
        {
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime);
        }
    }
    else if (examflag == 7)
    {
        if (exampause)
        {
            SendMotionCmd(0, 0, false, true);
            return;
        }

        if (currentshiftindex == nextshiftindex && currentclutchindex == (int)ClutchState::Released)
        {
            examflag = 0;
            pauseflag = 0;
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime);
            PRINTF(LOG_INFO, "%s: 5-axis shift change finished.\n", __func__);
            return;
        }

        if (pauseflag == 0)
        {
            SendMotionCmd(0, 0, true, false,
                          nextshiftindex,
                          (int)ClutchState::Released,
                          (int)ShiftChangingMode::FiveAxis);
        }
        else
        {
            SendMotionCmd(0, 0);
        }
        pauseflag++;

        if (pauseflag > 7)
        {
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime);
        }
    }
    else if (examflag == 8)
    {
        int howtoreleaseclutch = ui->radioButton_keep->isChecked() ? (int)ClutchReleasingMode::Slowly : (int)ClutchReleasingMode::SlowlyWithRecovery;

        if (pauseflag == 0)
        {
            SendMotionCmd(0, 0, true, false,
                          nextshiftindex,
                          (int)ClutchState::Released,
                          (int)ShiftChangingMode::ManualShift,
                          howtoreleaseclutch);
        }
        else
        {
            SendMotionCmd(0, 0);
        }
        pauseflag++;

        if (pauseflag > 7)
        {
            ResolveAndShowTime(ShiftClutchParams::totalTime, ShiftClutchParams::partialTime);

            if (ShiftClutchParams::shiftclutchState == 0)
            {
                examflag = 0;
                pauseflag = 0;
                PRINTF(LOG_INFO, "%s: manual shift change finished.\n", __func__);
                return;
            }
        }
    }
    else if (examflag == 19)
    {
        ui->lineEdit_shifttime->setText("");

        if (Configuration::GetInstance()->ifManualShift)
        {
            if (currentshiftindex == (int)ManualShiftState::Gear_N)
            {
                if (currentclutchindex == (int)ClutchState::Released)
                {
                    if (ifPedalAtPositions(true, true))
                    {
                        // 停止
                        AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

                        examflag = -1;
                        isExaming = false;
                        examtimer->stop();

                        ui->pushButton_startexam->setEnabled(true);
                    }
                    else
                    {
                        SendMotionCmd(-100, -100);
                    }
                }
                else
                {
                    if (pauseflag == 0)
                    {
                        SendMotionCmd(0, 0, true, false,
                                      (int)ManualShiftState::Gear_N,
                                      (int)ClutchState::Released,
                                      (int)ShiftChangingMode::OnlyClutch);
                    }
                    else
                    {
                        SendMotionCmd(0, 0);
                    }
                    pauseflag++;

                    if (pauseflag > 7)
                    {
                        if (ShiftClutchParams::shiftclutchState == 0)
                        {
                            pauseflag = 0;
                        }
                    }
                }
            }
            else
            {
                if (pauseflag == 0)
                {
                    SendMotionCmd(0, 0, true, false,
                                  (int)ManualShiftState::Gear_N,
                                  (int)ClutchState::Released,
                                  (int)ShiftChangingMode::FiveAxis);
                }
                else
                {
                    SendMotionCmd(0, 0);
                }
                pauseflag++;

                if (pauseflag > 7)
                {
                    if (ShiftClutchParams::shiftclutchState == 0)
                    {
                        pauseflag = 0;
                    }
                }
            }
        }
        else
        {
            if (currentshiftindex == (int)AutoShiftState::Gear_N)
            {
                if (ifPedalAtPositions(true, true))
                {
                    // 停止
                    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

                    examflag = -1;
                    isExaming = false;
                    examtimer->stop();

                    ui->pushButton_startexam->setEnabled(true);
                }
                else
                {
                    SendMotionCmd(-100, -100);
                }
            }
            else
            {
                if (pauseflag == 0)
                {
                    SendMotionCmd(0, 0, true, false,
                                  (int)AutoShiftState::Gear_N,
                                  (int)ClutchState::Released,
                                  (int)ShiftChangingMode::AutoShift);
                }
                else
                {
                    SendMotionCmd(0, 0);
                }
                pauseflag++;

                if (pauseflag > 7)
                {
                    if (ShiftClutchParams::shiftclutchState == 0)
                    {
                        pauseflag = 0;
                    }
                }
            }
        }
    }
    else
    {
        PRINTF(LOG_WARNING, "%s: (%d) state does not exist in exam.\n", __func__, examflag);
    }
    return;
}


void ShiftClutchUI::on_comboBox_way_currentIndexChanged(int index)
{
    if (ifenablewaychangedeventhappen)
    {
        ifConfirmShiftClutchInfo = false;
        ifConfirmChangingShiftTime = false;

        Configuration::GetInstance()->ifManualShift = !index;
        Configuration::GetInstance()->ifExistSixShift = false;
        Configuration::GetInstance()->ifExistBackShift = false;
        Configuration::GetInstance()->ifAutoRecordMidN = false;

        RefreshShiftMap();
        RefreshConfirmShiftClutchInfoState();
        SetZeroShiftClutchInfo();
        RefreshShiftClutchTeachPanel();
        RefreshExamPanel();
        RefreshChangeShiftPanel();
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
    const QString txtpath = QString::fromStdString(Configuration::sysFilePath);
    QString fileName = QFileDialog::getOpenFileName(this, tr("读取"), txtpath, tr("XML Files(*.xml)"));
    if (fileName == "") return;
    std::string fn = NormalFile::GetFileName(fileName.toStdString().c_str());

    std::string tempcarname = Configuration::GetInstance()->carTypeName;
    Configuration::GetInstance()->carTypeName = fn;

    if(Configuration::GetInstance()->ReadFromFile() == 0){
        QMessageBox::information( this,"提示", tr( (QString("读取").toStdString() + Configuration::GetInstance()->carTypeName + QString("成功").toStdString()).c_str() ) );

        ifConfirmShiftClutchInfo = false;
        ifConfirmChangingShiftTime = false;

        RefreshShiftMap();
        RefreshMainControlStrip();
        RefreshShiftClutchTeachPanel();
        RefreshExamPanel();
        RefreshChangeShiftPanel();

        emit ReadXMLFromShifting();
    }else{
        QMessageBox::information(this,"提示", tr( (QString("!!!读取").toStdString() + Configuration::GetInstance()->carTypeName + QString("失败!!!").toStdString()).c_str() ) );
        Configuration::GetInstance()->carTypeName = tempcarname;
        return;
    }
}

void ShiftClutchUI::on_pushButton_confirmSC_clicked()
{
    int ret = QMessageBox::information(NULL, tr("提示"), tr("请确认挡位离合信息！"), tr("确认"), tr("取消"));
    if(ret == 0)
    {
        ifConfirmShiftClutchInfo = true;

        RefreshConfirmShiftClutchInfoState();

        // 发送重置信息
        SendResetSignal();

        // 发送挡位离合信息
        SendShiftClutchInfo();

        // 发送急停位置消息
        SendEmergencyStopInfo();

        // 保存并发送配置文件
        AutoDriveRobotApiClient::GetInstance()->Send_SaveAndSendConfMsg(true, true);
    }
}

void ShiftClutchUI::on_checkBox_sixshift_stateChanged(int arg1)
{

    if (ifenablecheckBoxchangedeventhappen)
    {
        if (arg1)
        {
            Configuration::GetInstance()->ifExistSixShift = true;
        }
        else
        {
            Configuration::GetInstance()->ifExistSixShift = false;
        }

        Configuration::GetInstance()->shiftAxisAngles1[9] = 0;
        Configuration::GetInstance()->shiftAxisAngles2[9] = 0;

        RefreshShiftMap();
        RefreshShiftLists(true, false);
        RefreshShiftComboBox();
    }
}

void ShiftClutchUI::on_checkBox_backshift_stateChanged(int arg1)
{
    if (ifenablecheckBoxchangedeventhappen)
    {
        if (arg1)
        {
            Configuration::GetInstance()->ifExistBackShift = true;
        }
        else
        {
            Configuration::GetInstance()->ifExistBackShift = false;
        }

        Configuration::GetInstance()->shiftAxisAngles1[10] = 0;
        Configuration::GetInstance()->shiftAxisAngles2[10] = 0;

        RefreshShiftMap();
        RefreshShiftLists(true, false);
        RefreshShiftComboBox();
    }
}

void ShiftClutchUI::on_checkBox_autoset_stateChanged(int arg1)
{
    if (arg1)
    {
        Configuration::GetInstance()->ifAutoRecordMidN = true;
        RefreshShiftComboBox();
        UnenableComboBoxItem(ui->comboBox_shift, 8);
        UnenableComboBoxItem(ui->comboBox_shift, 9);
        UnenableComboBoxItem(ui->comboBox_shift, 10);

        AutoRecordAtManualShift();
    }
    else
    {
        Configuration::GetInstance()->ifAutoRecordMidN = false;
        RefreshShiftComboBox();
    }
}

void ShiftClutchUI::on_pushButton_record1_clicked()
{
    ifConfirmShiftClutchInfo = false;
    RefreshConfirmShiftClutchInfoState();

    const int tempshiftindexC = ui->comboBox_shift->currentIndex();
    const int tempshiftindex = GetEnumIndexfromComboBoxIndexofShift(tempshiftindexC);

    Configuration::GetInstance()->shiftAxisAngles1[tempshiftindex] = RobotParams::angleRealTime[3];
    Configuration::GetInstance()->shiftAxisAngles2[tempshiftindex] = RobotParams::angleRealTime[4];

    ModifyShiftInfoItem(tempshiftindexC, Configuration::GetInstance()->shiftAxisAngles1[tempshiftindex], Configuration::GetInstance()->shiftAxisAngles2[tempshiftindex]);
}

void ShiftClutchUI::on_pushButton_reset1_clicked()
{
    ifConfirmShiftClutchInfo = false;
    RefreshConfirmShiftClutchInfoState();

    SetZeroShiftClutchInfo(true, false);
    RefreshShiftLists(true, true);
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


void ShiftClutchUI::on_pushButton_motor1plus_pressed()
{
    const double speed = 1 * RobotParams::singleAxisBtnRatioS;

    std::vector<int> moveAxes;
    std::vector<double> moveSpeed;
    moveAxes.push_back(3);
    moveSpeed.push_back(speed);

    AutoDriveRobotApiClient::GetInstance()->Send_MoveSingleAxisMsg(moveAxes, moveSpeed);
}

void ShiftClutchUI::on_pushButton_motor1plus_released()
{
    std::vector<int> stopAxes;
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        stopAxes.push_back(i);
    }

    AutoDriveRobotApiClient::GetInstance()->Send_StopSingleAxisMsg(stopAxes);
}

void ShiftClutchUI::on_pushButton_motor1minus_pressed()
{
    const double speed = -1 * RobotParams::singleAxisBtnRatioS;

    std::vector<int> moveAxes;
    std::vector<double> moveSpeed;
    moveAxes.push_back(3);
    moveSpeed.push_back(speed);

    AutoDriveRobotApiClient::GetInstance()->Send_MoveSingleAxisMsg(moveAxes, moveSpeed);
}

void ShiftClutchUI::on_pushButton_motor1minus_released()
{
    std::vector<int> stopAxes;
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        stopAxes.push_back(i);
    }

    AutoDriveRobotApiClient::GetInstance()->Send_StopSingleAxisMsg(stopAxes);
}

void ShiftClutchUI::on_pushButton_motor2plus_pressed()
{
    const double speed = 1 * RobotParams::singleAxisBtnRatioS;

    std::vector<int> moveAxes;
    std::vector<double> moveSpeed;
    moveAxes.push_back(4);
    moveSpeed.push_back(speed);

    AutoDriveRobotApiClient::GetInstance()->Send_MoveSingleAxisMsg(moveAxes, moveSpeed);
}

void ShiftClutchUI::on_pushButton_motor2plus_released()
{
    std::vector<int> stopAxes;
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        stopAxes.push_back(i);
    }

    AutoDriveRobotApiClient::GetInstance()->Send_StopSingleAxisMsg(stopAxes);
}

void ShiftClutchUI::on_pushButton_motor2minus_pressed()
{
    const double speed = -1 * RobotParams::singleAxisBtnRatioS;

    std::vector<int> moveAxes;
    std::vector<double> moveSpeed;
    moveAxes.push_back(4);
    moveSpeed.push_back(speed);

    AutoDriveRobotApiClient::GetInstance()->Send_MoveSingleAxisMsg(moveAxes, moveSpeed);
}

void ShiftClutchUI::on_pushButton_motor2minus_released()
{
    std::vector<int> stopAxes;
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        stopAxes.push_back(i);
    }

    AutoDriveRobotApiClient::GetInstance()->Send_StopSingleAxisMsg(stopAxes);
}

void ShiftClutchUI::on_comboBox_clutch_currentIndexChanged(int index)
{
    Q_UNUSED(index);
    RefreshClutchSpeed();
}

void ShiftClutchUI::on_pushButton_record2_clicked()
{
    ifConfirmShiftClutchInfo = false;
    RefreshConfirmShiftClutchInfoState();

    const int tempclutchindex = ui->comboBox_clutch->currentIndex();
    double info;

    switch (tempclutchindex) {
    case 0:
        Configuration::GetInstance()->clutchAngles[(int)ClutchState::Pressed] = RobotParams::angleRealTime[2];
        info = Configuration::GetInstance()->clutchAngles[(int)ClutchState::Pressed];
        break;
    case 1:
        Configuration::GetInstance()->clutchAngles[(int)ClutchState::Released] = RobotParams::angleRealTime[2];
        info = Configuration::GetInstance()->clutchAngles[(int)ClutchState::Released];
        break;
    case 2:
        Configuration::GetInstance()->clutchUpSpeed = ui->lineEdit_speed->text().trimmed().toDouble();
        info = Configuration::GetInstance()->clutchUpSpeed;
        break;
    case 3:
        Configuration::GetInstance()->clutchUpSpeedAtDeparture = ui->lineEdit_speed->text().trimmed().toDouble();
        info = Configuration::GetInstance()->clutchUpSpeedAtDeparture;
        break;
    default:
        break;
    }

    ModifyClutchInfoItem(tempclutchindex, info);
}

void ShiftClutchUI::on_pushButton_reset2_clicked()
{
    ifConfirmShiftClutchInfo = false;
    RefreshConfirmShiftClutchInfoState();

    SetZeroShiftClutchInfo(false, true);
    RefreshShiftLists(true, true);
}

void ShiftClutchUI::on_pushButton_startexam_clicked()
{
    // 挡位离合信息确认
    if (!ifConfirmShiftClutchInfo)
    {
        QMessageBox::warning(NULL, tr("警告"), tr("请确认挡位信息后再进行测试！"));
        return;
    }

    // 手动归空挡提示
    int ret = QMessageBox::information(NULL, tr("提示"), tr("请确认所有挡位离合信息采集完毕，\r\n再把挡位拨到空挡附近！"), tr("确认"), tr("取消"));
    if(ret == 1)
    {
        return;
    }

    // 保证在空挡
    bool ifatN = false;
    if (Configuration::GetInstance()->ifManualShift)
    {
        ifatN = ifReachedPositionbyHand((int)ManualShiftState::Gear_N);
    }
    else
    {
        ifatN = ifReachedPositionbyHand((int)AutoShiftState::Gear_N);
    }
    if (!ifatN)
    {
        QMessageBox::warning(NULL, tr("警告"), tr("挡位不在空挡附近\r\n请手动调整！"));
        return;
    }

    // 禁用按钮
    ui->pushButton_startexam->setEnabled(false);

    // 部分初始化
    if (Configuration::GetInstance()->ifManualShift)
    {
        currentclutchindex = (int)ClutchState::Released; // 当前离合状态索引号
        currentshiftindex = (int)ManualShiftState::Gear_N; // 当前挡位状态索引号
        nextclutchindex = (int)ClutchState::Pressed; // 目标离合状态索引号
        nextshiftindex = (int)ManualShiftState::Gear_1; // 目标挡位状态索引号

        shiftexampause = false;
        clutchexampause = false;
        exampause = false;

        ui->pushButton_shiftpause->setText(" 暂 停 ");
        ui->pushButton_pause->setText(" 暂 停 ");
        ui->pushButton_clutchpause->setText(" 暂 停 ");

        ui->lineEdit_shiftnow->setText(ManualShiftStateString[currentshiftindex]);

        ifenablecomboBoxchangedeventhappen = false;
        ui->comboBox_shiftaim->setCurrentIndex(GetComboBoxIndexfromEnumIndexofShift(nextshiftindex));
        ifenablecomboBoxchangedeventhappen = true;

        ui->lineEdit_shifttrace->setText(GetShiftChangingRoute());
    }
    else
    {
        currentclutchindex = (int)ClutchState::Released; // 当前离合状态索引号
        currentshiftindex = (int)AutoShiftState::Gear_N; // 当前挡位状态索引号
        nextclutchindex = (int)ClutchState::Released; // 目标离合状态索引号
        nextshiftindex = (int)AutoShiftState::Gear_D; // 目标挡位状态索引号

        shiftexampause = false;
        clutchexampause = false;
        exampause = false;

        ui->pushButton_shiftpause->setText(" 暂 停 ");
        ui->pushButton_pause->setText(" 暂 停 ");
        ui->pushButton_clutchpause->setText(" 暂 停 ");

        ui->lineEdit_shiftnow->setText(AutoShiftStateString[currentshiftindex]);

        ifenablecomboBoxchangedeventhappen = false;
        ui->comboBox_shiftaim->setCurrentIndex(GetComboBoxIndexfromEnumIndexofShift(nextshiftindex));
        ifenablecomboBoxchangedeventhappen = true;

        ui->lineEdit_shifttrace->setText(GetShiftChangingRoute());
    }
    pauseflag = 0;

    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

    // 定时器打开
    examflag = -1;
    examtimer->start(RobotParams::UITimerMs);

    // 纠正到空挡 抬起离合
    // 开始测试
    examflag = 1;
}

void ShiftClutchUI::on_pushButton_stopexam_clicked()
{
    // 暂停下不能停止测试
    if (shiftexampause || clutchexampause || exampause)
    {
        QMessageBox::warning(NULL, tr("警告"), tr("测试处于暂停状态\r\n无法停止测试\r\n急停可以按'立即停止'！"));
        return;
    }

    // 停止测试视车辆状态分类
    if (RobotParams::powerMode == PedalRobot::Run)
    {
        if (RobotParams::canCarSpeed >1 || RobotParams::pulseCarSpeed > 1)
        {
            QMessageBox::warning(NULL, tr("警告"), tr("车辆发动且有速度\r\n请停车后再停止测试！"));
            return;
        }
        else
        {
            // 部分初始化
            if (Configuration::GetInstance()->ifManualShift)
            {
                nextshiftindex = (int)ManualShiftState::Gear_N;
            }
            else
            {
                nextshiftindex = (int)AutoShiftState::Gear_N;
            }

            ui->pushButton_stopexam->setEnabled(false);
            ui->tabWidget2->setEnabled(false);
            pauseflag = 0;

            examflag = 19;
        }
    }
    else
    {
        // 部分初始化
        if (Configuration::GetInstance()->ifManualShift)
        {
            nextshiftindex = (int)ManualShiftState::Gear_N;
        }
        else
        {
            nextshiftindex = (int)AutoShiftState::Gear_N;
        }

        ui->pushButton_stopexam->setEnabled(false);
        ui->tabWidget2->setEnabled(false);
        pauseflag = 0;

        examflag = 19;
    }
}

void ShiftClutchUI::on_pushButton_stopnow_clicked()
{
    // 重置换挡过程信息
//    SendShiftClutchInfo(true);

    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
    PRINTF(LOG_WARNING, "%s: immediately stop.\n", __func__);
    examtimer->stop();

    // 各标志位初始化
    shiftexampause = false;
    clutchexampause = false;
    exampause = false;
    pauseflag = 0;
    examflag = -1;

    // 退出测试标志位
    isExaming = false;

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
    essf << std::right << std::setw(15) << RobotParams::normalMotionAccuracy << "\n";
    essf << 'T' << "\n";
    essf << Configuration::GetInstance()->translateSpeed << "\n";
    essf << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[0];
    essf << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];

    if (Configuration::GetInstance()->ifManualShift)
    {
        essf << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[(int)ClutchState::Pressed];
        essf << std::right << std::setw(15) << RobotParams::angleRealTime[3];
        essf << std::right << std::setw(15) << RobotParams::angleRealTime[4];
        essf << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }
    else
    {
        essf << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[(int)ClutchState::Released];
        essf << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[(int)AutoShiftState::Gear_N];
        essf << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[(int)AutoShiftState::Gear_N];
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


void ShiftClutchUI::on_comboBox_shiftaim_currentIndexChanged(int index)
{
    if (ifenablecomboBoxchangedeventhappen)
    {
        if (examflag != 0) return;

        if (currentshiftindex == GetEnumIndexfromComboBoxIndexofShift(index))
        {
            ui->lineEdit_shifttrace->setText(" ----- ----- ");
            QMessageBox::information(NULL, tr("提示"), tr("已经到达位置！"));
            return;
        }

        nextshiftindex = GetEnumIndexfromComboBoxIndexofShift(index);
        ui->lineEdit_shifttrace->setText(GetShiftChangingRoute());
    }
}

void ShiftClutchUI::on_pushButton_shiftrun_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Run)
    {
        QMessageBox::information(NULL, tr("提示"), tr("车辆发动状态下该测试不被允许 ！"));
        return;
    }

    if (examflag != 0)
    {
        QMessageBox::information(NULL, tr("提示"), tr("正在执行换挡\r\n请待执行完毕再试 ！"));
        return;
    }

    examflag = 2;
    pauseflag = 0;
    ui->lineEdit_shifttime->setText("");
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

void ShiftClutchUI::on_pushButton_run3_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Run)
    {
        QMessageBox::information(NULL, tr("提示"), tr("车辆发动状态下该测试不被允许 ！"));
        return;
    }

    if (examflag != 0)
    {
        QMessageBox::information(NULL, tr("提示"), tr("正在执行换挡\r\n请待执行完毕再试 ！"));
        return;
    }

    examflag = 6;
    pauseflag = 0;
    ui->lineEdit_shifttime->setText("");
}

void ShiftClutchUI::on_pushButton_run5_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Run)
    {
        QMessageBox::information(NULL, tr("提示"), tr("车辆发动状态下该测试不被允许 ！"));
        return;
    }

    if (examflag != 0)
    {
        QMessageBox::information(NULL, tr("提示"), tr("正在执行换挡\r\n请待执行完毕再试 ！"));
        return;
    }

    examflag = 7;
    pauseflag = 0;
    ui->lineEdit_shifttime->setText("");
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

void ShiftClutchUI::on_pushButton_bottom_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Run)
    {
        QMessageBox::information(NULL, tr("提示"), tr("车辆发动状态下该测试不被允许 ！"));
        return;
    }

    if (examflag != 0)
    {
        QMessageBox::information(NULL, tr("提示"), tr("正在执行换挡\r\n请待执行完毕再试 ！"));
        return;
    }

    examflag = 3;
    pauseflag = 0;
    ui->lineEdit_shifttime->setText("");
}

void ShiftClutchUI::on_pushButton_top_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Run)
    {
        QMessageBox::information(NULL, tr("提示"), tr("车辆发动状态下该测试不被允许 ！"));
        return;
    }

    if (examflag != 0)
    {
        QMessageBox::information(NULL, tr("提示"), tr("正在执行换挡\r\n请待执行完毕再试 ！"));
        return;
    }

    examflag = 4;
    pauseflag = 0;
    ui->lineEdit_shifttime->setText("");
}

void ShiftClutchUI::on_pushButton_speed_clicked()
{
    if (RobotParams::powerMode == PedalRobot::Run)
    {
        QMessageBox::information(NULL, tr("提示"), tr("车辆发动状态下该测试不被允许 ！"));
        return;
    }

    if (examflag != 0)
    {
        QMessageBox::information(NULL, tr("提示"), tr("正在执行换挡\r\n请待执行完毕再试 ！"));
        return;
    }

    examflag = 5;
    pauseflag = 0;
    ui->lineEdit_shifttime->setText("");
    ui->checkBox_departure->setEnabled(false);
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

void ShiftClutchUI::on_pushButton_runtest_clicked()
{
    if (RobotParams::powerMode != PedalRobot::Run)
    {
        QMessageBox::information(NULL, tr("提示"), tr("只有车辆发动状态下该测试才被允许 ！"));
        return;
    }

    if (examflag != 0)
    {
        QMessageBox::information(NULL, tr("提示"), tr("正在执行换挡\r\n请待执行完毕再试 ！"));
        return;
    }

    examflag = 8;
    pauseflag = 0;
    ui->lineEdit_shifttime->setText("");
}

void ShiftClutchUI::on_pushButton_runtest_brkp_pressed()
{
    if (RobotParams::powerMode == PedalRobot::Run && examflag == 0)
    {
        relativebrk = relativevalue;
    }
}

void ShiftClutchUI::on_pushButton_runtest_brkp_released()
{
    if (RobotParams::powerMode == PedalRobot::Run && examflag == 0)
    {
        relativebrk = 0;
    }
}

void ShiftClutchUI::on_pushButton_runtest_brkm_pressed()
{
    if (RobotParams::powerMode == PedalRobot::Run && examflag == 0)
    {
        relativebrk = -relativevalue;
    }
}

void ShiftClutchUI::on_pushButton_runtest_brkm_released()
{
    if (RobotParams::powerMode == PedalRobot::Run && examflag == 0)
    {
        relativebrk = 0;
    }
}

void ShiftClutchUI::on_pushButton_runtest_accp_pressed()
{
    if (RobotParams::powerMode == PedalRobot::Run && examflag == 0)
    {
        relativeacc = relativevalue;
    }
}

void ShiftClutchUI::on_pushButton_runtest_accp_released()
{
    if (RobotParams::powerMode == PedalRobot::Run && examflag == 0)
    {
        relativeacc = 0;
    }
}

void ShiftClutchUI::on_pushButton_runtest_accm_pressed()
{
    if (RobotParams::powerMode == PedalRobot::Run && examflag == 0)
    {
        relativeacc = -relativevalue;
    }
}

void ShiftClutchUI::on_pushButton_runtest_accm_released()
{
    if (RobotParams::powerMode == PedalRobot::Run && examflag == 0)
    {
        relativeacc = 0;
    }
}


void ShiftClutchUI::SendMotionCmd(const double deltabrk, const double deltaacc, const bool ifshiftmode, const bool ifpause, const int aimshift, const int aimclutch, const int howtochangeshift, const int howtoreleaseclutch)
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

void ShiftClutchUI::SendShiftClutchInfo()
{
    std::vector<double> reservedParam;
    reservedParam.clear();
    reservedParam.reserve(64);

    reservedParam.push_back(Configuration::GetInstance()->ifManualShift ? 1.0 : 0.0);
    for (int i = 0; i < 11; ++i)
    {
        reservedParam.push_back(Configuration::GetInstance()->shiftAxisAngles1[i]);
        reservedParam.push_back(Configuration::GetInstance()->shiftAxisAngles2[i]);
    }
    for (int i = 0; i < 2; ++i)
    {
        reservedParam.push_back(Configuration::GetInstance()->clutchAngles[i]);
    }
    for (int i =0; i < 3; ++i)
    {
        reservedParam.push_back(Configuration::GetInstance()->angleErr_A[i]);
    }
    reservedParam.push_back(Configuration::GetInstance()->angleErr_P);
    for (int i = 0; i < 3; ++i)
    {
        reservedParam.push_back(Configuration::GetInstance()->curveMotionSpeed[i]);
    }
    reservedParam.push_back(Configuration::GetInstance()->clutchUpSpeedAtDeparture);
    reservedParam.push_back(Configuration::GetInstance()->clutchUpSpeed);
    reservedParam.push_back(Configuration::GetInstance()->startAccAngleValue);
    for (int i = 0; i < 2; ++i)
    {
        reservedParam.push_back(Configuration::GetInstance()->pedalRecoveryPercent[i]);
    }

    AutoDriveRobotApiClient::GetInstance()->Send_SetReservedParamConf(reservedParam);
}

void ShiftClutchUI::SendResetSignal()
{
    AutoDriveRobotApiClient::GetInstance()->Send_MessageInformMsg(1);
}

void ShiftClutchUI::SendEmergencyStopInfo()
{
    int emergencyStopType = !Configuration::GetInstance()->ifManualShift;

    std::vector<double> emergencyStopTheta;
    emergencyStopTheta.push_back( Configuration::GetInstance()->deathPos[0] );
    emergencyStopTheta.push_back( Configuration::GetInstance()->deathPos[1] );

    if (Configuration::GetInstance()->ifManualShift)
    {
        emergencyStopTheta.push_back( Configuration::GetInstance()->clutchAngles[(int)ClutchState::Pressed] );
        emergencyStopTheta.push_back( 0.0 );
        emergencyStopTheta.push_back( 0.0 );
    }
    else
    {
        emergencyStopTheta.push_back( Configuration::GetInstance()->clutchAngles[(int)ClutchState::Released] );
        emergencyStopTheta.push_back( Configuration::GetInstance()->shiftAxisAngles1[(int)AutoShiftState::Gear_N] );
        emergencyStopTheta.push_back( Configuration::GetInstance()->shiftAxisAngles2[(int)AutoShiftState::Gear_N] );
    }

    emergencyStopTheta.push_back( RobotParams::angleRealTime[5] );

    AutoDriveRobotApiClient::GetInstance()->Send_SetPedalRobotEmergencyStopThetaMsg(emergencyStopType, emergencyStopTheta);
}


bool ShiftClutchUI::ifPedalAtPositions(bool isbrklow, bool isacclow)
{
    double brkaim, accaim;
    if (isbrklow)
    {
        brkaim = Configuration::GetInstance()->deathPos[0];
    }
    else
    {
        brkaim = Configuration::GetInstance()->limPos[0];
    }
    if (isacclow)
    {
        accaim = Configuration::GetInstance()->deathPos[1];
    }
    else
    {
        accaim = Configuration::GetInstance()->limPos[1];
    }

    if (fabs(RobotParams::angleRealTime[0] - brkaim) < 0.5 && fabs(RobotParams::angleRealTime[1] - accaim) < 0.5)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool ShiftClutchUI::SetTempArrivalFile(QVector<QVector<double>> anglelist, int length)
{
    // 按照车型配置文件更新temparrival.txt
    std::fstream taf(Configuration::temparrivalFilePath.c_str(), std::fstream::out | std::fstream::binary);
    if(taf.fail()){
        PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, Configuration::temparrivalFilePath.c_str());
        return false;
    }
    taf << RobotParams::robotType << "\n";
    taf << 'R' << "\n";
    taf << std::right << std::setw(15) << 0;
    taf << std::right << std::setw(15) << 0;
    taf << std::right << std::setw(15) << 2000;
    taf << std::right << std::setw(15) << 0;
    taf << std::right << std::setw(15) << RobotParams::normalMotionAccuracy << "\n";

    for (int i = 0; i < length; ++i)
    {
        taf << 'T' << "\n";
        taf << Configuration::GetInstance()->translateSpeed << "\n";
        taf << std::right << std::setw(15) << anglelist[i][0];
        taf << std::right << std::setw(15) << anglelist[i][1];
        taf << std::right << std::setw(15) << anglelist[i][2];
        taf << std::right << std::setw(15) << anglelist[i][3];
        taf << std::right << std::setw(15) << anglelist[i][4];
        taf << std::right << std::setw(15) << anglelist[i][5] << "\n";
    }

    taf.close();

    return true;
}

bool ShiftClutchUI::ifReachedPositionbyHand(int index)
{
    double err[2] = {Configuration::GetInstance()->angleErr_M[1], Configuration::GetInstance()->angleErr_M[2]};

    if (fabs(RobotParams::angleRealTime[3] - Configuration::GetInstance()->shiftAxisAngles1[index]) < err[0] &&
            fabs(RobotParams::angleRealTime[4] - Configuration::GetInstance()->shiftAxisAngles2[index]) < err[1])
    {
        return true;
    }
    else
    {
        return false;
    }
}


QPair<double, double> ShiftClutchUI::CalOrthogonalFootOfPoints(QPair<double, double> p1, QPair<double, double> p2, QPair<double, double> pN)
{
    double XA = p1.first - p2.first;
    double XB = pN.first - p2.first;
    double YA = p1.second - p2.second;
    double YB = pN.second - p2.second;
    double k, x, y;
    if (XA == 0 && YA == 0)
    {
        k = 0; x = 0; y = 0;
    }
    else
    {
        k = (XA * XB + YA * YB) / (XA * XA + YA * YA);
        x = k * p1.first + (1 - k) * p2.first;
        y = k * p1.second + (1 - k) * p2.second;
    }

    return QPair<double, double>(x, y);
}

void ShiftClutchUI::AutoRecordAtManualShift()
{
    if (Configuration::GetInstance()->ifAutoRecordMidN)
    {
        QPair<double, double> shift_1(Configuration::GetInstance()->shiftAxisAngles1[1], Configuration::GetInstance()->shiftAxisAngles2[1]);
        QPair<double, double> shift_2(Configuration::GetInstance()->shiftAxisAngles1[2], Configuration::GetInstance()->shiftAxisAngles2[2]);
        QPair<double, double> shift_N(Configuration::GetInstance()->shiftAxisAngles1[0], Configuration::GetInstance()->shiftAxisAngles2[0]);

        QPair<double, double> shift_5(Configuration::GetInstance()->shiftAxisAngles1[5], Configuration::GetInstance()->shiftAxisAngles2[5]);
        QPair<double, double> shift_6(Configuration::GetInstance()->shiftAxisAngles1[9], Configuration::GetInstance()->shiftAxisAngles2[9]);

        QPair<double, double> shift_R(Configuration::GetInstance()->shiftAxisAngles1[10], Configuration::GetInstance()->shiftAxisAngles2[10]);

        QPair<double, double> NL = CalOrthogonalFootOfPoints(shift_1, shift_2, shift_N);
        Configuration::GetInstance()->shiftAxisAngles1[6] = NL.first;
        Configuration::GetInstance()->shiftAxisAngles2[6] = NL.second;

        QPair<double, double> NR;
        if (Configuration::GetInstance()->ifExistSixShift)
        {
            NR = CalOrthogonalFootOfPoints(shift_5, shift_6, shift_N);
        }
        else
        {
            NR = CalOrthogonalFootOfPoints(shift_N, NL, shift_5);
        }
        Configuration::GetInstance()->shiftAxisAngles1[7] = NR.first;
        Configuration::GetInstance()->shiftAxisAngles2[7] = NR.second;

        QPair<double, double> NB;
        if (Configuration::GetInstance()->ifExistBackShift)
        {
            NB = CalOrthogonalFootOfPoints(shift_N, NL, shift_R);
            Configuration::GetInstance()->shiftAxisAngles1[8] = NB.first;
            Configuration::GetInstance()->shiftAxisAngles2[8] = NB.second;
        }

        if (Configuration::GetInstance()->ifExistSixShift && Configuration::GetInstance()->ifExistBackShift)
        {
            ui->list1->item(9)->setText("NLeft\t\t\t\t " + QString::number(NL.first, 'f', 2) + "\t\t\t\t " + QString::number(NL.second, 'f', 2));
        }
        else if (!Configuration::GetInstance()->ifExistSixShift && !Configuration::GetInstance()->ifExistBackShift)
        {
            ui->list1->item(7)->setText("NLeft\t\t\t\t " + QString::number(NL.first, 'f', 2) + "\t\t\t\t " + QString::number(NL.second, 'f', 2));
        }
        else
        {
            ui->list1->item(8)->setText("NLeft\t\t\t\t " + QString::number(NL.first, 'f', 2) + "\t\t\t\t " + QString::number(NL.second, 'f', 2));
        }

        if (Configuration::GetInstance()->ifExistSixShift && Configuration::GetInstance()->ifExistBackShift)
        {
            ui->list1->item(10)->setText("NRight\t\t\t\t " + QString::number(NR.first, 'f', 2) + "\t\t\t\t " + QString::number(NR.second, 'f', 2));
        }
        else if (!Configuration::GetInstance()->ifExistSixShift && !Configuration::GetInstance()->ifExistBackShift)
        {
            ui->list1->item(8)->setText("NRight\t\t\t\t " + QString::number(NR.first, 'f', 2) + "\t\t\t\t " + QString::number(NR.second, 'f', 2));
        }
        else
        {
            ui->list1->item(9)->setText("NRight\t\t\t\t " + QString::number(NR.first, 'f', 2) + "\t\t\t\t " + QString::number(NR.second, 'f', 2));
        }

        if (Configuration::GetInstance()->ifExistBackShift)
        {
            if (Configuration::GetInstance()->ifExistSixShift)
            {
                ui->list1->item(11)->setText("NBack\t\t\t\t " + QString::number(NB.first, 'f', 2) + "\t\t\t\t " + QString::number(NB.second, 'f', 2));
            }
            else
            {
                ui->list1->item(10)->setText("NBack\t\t\t\t " + QString::number(NB.first, 'f', 2) + "\t\t\t\t " + QString::number(NB.second, 'f', 2));
            }
        }
    }
}

void ShiftClutchUI::ModifyShiftInfoItem(int itemindex, double angle1, double angle2)
{
    if (Configuration::GetInstance()->ifManualShift)
    {
        switch (itemindex)
        {
        case 0:
            ui->list1->item(itemindex + 1)->setText("N    \t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            break;
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
            ui->list1->item(itemindex + 1)->setText(QString::number(itemindex) + "    \t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            break;
        case 7:
            if (Configuration::GetInstance()->ifExistSixShift)
            {
                ui->list1->item(itemindex + 1)->setText("R    \t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            else
            {
                ui->list1->item(itemindex)->setText("R    \t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            break;
        case 8:
            if (Configuration::GetInstance()->ifExistSixShift && Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->item(itemindex + 1)->setText("NLeft\t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            else if (!Configuration::GetInstance()->ifExistSixShift && !Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->item(itemindex - 1)->setText("NLeft\t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            else
            {
                ui->list1->item(itemindex)->setText("NLeft\t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            break;
        case 9:
            if (Configuration::GetInstance()->ifExistSixShift && Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->item(itemindex + 1)->setText("NRight\t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            else if (!Configuration::GetInstance()->ifExistSixShift && !Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->item(itemindex - 1)->setText("NRight\t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            else
            {
                ui->list1->item(itemindex)->setText("NRight\t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            break;
        case 10:
            if (Configuration::GetInstance()->ifExistSixShift)
            {
                ui->list1->item(itemindex + 1)->setText("NBack\t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            else
            {
                ui->list1->item(itemindex)->setText("NBack\t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            break;
        default:
            break;
        }

        // 自动补齐
        AutoRecordAtManualShift();
    }
    else
    {
        switch (itemindex)
        {
        case 0:
            ui->list1->item(itemindex + 1)->setText("P    \t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            break;
        case 1:
            ui->list1->item(itemindex + 1)->setText("R    \t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            break;
        case 2:
            if (Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->item(itemindex + 1)->setText("N    \t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            else
            {
                ui->list1->item(itemindex)->setText("N    \t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            break;
        case 3:
            if (Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->item(itemindex + 1)->setText("D    \t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            else
            {
                ui->list1->item(itemindex)->setText("D    \t\t\t\t " + QString::number(angle1, 'f', 2) + "\t\t\t\t " + QString::number(angle2, 'f', 2));
            }
            break;
        default:
            break;
        }
    }
}

void ShiftClutchUI::ModifyClutchInfoItem(int itemindex, double info)
{
    if (Configuration::GetInstance()->ifManualShift)
    {
        switch (itemindex) {
        case 0:
            ui->list2->item(itemindex + 1)->setText("踩住位置\t\t\t\t\t\t " + QString::number(info, 'f', 2) + "\t\t\t\t\t\t / ");
            break;
        case 1:
            ui->list2->item(itemindex + 1)->setText("松开位置\t\t\t\t\t\t " + QString::number(info, 'f', 2) + "\t\t\t\t\t\t / ");
            break;
        case 2:
            ui->list2->item(itemindex + 1)->setText("松开速度\t\t\t\t\t\t /    \t\t\t\t\t\t " + QString::number(info, 'f', 2));
            break;
        case 3:
            ui->list2->item(itemindex + 1)->setText("慢抬速度\t\t\t\t\t\t /    \t\t\t\t\t\t " + QString::number(info, 'f', 2));
            break;
        default:
            break;
        }
    }
}


int ShiftClutchUI::GetNodeofShiftStatus(int givenshiftstatus)
{
    int finalnode;
    ManualShiftState usedstatus = ManualShiftState(givenshiftstatus);

    switch (usedstatus) {
    case ManualShiftState::Gear_NLeft:
    case ManualShiftState::Gear_1:
    case ManualShiftState::Gear_2:
        finalnode =(int) ManualShiftState::Gear_NLeft;
        break;
    case ManualShiftState::Gear_N:
    case ManualShiftState::Gear_3:
    case ManualShiftState::Gear_4:
        finalnode =(int) ManualShiftState::Gear_N;
        break;
    case ManualShiftState::Gear_NRight:
    case ManualShiftState::Gear_5:
    case ManualShiftState::Gear_6:
        finalnode =(int) ManualShiftState::Gear_NRight;
        break;
    case ManualShiftState::Gear_NBack:
    case ManualShiftState::Gear_R:
        finalnode =(int) ManualShiftState::Gear_NBack;
        break;
    default:
        PRINTF(LOG_WARNING, "%s: no matched nodeshift.\n", __func__);
        finalnode = 0;
        break;
    }

    return finalnode;
}

QString ShiftClutchUI::GetShiftChangingRoute()
{
    QString changingroutestring;
    if (Configuration::GetInstance()->ifManualShift)
    {
        changingroutestring = ManualShiftStateString[currentshiftindex];
    }
    else
    {
        changingroutestring = AutoShiftStateString[currentshiftindex];
    }
    QVector<int> changingrouteindexchain;
    const int currentpos = currentshiftindex;
    const int finalaim = nextshiftindex;

    if (Configuration::GetInstance()->ifManualShift)
    {
        const int nodeforcurrentpos = GetNodeofShiftStatus(currentpos);
        const int nodeforfinalaim = GetNodeofShiftStatus(finalaim);

        if (nodeforcurrentpos == nodeforfinalaim)
        {
            changingrouteindexchain.append(finalaim);
        }
        else
        {
            if (nodeforcurrentpos == currentpos)
            {
                changingrouteindexchain.append(nodeforfinalaim);
                changingrouteindexchain.append(finalaim);
            }
            else if (nodeforfinalaim == finalaim)
            {
                changingrouteindexchain.append(nodeforcurrentpos);
                changingrouteindexchain.append(finalaim);
            }
            else
            {
                changingrouteindexchain.append(nodeforcurrentpos);
                changingrouteindexchain.append(nodeforfinalaim);
                changingrouteindexchain.append(finalaim);
            }
        }
    }
    else
    {
        changingrouteindexchain.append(finalaim);
    }

    QVector<int>::iterator iter;
    if (Configuration::GetInstance()->ifManualShift)
    {
        for (iter=changingrouteindexchain.begin();iter!=changingrouteindexchain.end();iter++)
        {
            changingroutestring += (QString(" ---> ") + ManualShiftStateString[*iter]);
        }
    }
    else
    {
        for (iter=changingrouteindexchain.begin();iter!=changingrouteindexchain.end();iter++)
        {
            changingroutestring += (QString(" ---> ") + AutoShiftStateString[*iter]);
        }
    }

    return changingroutestring;
}

int ShiftClutchUI::GetComboBoxIndexfromEnumIndexofShift(int enumindex)
{
    int result = 0;

    if (Configuration::GetInstance()->ifManualShift)
    {
        if (enumindex > 8)
        {
            result = enumindex - 3;
        }
        else if (enumindex > 5)
        {
            result = enumindex + 2;
        }
        else
        {
            result = enumindex;
        }
    }
    else
    {
        if (enumindex > 2)
        {
            result = enumindex - 2;
        }
        else if (enumindex > 0)
        {
            result = enumindex + 1;
        }
        else
        {
            result = enumindex;
        }
    }

    return result;
}

int ShiftClutchUI::GetEnumIndexfromComboBoxIndexofShift(int comboboxindex)
{
    int result = 0;

    if (Configuration::GetInstance()->ifManualShift)
    {
        if (comboboxindex > 7)
        {
            result = comboboxindex - 2;
        }
        else if (comboboxindex > 5)
        {
            result = comboboxindex + 3;
        }
        else
        {
            result = comboboxindex;
        }
    }
    else
    {
        if (comboboxindex > 1)
        {
            result = comboboxindex - 1;
        }
        else if (comboboxindex > 0)
        {
            result = comboboxindex + 2;
        }
        else
        {
            result = comboboxindex;
        }
    }

    return result;
}

void ShiftClutchUI::UnenableComboBoxItem(QComboBox *cBox, int num)
{
    cBox->setItemData(num, 0, Qt::UserRole - 1);
    cBox->setItemData(num, QColor(230, 230, 230), Qt::BackgroundColorRole);
}


void ShiftClutchUI::RefreshShiftMap()
{
    QPixmapCache::clear();
    QString picpath;
    if (Configuration::GetInstance()->ifManualShift)
    {
        if (Configuration::GetInstance()->ifExistSixShift)
        {
            if (Configuration::GetInstance()->ifExistBackShift)
            {
                picpath = QString::fromStdString(Configuration::mainFolder) + QString("/manualshift_6R.png");
            }
            else
            {
                picpath = QString::fromStdString(Configuration::mainFolder) + QString("/manualshift_6_noR.png");
            }
        }
        else
        {
            if (Configuration::GetInstance()->ifExistBackShift)
            {
                picpath = QString::fromStdString(Configuration::mainFolder) + QString("/manualshift_R_no6.png");
            }
            else
            {
                picpath = QString::fromStdString(Configuration::mainFolder) + QString("/manualshift_no6R.png");
            }
        }
    }
    else
    {
        if (Configuration::GetInstance()->ifExistBackShift)
        {
            picpath = QString::fromStdString(Configuration::mainFolder) + QString("/autoshift_R.png");
        }
        else
        {
            picpath = QString::fromStdString(Configuration::mainFolder) + QString("/autoshift_noR.png");
        }
    }
    pic.load(picpath);
    ui->label_pic->setPixmap(pic);
}


void ShiftClutchUI::RefreshCarTypeShowing()
{
    ui->lineEdit_type->setText( QString::fromStdString( Configuration::GetInstance()->carTypeName.substr(0, Configuration::GetInstance()->carTypeName.length() - 4) ) );
}

void ShiftClutchUI::RefreshCarGearTypeShowing()
{
    ifenablewaychangedeventhappen = false;
    ui->comboBox_way->setCurrentIndex(!Configuration::GetInstance()->ifManualShift);
    ifenablewaychangedeventhappen =  true;
}

void ShiftClutchUI::RefreshConfirmShiftClutchInfoState()
{
    ui->pushButton_confirmSC->setEnabled(!ifConfirmShiftClutchInfo);
}

void ShiftClutchUI::RefreshMainControlStrip()
{
    RefreshCarTypeShowing();
    RefreshCarGearTypeShowing();
    RefreshConfirmShiftClutchInfoState();
}


void ShiftClutchUI::RefreshShiftLists(bool enableshift, bool enableclutch)
{
    // 清空
    ui->list1->clear();
    ui->list2->clear();

    if (enableshift)
    {
        // 更新List1
        if (Configuration::GetInstance()->ifManualShift)
        {
            ui->list1->addItem(tr("挡位\t\t\t\t角度1/deg\t\t\t角度2/deg"));
            ui->list1->addItem( tr( (QString("N    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_N], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_N], 'f', 2)).toStdString().c_str() ) );
            ui->list1->addItem( tr( (QString("1    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_1], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_1], 'f', 2)).toStdString().c_str() ) );
            ui->list1->addItem( tr( (QString("2    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_2], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_2], 'f', 2)).toStdString().c_str() ) );
            ui->list1->addItem( tr( (QString("3    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_3], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_3], 'f', 2)).toStdString().c_str() ) );
            ui->list1->addItem( tr( (QString("4    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_4], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_4], 'f', 2)).toStdString().c_str() ) );
            ui->list1->addItem( tr( (QString("5    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_5], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_5], 'f', 2)).toStdString().c_str() ) );

            if (Configuration::GetInstance()->ifExistSixShift)
            {
                ui->list1->addItem( tr( (QString("6    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_6], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_6], 'f', 2)).toStdString().c_str() ) );
            }

            if (Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->addItem( tr( (QString("R    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_R], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_R], 'f', 2)).toStdString().c_str() ) );
            }

            ui->list1->addItem( tr( (QString("NLeft\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_NLeft], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_NLeft], 'f', 2)).toStdString().c_str() ) );
            ui->list1->addItem( tr( (QString("NRight\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_NRight], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_NRight], 'f', 2)).toStdString().c_str() ) );

            if (Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->addItem( tr( (QString("NBack\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)ManualShiftState::Gear_NBack], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)ManualShiftState::Gear_NBack], 'f', 2)).toStdString().c_str() ) );
            }
        }
        else
        {
            ui->list1->addItem( tr("挡位\t\t\t\t角度1/deg\t\t\t角度2/deg"));
            ui->list1->addItem( tr( (QString("P    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)AutoShiftState::Gear_P], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)AutoShiftState::Gear_P], 'f', 2)).toStdString().c_str() ) );

            if (Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->addItem( tr( (QString("R    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)AutoShiftState::Gear_R], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)AutoShiftState::Gear_R], 'f', 2)).toStdString().c_str() ) );
            }

            ui->list1->addItem( tr( (QString("N    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)AutoShiftState::Gear_N], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)AutoShiftState::Gear_N], 'f', 2)).toStdString().c_str() ) );
            ui->list1->addItem( tr( (QString("D    \t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles1[(int)AutoShiftState::Gear_D], 'f', 2) + QString("\t\t\t\t ") + QString::number(Configuration::GetInstance()->shiftAxisAngles2[(int)AutoShiftState::Gear_D], 'f', 2)).toStdString().c_str() ) );
        }
    }

    if(enableclutch)
    {
        // 更新List2
        ui->list2->addItem(tr("位置\t\t\t\t\t\t\t角度/deg\t\t\t\t\t速度"));
        ui->list2->addItem( tr( (QString("踩住位置\t\t\t\t\t\t ") + QString::number(Configuration::GetInstance()->clutchAngles[(int)ClutchState::Pressed], 'f', 2) + QString("\t\t\t\t\t\t / ")).toStdString().c_str() ) );
        ui->list2->addItem( tr( (QString("松开位置\t\t\t\t\t\t ") + QString::number(Configuration::GetInstance()->clutchAngles[(int)ClutchState::Released], 'f', 2) + QString("\t\t\t\t\t\t / ")).toStdString().c_str() ) );
        ui->list2->addItem( tr( (QString("松开速度\t\t\t\t\t\t /    \t\t\t\t\t\t ") + QString::number(Configuration::GetInstance()->clutchUpSpeed, 'f', 2)).toStdString().c_str() ) );
        ui->list2->addItem( tr( (QString("慢抬速度\t\t\t\t\t\t /    \t\t\t\t\t\t ") + QString::number(Configuration::GetInstance()->clutchUpSpeedAtDeparture, 'f', 2)).toStdString().c_str() ) );
    }
}

void ShiftClutchUI::RefreshShiftComboBox()
{
    ifenablecomboBoxchangedeventhappen = false;

    // 清空ComboBox
    ui->comboBox_shift->clear();
    ui->comboBox_shiftaim->clear();

    if (Configuration::GetInstance()->ifManualShift)
    {
        ui->comboBox_shift->insertItem(0, "N");
        ui->comboBox_shift->insertItem(1, "1");
        ui->comboBox_shift->insertItem(2, "2");
        ui->comboBox_shift->insertItem(3, "3");
        ui->comboBox_shift->insertItem(4, "4");
        ui->comboBox_shift->insertItem(5, "5");
        ui->comboBox_shift->insertItem(6, "6");
        ui->comboBox_shift->insertItem(7, "R");
        ui->comboBox_shift->insertItem(8, "NLeft");
        ui->comboBox_shift->insertItem(9, "NRight");
        ui->comboBox_shift->insertItem(10, "NBack");

        ui->comboBox_shiftaim->insertItem(0, "N");
        ui->comboBox_shiftaim->insertItem(1, "1");
        ui->comboBox_shiftaim->insertItem(2, "2");
        ui->comboBox_shiftaim->insertItem(3, "3");
        ui->comboBox_shiftaim->insertItem(4, "4");
        ui->comboBox_shiftaim->insertItem(5, "5");
        ui->comboBox_shiftaim->insertItem(6, "6");
        ui->comboBox_shiftaim->insertItem(7, "R");

        if (!Configuration::GetInstance()->ifExistSixShift)
        {
            UnenableComboBoxItem(ui->comboBox_shift, 6);
            UnenableComboBoxItem(ui->comboBox_shiftaim, 6);
        }

        if (!Configuration::GetInstance()->ifExistBackShift)
        {
            UnenableComboBoxItem(ui->comboBox_shift, 7);
            UnenableComboBoxItem(ui->comboBox_shiftaim, 7);
            UnenableComboBoxItem(ui->comboBox_shift, 10);
        }
    }
    else
    {
        ui->comboBox_shift->insertItem(0, "P");
        ui->comboBox_shift->insertItem(1, "R");
        ui->comboBox_shift->insertItem(2, "N");
        ui->comboBox_shift->insertItem(3, "D");


        ui->comboBox_shiftaim->insertItem(0, "P");
        ui->comboBox_shiftaim->insertItem(1, "R");
        ui->comboBox_shiftaim->insertItem(2, "N");
        ui->comboBox_shiftaim->insertItem(3, "D");

        if (!Configuration::GetInstance()->ifExistBackShift)
        {
            UnenableComboBoxItem(ui->comboBox_shift, 1);
            UnenableComboBoxItem(ui->comboBox_shiftaim, 1);
        }
    }

    ifenablecomboBoxchangedeventhappen = true;
}

void ShiftClutchUI::RefreshShiftSixAndBack()
{
    ifenablecheckBoxchangedeventhappen = false;

    ui->checkBox_sixshift->setChecked(Configuration::GetInstance()->ifExistSixShift);
    ui->checkBox_backshift->setChecked(Configuration::GetInstance()->ifExistBackShift);

    ui->checkBox_sixshift->setEnabled(Configuration::GetInstance()->ifManualShift);

    ifenablecheckBoxchangedeventhappen = true;
}

void ShiftClutchUI::RefreshShiftAutoSet()
{
    ui->checkBox_autoset->setChecked(Configuration::GetInstance()->ifAutoRecordMidN);
    ui->checkBox_autoset->setEnabled(Configuration::GetInstance()->ifManualShift);
}

void ShiftClutchUI::RefreshClutchSpeed()
{
    int indexofclutchcomboBox = ui->comboBox_clutch->currentIndex();

    if (indexofclutchcomboBox > 1)
    {
        ui->lineEdit_speed->setEnabled(true);
        if (indexofclutchcomboBox == 2)
        {
            ui->lineEdit_speed->setText( QString::number(Configuration::GetInstance()->clutchUpSpeed, 'f', 2) );
        }
        else
        {
            ui->lineEdit_speed->setText( QString::number(Configuration::GetInstance()->clutchUpSpeedAtDeparture, 'f', 2) );
        }
    }
    else
    {
        ui->lineEdit_speed->setEnabled(false);
    }
}

void ShiftClutchUI::RefreshShiftClutchTeachPanel()
{
    RefreshShiftLists();
    RefreshShiftComboBox();
    RefreshShiftSixAndBack();
    RefreshShiftAutoSet();
    RefreshClutchSpeed();

    if (Configuration::GetInstance()->ifManualShift)
    {
        ui->tab_clutch->setEnabled(true);
    }
    else
    {
        ui->tab_clutch->setEnabled(false);
    }
}


void ShiftClutchUI::RefreshExamControlStrip()
{
    ui->pushButton_stopnow->setEnabled(true);

    if (isExaming)
    {
        ui->pushButton_startexam->setEnabled(false);
        ui->pushButton_stopexam->setEnabled(true);
    }
    else
    {
        ui->pushButton_startexam->setEnabled(true);
        ui->pushButton_stopexam->setEnabled(false);
    }
}

void ShiftClutchUI::RefreshExamShiftPanel()
{
    if (Configuration::GetInstance()->ifManualShift)
    {
        ui->lineEdit_shiftnow->setText(ManualShiftStateString[currentshiftindex]);

        ifenablecomboBoxchangedeventhappen = false;
        ui->comboBox_shiftaim->setCurrentIndex(GetComboBoxIndexfromEnumIndexofShift(nextshiftindex));
        ifenablecomboBoxchangedeventhappen = true;

        ui->lineEdit_shifttrace->setText(GetShiftChangingRoute());

        ui->lineEdit_shifttime->setText("");

        ui->pushButton_run3->setEnabled(true);
        ui->pushButton_run5->setEnabled(true);
        ui->pushButton_pause->setEnabled(true);

        ui->pushButton_runtest->setEnabled(true);
        ui->pushButton_runtest_brkp->setEnabled(true);
        ui->pushButton_runtest_brkm->setEnabled(true);
        ui->pushButton_runtest_accp->setEnabled(true);
        ui->pushButton_runtest_accm->setEnabled(true);
        ui->radioButton_recovery->setEnabled(true);
        ui->radioButton_keep->setEnabled(true);

        if (Configuration::GetInstance()->accMotionAtClutchReleasing == 1)
        {
            ui->radioButton_keep->setChecked(true);
            ui->radioButton_recovery->setChecked(false);
        }
        else
        {
            ui->radioButton_keep->setChecked(false);
            ui->radioButton_recovery->setChecked(true);
        }
    }
    else
    {
        ui->lineEdit_shiftnow->setText(AutoShiftStateString[currentshiftindex]);

        ifenablecomboBoxchangedeventhappen = false;
        ui->comboBox_shiftaim->setCurrentIndex(GetComboBoxIndexfromEnumIndexofShift(nextshiftindex));
        ifenablecomboBoxchangedeventhappen = true;

        ui->lineEdit_shifttrace->setText(GetShiftChangingRoute());

        ui->lineEdit_shifttime->setText("");

        ui->pushButton_run3->setEnabled(false);
        ui->pushButton_run5->setEnabled(false);
        ui->pushButton_pause->setEnabled(false);

        ui->pushButton_runtest->setEnabled(false);
        ui->pushButton_runtest_brkp->setEnabled(false);
        ui->pushButton_runtest_brkm->setEnabled(false);
        ui->pushButton_runtest_accp->setEnabled(false);
        ui->pushButton_runtest_accm->setEnabled(false);
        ui->radioButton_recovery->setEnabled(false);
        ui->radioButton_keep->setEnabled(false);
    }
}

void ShiftClutchUI::RefreshExamClutchPanel()
{
    if (Configuration::GetInstance()->ifManualShift)
    {
        ui->tab_examclutch->setEnabled(true);
        ui->lineEdit_clutchtime->setText("");
    }
    else
    {
        ui->tab_examclutch->setEnabled(false);
    }
}

void ShiftClutchUI::RefreshExamPanel()
{
    RefreshExamControlStrip();
    RefreshExamShiftPanel();
    RefreshExamClutchPanel();

    ui->tabWidget2->setEnabled(false);
}

void ShiftClutchUI::RefreshConfirmChangeShiftTimeState()
{
    ui->pushButton_confirmshiftchangingtime->setEnabled(!ifConfirmChangingShiftTime);
}

void ShiftClutchUI::RefreshChangeShiftPlot()
{
    ui->plotwidget->clearGraphs();
    ui->plotwidget->clearItems();
    ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes);
    ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
    ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));
    ui->plotwidget->xAxis->setRange(0,1800);
    ui->plotwidget->yAxis->setRange(-5, 140);
    ui->plotwidget->replot();
}

void ShiftClutchUI::RefreshChangeShiftPanel()
{
    RefreshConfirmChangeShiftTimeState();
    RefreshChangeShiftPlot();
    if (Configuration::GetInstance()->ifManualShift)
    {
        ui->tab_changeshift->setEnabled(true);
    }
    else
    {
        ui->tab_changeshift->setEnabled(false);
    }
}


void ShiftClutchUI::SetZeroShiftClutchInfo(bool enableshift, bool enableclutch)
{
    // 清空
    ui->list1->clear();
    ui->list2->clear();

    if (enableshift)
    {
        // 更新List1
        if (Configuration::GetInstance()->ifManualShift)
        {
            ui->list1->addItem(tr("挡位\t\t\t\t角度1/deg\t\t\t角度2/deg"));
            ui->list1->addItem( tr( QString("N    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            ui->list1->addItem( tr( QString("1    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            ui->list1->addItem( tr( QString("2    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            ui->list1->addItem( tr( QString("3    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            ui->list1->addItem( tr( QString("4    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            ui->list1->addItem( tr( QString("5    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );

            if (Configuration::GetInstance()->ifExistSixShift)
            {
                ui->list1->addItem( tr( QString("6    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            }

            if (Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->addItem( tr( QString("R    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            }

            ui->list1->addItem( tr( QString("NLeft\t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            ui->list1->addItem( tr( QString("NRight\t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );

            if (Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->addItem( tr( QString("NBack\t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            }
        }
        else
        {
            ui->list1->addItem( tr("挡位\t\t\t\t角度1/deg\t\t\t角度2/deg"));
            ui->list1->addItem( tr( QString("P    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );

            if (Configuration::GetInstance()->ifExistBackShift)
            {
                ui->list1->addItem( tr( QString("R    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            }

            ui->list1->addItem( tr( QString("N    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
            ui->list1->addItem( tr( QString("D    \t\t\t\t 00.00\t\t\t\t 00.00").toStdString().c_str() ) );
        }
    }

    if (enableclutch)
    {
        // 更新List2
        ui->list2->addItem(tr("位置\t\t\t\t\t\t\t角度/deg\t\t\t\t\t速度"));
        ui->list2->addItem( tr( QString("踩住位置\t\t\t\t\t\t 00.00\t\t\t\t\t\t / ").toStdString().c_str() ) );
        ui->list2->addItem( tr( QString("松开位置\t\t\t\t\t\t 00.00\t\t\t\t\t\t / ").toStdString().c_str() ) );
        ui->list2->addItem( tr( QString("松开速度\t\t\t\t\t\t /    \t\t\t\t\t\t 0.00").toStdString().c_str() ) );
        ui->list2->addItem( tr( QString("慢抬速度\t\t\t\t\t\t /    \t\t\t\t\t\t 0.00").toStdString().c_str() ) );
    }

    // 更新挡位离合信息
    if (enableshift)
    {
        for (int i=0; i<11; ++i)
        {
            Configuration::GetInstance()->shiftAxisAngles1[i] = 0.0;
            Configuration::GetInstance()->shiftAxisAngles2[i] = 0.0;
        }
    }

    if (enableclutch)
    {
        for (int i=0; i<2; ++i)
        {
            Configuration::GetInstance()->clutchAngles[i] = 0.0;
        }
        Configuration::GetInstance()->clutchUpSpeed = 0.0;
        Configuration::GetInstance()->clutchUpSpeedAtDeparture = 0.0;
    }
}

void ShiftClutchUI::SwitchConfirmShiftClutchInfoState()
{
    ifConfirmShiftClutchInfo = !ifConfirmShiftClutchInfo;
    RefreshConfirmShiftClutchInfoState();
}


void ShiftClutchUI::InitialUI()
{
    RefreshShiftMap();
    RefreshMainControlStrip();
    RefreshShiftClutchTeachPanel();
    RefreshExamPanel();
    RefreshChangeShiftPanel();
}

void ShiftClutchUI::SendMoveCommandAll(QVector<double> values, QVector<int> cmdstate, const int customvariable)
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

void ShiftClutchUI::ResolveAndSendCmd(double *cmd)
{
    QVector<double> values = {0, 0, 0, 0, 0, 0};
    QVector<int> cmdstate = {0, 0, 0, 0, 0, 0};

    for (int i = 0; i < 6; i++)
    {
        if (cmd[i] > -99)
        {
            values[i] = cmd[i];
            cmdstate[i] = 1;
        }
    }

    SendMoveCommandAll(values, cmdstate);
}

void ShiftClutchUI::ResolveAndShowTime(double totaltime, double *partialtime, bool ifclutchused)
{
    QString strtime;

    if (totaltime > 50)
    {
        strtime = QString::number(partialtime[0], 'f', 0);
    }
    else
    {
        return;
    }

    for (int i = 1; i < 5; ++i)
    {
        if (partialtime[i] > 50)
        {
            strtime += QString(" + ");
            strtime += QString::number(partialtime[i], 'f', 0);
        }
        else
        {
            break;
        }
    }

    strtime += QString(" = ");
    strtime += QString::number(totaltime, 'f', 0);

    if (ifclutchused)
    {
        ui->lineEdit_clutchtime->setText(strtime);
    }
    else
    {
        ui->lineEdit_shifttime->setText(strtime);
    }

    return;
}

void ShiftClutchUI::on_pushButton_readcurve_clicked()
{
    ifConfirmChangingShiftTime = false;
    RefreshChangeShiftPanel();

    //1) 选择曲线文件
    QString qstr = QFileDialog::getOpenFileName(NULL, QString("请选择实验曲线文件"), (Configuration::mainFolder+"/stdand_files/").c_str());
    if(qstr==""){
        return;
    }else if( !QFile::exists(qstr) ) {
        QMessageBox::information(NULL, "Warning", QObject::tr("所选定的文件不存在，请重新选择！")) ;
        return;
    }
    std::string str = qstr.toStdString();

    if( !QFile::exists(str.c_str()) ){
        QMessageBox::information(NULL, "Warning", QObject::tr("用于机器人控制的文件不存在:\n") + str.c_str());
        return;
    }

    if(ReadCurveFile(str) == -1){
        QMessageBox::information(NULL,"warning",QString("无法读取该目标曲线文件!")+str.c_str());
        return;
    }

    ShowCurveForSee();
}

int ShiftClutchUI::ReadCurveFile(const std::string &fileName)
{
    std::ifstream fs(fileName.c_str(), std::fstream::binary);
    PRINTF(LOG_DEBUG, "%s: read file = %s\n",__func__, fileName.c_str());
    if(fs.fail()){
        PRINTF(LOG_DEBUG, "%s: fail to open teach file\n",__func__);
        return -1;
    }

    manaulShiftSpeedTable.clear();
    manaulShiftIndexTable.clear();

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
                fs.getline(buf,1024);//R段的内容对于UI无用
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
                manaulShiftSpeedTable.clear();
                fs>>pointsNum;
                manaulShiftSpeedTable.reserve(pointsNum);

                while(!fs.eof()){
                    fs>>time>>speed;
                    manaulShiftSpeedTable.push_back( std::make_pair(time,speed) );
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

void ShiftClutchUI::ShowCurveForSee()
{
    ui->plotwidget->clearGraphs();
    ui->plotwidget->clearItems();
    ui->plotwidget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend);
    ui->plotwidget->xAxis->setLabel(QObject::tr("时间(s)"));
    ui->plotwidget->yAxis->setLabel(QObject::tr("速度(km/h)"));

    QPen myPen;
    myPen.setWidth(3);
    //0) 目标曲线
    ui->plotwidget->addGraph();
    myPen.setColor(Qt::red);
    ui->plotwidget->graph(0)->setPen(myPen);
    //1) 换挡时刻
    QCPGraph * scatterGraph = ui->plotwidget->addGraph();
    scatterGraph->setLineStyle(QCPGraph::lsNone);
    scatterGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 10));
    myPen.setColor(Qt::green);
    ui->plotwidget->graph(1)->setPen(myPen);

    ui->plotwidget->xAxis->setRange(manaulShiftSpeedTable[0].first, manaulShiftSpeedTable[manaulShiftSpeedTable.size() - 1].first);
    double maxSpeed = -200;
    double minSpeed = 200;
    foreach (auto item, manaulShiftSpeedTable) {
        if (item.second > maxSpeed) maxSpeed = item.second;
        if (item.second < minSpeed) minSpeed = item.second;
    }
    ui->plotwidget->yAxis->setRange(minSpeed-1, maxSpeed+1);

    size_t sz= manaulShiftSpeedTable.size();
    QVector<double> time, speed;
    for (size_t i = 0; i < sz;++i)
    {
        time.push_back(manaulShiftSpeedTable[i].first);
        speed.push_back(manaulShiftSpeedTable[i].second);
    }
    ui->plotwidget->graph(0)->setData(time, speed);

    size_t sz2= manaulShiftIndexTable.size();
    QVector<double> time2, indexSpeed;
    size_t lastIndex = 1;
    for (size_t i = 0; i < sz2; ++i)
    {
        time2.push_back(manaulShiftIndexTable[i].first);
        for (size_t j = lastIndex; j < sz; ++j)
        {
            if (manaulShiftIndexTable[i].first <= manaulShiftSpeedTable[j].first)
            {
                indexSpeed.push_back(manaulShiftSpeedTable[j].second);
                lastIndex = j;
                break;
            }
        }

        QCPItemText *shiftText = new QCPItemText(ui->plotwidget);
        shiftText->position->setCoords(time2[i], indexSpeed[i] + 7);
        shiftText->setPositionAlignment(Qt::AlignVCenter|Qt::AlignHCenter);
        shiftText->setText("[" + QString::number(time2[i]) + "秒]\r\n切" + ManualShiftStateString[manaulShiftIndexTable[i].second] + "挡");
        shiftText->setFont(QFont(font().family(), 10));
    }
    ui->plotwidget->graph(1)->setData(time2, indexSpeed);

    ui->plotwidget->replot();
}

void ShiftClutchUI::on_pushButton_confirmshiftchangingtime_clicked()
{
    int ret = QMessageBox::information(NULL, tr("提示"), tr("请确认换挡时刻信息！"), tr("确认"), tr("取消"));
    if(ret == 0)
    {
        ifConfirmChangingShiftTime = true;
        RefreshConfirmChangeShiftTimeState();
    }
}












