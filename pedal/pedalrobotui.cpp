#include "pedalrobotui.h"
#include "ui_pedalrobotui.h"

#include <fstream>
#include <QFileDialog>
#include <iostream>
#include <iomanip>

#include "pedalrobot.h"
#include "printf.h"
#include "robotparams.h"
#include "fileoperation/normalfile.h"
#include "autodriverobotapiclient.h"
#include "robotapi/AssistantFunc/fileassistantfunc.h"
#include "settingwidget/settingwidgetpedalrobotgetspeed.h"

const double speedCurveSpeedErrorLimit = 2.0; // NEDC/WLTC的曲线的速度误差限制

PedalRobotUI::PedalRobotUI(QWidget *parent, QLabel *_status, QLabel *_time) :
    QWidget(parent),
    ui(new Ui::PedalRobotUI),
    statusL(_status),
    timeL(_time),
    m_timerMutex(),
    m_enableMutex(false),
    ifSendGoHome(false),
    GoHomeRound(RobotParams::waitForGoHomeRound)
{
    ui->setupUi(this);

    ui->tabWidget->installEventFilter(this);

    conf = Configuration::GetInstance();

    scui = new ShiftClutchUI(this);
    ui->tabWidget->addTab( (QWidget*)scui, tr(" 换 挡 ") );

    stui = new SettingUI(this);
    ui->tabWidget->addTab( (QWidget*)stui, tr(" 设 置 ") );

    scui->ConnectXMLSignalWithSlot(stui);
    stui->ConnectXMLSignalWithSlot(scui);

    //connect(pUri, SIGNAL(EmergencyStopSignal()), this, SLOT(EmergencyStopSlot()));
    // 急停 should be solve

    pdTimer = new QTimer(this);
    connect( pdTimer, SIGNAL( timeout() ), this, SLOT( PedalTimerDone() ) );
    pdTimer->start(RobotParams::UITimerMs);

    pdRobot = new PedalRobot(ui->pQCustomPlot, conf, ui->plot_nvh_static, ui->plot_nvh_dynamics);

    InitWidgets();
    UpdateCarTypeWidget();

    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
}

PedalRobotUI::~PedalRobotUI()
{
    pdTimer->stop();

    delete pdRobot;
    delete pdTimer;
    delete conf;
//    delete scui;

    delete ui;
}

void PedalRobotUI::LockMutex()
{
    if(m_enableMutex){
        m_timerMutex.lock();
    }
}

void PedalRobotUI::UnlockMutex()
{
    if(m_enableMutex){
        m_timerMutex.unlock();
    }
}

void PedalRobotUI::PedalTimerDone()
{
    // 内部定时器上锁
//    LockMutex();

    // 软件倍频
    static int cnt = 0;
    const int rem = ++cnt%RobotParams::UITimerMultiplier;// ++cnt%3 rem=0/1/2...

    // 从服务器拿数据
    if(rem != 0){
        switch(rem){
        case 1: // rem=1
            AutoDriveRobotApiClient::GetInstance()->Send_GetPedalRobotDeviceDataMsg(); // CAN/MP412信息
            break;
        case 2: // rem=2
            AutoDriveRobotApiClient::GetInstance()->Send_GetRobotThetaMsg(); // 角度信息
            break;
        default:
            break;
        }
    }else{
        // 控制准备+时间差校验
        pdRobot->UpdatePart1();

        // rem=0 即18ms*3的控制周期 进行机器人的控制
        pdRobot->UpdatePart2();

        // 降低刷新次数 刷新界面
        static int cntUI = 0;
        if(++cntUI%RobotParams::updateUIFrequency == 0){

            // 定时器开始后 如果没问过是否回原 就问一下
            if (!RobotParams::askGoHomeatstart)
            {
                AutoDriveRobotApiClient::GetInstance()->Send_GetGoHomeResultMsg(); // 回原信息
                RobotParams::askGoHomeatstart = true;
            }

            // 发送了回原指令后
            if (ifSendGoHome)
            {
                if (GoHomeRound < 1)
                {
                    GoHomeRound = RobotParams::waitForGoHomeRound;
                    PRINTF(LOG_WARNING, "%s: go home overtime.\n", __func__);
                    ifSendGoHome = false;
                }

                if (!RobotParams::ifGoHome)
                {
                    GoHomeRound--;
                    AutoDriveRobotApiClient::GetInstance()->Send_GetGoHomeResultMsg(); // 回原信息
                }
                else
                {
                    if (RobotParams::readyToOrigin)
                    {
                        RobotParams::readyToOrigin = false;
                        ifSendGoHome = false;

                        RobotParams::askGoHomeatstartresult = 1;
                        RefreshOriginFile(); // 按照车型配置文件更新origin.txt
                        std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::originFilePath);
                        if(fileContent.empty()){
                            PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
                            return;
                        }
                        AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
                    }
                    else
                    {
                        GoHomeRound = RobotParams::waitForGoHomeRound;
                        PRINTF(LOG_INFO, "%s: go home success.\n", __func__);
                        RobotParams::readyToOrigin = true;
                    }
                }
            }

            AutoDriveRobotApiClient::GetInstance()->Send_GetStatusStringMsg(true, true); // 状态信息

            UpdateWidgets();
        }
    }

//    UnlockMutex();
}

void PedalRobotUI::SingleAxisPressed()
{
    int axis=0;
    double direction=0;
    if(sender() == ui->pushButton_plus1 || sender() == ui->pushButton_nvh_brkp){
        axis=0; direction=1;
    }else if(sender() == ui->pushButton_plus2 || sender() == ui->pushButton_nvh_accp){
        axis=1; direction=1;
    }else if(sender() == ui->pushButton_minus1 || sender() == ui->pushButton_nvh_brkn){
        axis=0; direction=-1;
    }else if(sender() == ui->pushButton_minus2 || sender() == ui->pushButton_nvh_accn){
        axis=1; direction=-1;
    }

    SetSingleAxisMove(axis, direction);
}

void PedalRobotUI::SingleAxisReleased()
{
    std::vector<int> stopAxes;
    for (unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        stopAxes.push_back(i);
    }

    AutoDriveRobotApiClient::GetInstance()->Send_StopSingleAxisMsg(stopAxes);
}

void PedalRobotUI::on_pushButton_origin_clicked()
{
    pdRobot->SoftStop();

    if (RobotParams::askGoHomeatstartresult == 1)
    {

        RefreshOriginFile(); // 按照车型配置文件更新origin.txt
        std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::originFilePath);
        if(fileContent.empty()){
            PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
            return;
        }
        AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
    }
    else if (RobotParams::askGoHomeatstartresult == 100)
    {
        AutoDriveRobotApiClient::GetInstance()->Send_GoHomeMsg(false);
        ifSendGoHome = true;
    }
}

void PedalRobotUI::on_pushButton_softStop_clicked()
{
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
    pdRobot->SoftStop();
    pdRobot->FinishQCustomPlot(false);
}

void PedalRobotUI::on_pushButton_softStop_liftPedals_clicked()
{
    // 停止
    on_pushButton_softStop_clicked();
    // 上抬踏板
    RefreshSoftStopFile(); // 按照车型配置文件更新softStop.txt
    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::softStopFilePath);
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
}

void PedalRobotUI::on_pushButton_startAction_clicked()
{
//    if (!pdRobot->CheckIfAtReady())
//    {
//        QMessageBox::information(NULL,"提示", QString("请正常测试后尝试运行曲线！"));
//        PRINTF(LOG_DEBUG, "%s: not at N shift or free clutch.\n", __func__);
//        return;
//    }

//    if (pdRobot->modelSelect != 0)
//    {
//        QMessageBox::information(NULL,"提示", QString("ACD界面下模式设置冲突，请更改后重试！"));
//        PRINTF(LOG_DEBUG, "%s: ACD module confliction.\n", __func__);
//        return;
//    }

//    if ( !pdRobot->isStateIdle() )
//    {
//        QMessageBox::information(NULL, "警告", QString("检测到有其他模式正在运行！\n请检查是否未退出测试或者其他运行模式！"));
//        PRINTF(LOG_WARNING, "%s: motion module confliction, state is\n\tswitchflag(%d,%d,%d,%d,%d,%d,%d,%d,%d,%d)\n\tisExaming(%d)\n\tisControlling(%d).\n", __func__, RobotParams::switchflag[0], RobotParams::switchflag[1], RobotParams::switchflag[2], RobotParams::switchflag[3], RobotParams::switchflag[4], RobotParams::switchflag[5], RobotParams::switchflag[6], RobotParams::switchflag[7], RobotParams::switchflag[8], RobotParams::switchflag[9], RobotParams::isExaming, pdRobot->GetIsControlling());
//        return;
//    }

    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

    if ( !pdRobot->SelectSpeedCurve( !ui->checkBox_repeatCurveFile->isChecked() ) )
    {
        return;
    }

//    if ( !pdRobot->isStateIdle() )
//    {
//        QMessageBox::information(NULL, "警告", QString("检测到有其他模式正在运行！\n请检查是否未退出测试或者其他运行模式！"));
//        PRINTF(LOG_WARNING, "%s: motion module confliction, state is\n\tswitchflag(%d,%d,%d,%d,%d,%d,%d,%d,%d,%d)\n\tisExaming(%d)\n\tisControlling(%d).\n", __func__, RobotParams::switchflag[0], RobotParams::switchflag[1], RobotParams::switchflag[2], RobotParams::switchflag[3], RobotParams::switchflag[4], RobotParams::switchflag[5], RobotParams::switchflag[6], RobotParams::switchflag[7], RobotParams::switchflag[8], RobotParams::switchflag[9], RobotParams::isExaming, pdRobot->GetIsControlling());
//        return;
//    }

    std::string fileContent = FileAssistantFunc::ReadFileContent(conf->defaultFile);
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

    PRINTF(LOG_DEBUG, "%s: starts.\n", __func__);
    pdRobot->StartQCustomPlot(conf->defaultFile);

    // 显示NEDC/WLTC的曲线类型
    const std::string fileName = NormalFile::GetFileName(conf->defaultFile.c_str()); // XXX_ARM
    const std::string curveType = fileName.substr(0,fileName.length()-4); // XXX
    ui->lineEdit_curveType->setText(curveType.c_str());
}

void PedalRobotUI::InitWidgets()
{
    QPalette palette;
    palette.setColor(QPalette::Text, Qt::white);
    ui->lineEdit_powerOnOff->setPalette(palette);

    // NVH界面设置
    ui->widget_nvh->setEnabled(false);
    ui->pushButton_nvh_run->setEnabled(true);
    ui->pushButton_nvh_stop->setEnabled(false);
    ui->lineEdit_nvh_p1time->setEnabled(false);
    ui->lineEdit_nvh_p1speed->setEnabled(false);
    ui->lineEdit_nvh_p2time->setEnabled(false);
    ui->lineEdit_nvh_p2speed->setEnabled(false);
    ui->pushButton_confirmaim->setEnabled(false);

    // ACD换挡界面设置
    ui->radioButton_cs2->setChecked(true);
    ui->comboBox_ashift->clear();
    ui->comboBox_ashift->insertItem(0, "N");
    ui->comboBox_ashift->insertItem(1, "1");
    ui->comboBox_ashift->insertItem(2, "2");
    ui->comboBox_ashift->insertItem(1, "3");
    ui->comboBox_ashift->insertItem(2, "4");
    ui->comboBox_ashift->insertItem(1, "5");
    ui->comboBox_ashift->insertItem(2, "6");

    EnableButtonsForGoHome(false);
}

void PedalRobotUI::UpdateWidgets()
{
    // 状态显示
    const double error = pdRobot->GetError();
    QPalette palette;
    if(fabs(error) > speedCurveSpeedErrorLimit){
        palette.setColor(QPalette::Text, Qt::red);
    }else{
        palette.setColor(QPalette::Text, Qt::black);
    }
    ui->lineEdit_sysControlMethod->setPalette(palette);
    ui->lineEdit_sysControlMethod->setText(
                QString::number(pdRobot->GetSysControlMethod()) +'/'+
                QString::number(error, 'g', 3));

    // PID显示
    double PID[3];

    ui->lineEdit_PIDParams->setText(
                QString::number(PID[0], 'g', 3)+"/"+
                QString::number(PID[1], 'g', 3)+"/"+
                QString::number(PID[2], 'g', 3));

    // 油门踏板位置
    ui->lineEdit_d1->setText( QString::number(RobotParams::angleRealTime[0], 'g', 4) );
    ui->lineEdit_d2->setText( QString::number(RobotParams::angleRealTime[1], 'g', 4) );
    ui->progressBar_brake->setValue( qRound(pdRobot->GetBrakePosition()) );
    ui->progressBar_accelerator->setValue( qRound(pdRobot->GetAcceleratorPosition()) );

    // 挡位离合位置
//    QString shiftnow = QString::fromStdString(RobotParams::currentshiftvalue);
//    if (shiftnow == QString("N_1&2") || shiftnow == QString("N_3&4") || shiftnow == QString("N_5&6"))
//    {
//        shiftnow = QString("N");
//    }
//    ui->lineEdit_shiftcurrent->setText( shiftnow );
//    ui->lineEdit_clutchcurrent->setText( QString::fromStdString(RobotParams::currentclutchvalue) );
    scui->UpdateAnglesForShiftClutch();
//    ui->lineEdit_cshift->setText( shiftnow );
//    ui->lineEdit_cc->setText( QString::fromStdString(RobotParams::currentclutchvalue) );

    // CAN数据
    ui->lineEdit_CanBrakeOpenValue->setText(QString::number(pdRobot->GetBrakePosition(), 'g', 4));
    ui->lineEdit_CanAccOpenValue->setText(QString::number(pdRobot->GetAcceleratorPosition(), 'g', 4));
    ui->lineEdit_CanSpeed->setText(QString::number(pdRobot->GetCanCarSpeed(), 'g', 4));
    const int mode = pdRobot->GetPowerMode();
    ui->lineEdit_CanPowerMode->setText(QString::number(mode));



    // 工作模式
    if(mode == PedalRobot::Run){
        ui->lineEdit_powerOnOff->setStyleSheet("background-color:green");
    }else if(mode == PedalRobot::Off){
        ui->lineEdit_powerOnOff->setStyleSheet("background-color:red");
    }else{
        ui->lineEdit_powerOnOff->setStyleSheet("background-color:blue");
    }

    // 回原状态
    if (RobotParams::ifGoHome)
    {
        ui->pushButton_origin->setStyleSheet("color:green");
    }
    else
    {
        ui->pushButton_origin->setStyleSheet("color:red");
    }

    // MP412数据
    ui->lineEdit_adVoltage->setText("");
    ui->lineEdit_pulseFrequency->setText("");
    ui->lineEdit_UsbMP412Speed->setText( QString::number( pdRobot->GetMP412CarSpeed() ) );

    // 车辆文件
    UpdateCarTypeWidget();

    // 速度反馈
    UpdateGetSpeedWidget();

    // 状态栏
    statusL->setText( QString::fromStdString(RobotParams::statusStr) );

    // 时间栏
    QDateTime time = QDateTime::currentDateTime();
    QString str = time.toString("yyyy-MM-dd hh:mm:ss");
    timeL->setText(str);


    // 确认回原等状态
    static bool isGoHomedLast = !RobotParams::ifGoHome; // 初始值 保证进入if判断

    if(RobotParams::ifGoHome != isGoHomedLast){ // 状态有变化
        isGoHomedLast = RobotParams::ifGoHome;
        EnableButtonsForGoHome(true);
    }


}

void PedalRobotUI::UpdateCarTypeWidget()
{
    std::string carTypewithxml = NormalFile::GetFileName( (conf->sysFilePath + conf->carTypeName).c_str() );
    const std::string carType = carTypewithxml.substr(0, carTypewithxml.length() - 4);
    static std::string carTypeLast = "";
    if(carTypeLast != carType){
        carTypeLast = carType;
        ui->lineEdit_carType->setText(carType.c_str());
    }
}

void PedalRobotUI::UpdateGetSpeedWidget()
{
    QString content="";
    switch (conf->getSpeedMethod){
    case 0:
        content = tr("脉冲的车速");
        break;
    case 1:
        content = tr("CAN卡的车速");
        break;
    default:
        content = tr("未知方法!");
        break;
    }
    ui->lineEdit_getSpeedMethod->setText(content);
}

void PedalRobotUI::EnableButtonsForGoHome(bool enable)
{
    //enable=false 尚未回原
    if (!enable)
    {
        ui->pushButton_softStop_liftPedals->setEnabled(false);
        ui->pushButton_slowlybrake->setEnabled(false);
        ui->pushButton_startAction->setEnabled(false);
    }
    else
    {
//        if ( RobotParams::ifGoHome && RobotParams::ifConfirmSC && ( !conf->ifManualShift || (conf->ifManualShift && RobotParams::ifConfirmCS) ) )
//        {
            ui->pushButton_softStop_liftPedals->setEnabled(enable);
            ui->pushButton_slowlybrake->setEnabled(enable);
            ui->pushButton_startAction->setEnabled(enable);
//        }
//        else
//        {
//            ui->pushButton_softStop_liftPedals->setEnabled(false);
//            ui->pushButton_slowlybrake->setEnabled(false);
//            ui->pushButton_startAction->setEnabled(false);
//        }
    }
}

bool PedalRobotUI::eventFilter(QObject *watched, QEvent *event)
{
    //程序快捷键
    Q_UNUSED(watched);

    if(FilterTabSwitchKey(event)){
        return true;
    }
    if(FilterSingleAxisKey(event)){
        return true;
    }

    return QWidget::eventFilter(watched, event);
}

bool PedalRobotUI::FilterTabSwitchKey(QEvent *event)
{
    if(event->type() == QEvent::KeyPress){
        QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
        if(!keyEvent->isAutoRepeat()){//不是重复按键
            //确定tabIndex
            int tabIndex = -1;
            switch(keyEvent->key()){//F1~F4
            case Qt::Key_F1: tabIndex = 0; break;
            case Qt::Key_F2: tabIndex = 1; break;
            case Qt::Key_F3: tabIndex = 2; break;
            case Qt::Key_F4: tabIndex = 3; break;
            default: break;
            }
            //切换tab
            if(tabIndex != -1){
                ui->tabWidget->setCurrentIndex(tabIndex);
                return true;
            }
        }
    }

    return false;
}

bool PedalRobotUI::FilterSingleAxisKey(QEvent *event)
{
    bool validKey = false;

    int keyType = 0;
    switch(event->type()){
    case QEvent::KeyPress:   keyType = 1; break;
    case QEvent::KeyRelease: keyType = -1; break;
    default: break;
    }

    if(keyType != 0){//1=press or -1=release
        QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
        if(!keyEvent->isAutoRepeat()){//不是重复按键
            if(keyEvent->modifiers()&Qt::AltModifier){//Alt+1~4
                switch(keyEvent->key()){
                case Qt::Key_1://刹车-
                    validKey = SingleAxisKeyAction(0, false/*+direction?*/, keyType==1? true: false/*press?*/);
                    break;
                case Qt::Key_2://刹车+
                    validKey = SingleAxisKeyAction(0, true/*+direction?*/, keyType==1? true: false/*press?*/);
                    break;
                case Qt::Key_3://油门-
                    validKey = SingleAxisKeyAction(1, false/*+direction?*/, keyType==1? true: false/*press?*/);
                    break;
                case Qt::Key_4://油门+
                    validKey = SingleAxisKeyAction(1, true/*+direction?*/, keyType==1? true: false/*press?*/);
                    break;
                }
            }
        }
    }

    return validKey;
}

bool PedalRobotUI::SingleAxisKeyAction(const int axisNum, const bool posDirection, const bool keyPress)
{
    //操作tab页面 该快捷键才有效
    if(ui->tabWidget->currentIndex() != 2){
        return false;
    }

    if(keyPress){//press
        SetSingleAxisMove(axisNum, posDirection? 1: -1);
    }else{//release
        SingleAxisReleased();
    }

    if(axisNum == 0){
        if(posDirection){
            SetSingleAxisButtonStyle(ui->pushButton_plus1, keyPress);
        }else{
            SetSingleAxisButtonStyle(ui->pushButton_minus1, keyPress);
        }
    }else if(axisNum == 1){
        if(posDirection){
            SetSingleAxisButtonStyle(ui->pushButton_plus2, keyPress);
        }else{
            SetSingleAxisButtonStyle(ui->pushButton_minus2, keyPress);
        }
    }

    return true;
}

void PedalRobotUI::SetSingleAxisMove(const int axisNum, const int direction)
{
    if(sender() == ui->pushButton_plus1 || sender() == ui->pushButton_plus2 || sender() == ui->pushButton_minus1 || sender() == ui->pushButton_minus2)
    {
       pdRobot->SoftStop();
    }

    const double speed = direction * RobotParams::singleAxisBtnRatio;

    std::vector<int> moveAxes;
    std::vector<double> moveSpeed;
    moveAxes.push_back(axisNum);
    moveSpeed.push_back(speed);

    AutoDriveRobotApiClient::GetInstance()->Send_MoveSingleAxisMsg(moveAxes, moveSpeed);
}

void PedalRobotUI::SetSingleAxisButtonStyle(QPushButton *btn, const bool keyPress)
{
    static QString initStyle = ui->pushButton_minus1->styleSheet();
    const QString pressStyle("background-color: rgb(85, 170, 255);");
    btn->setStyleSheet(keyPress? pressStyle: initStyle);
}

void PedalRobotUI::RefreshSoftStopFile()
{
    std::fstream ssf(Configuration::softStopFilePath.c_str(), std::fstream::out | std::fstream::binary);
    if(ssf.fail()){
        PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, Configuration::softStopFilePath.c_str());
        return;
    }
    ssf << RobotParams::robotType << "\n";
    ssf << 'R' << "\n";
    ssf << std::right << std::setw(15) << 0;
    ssf << std::right << std::setw(15) << 0;
    ssf << std::right << std::setw(15) << 2000;
    ssf << std::right << std::setw(15) << 0;
    ssf << std::right << std::setw(15) << 0 << "\n";
    ssf << 'T' << "\n";
    ssf << Configuration::GetInstance()->translateSpeed << "\n";
    ssf << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[0];
    ssf << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];

    if (Configuration::GetInstance()->ifManualShift)
    {
        ssf << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[0];
        ssf << std::right << std::setw(15) << RobotParams::angleRealTime[3];
        ssf << std::right << std::setw(15) << RobotParams::angleRealTime[4];
        ssf << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }
    else
    {
        ssf << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
        ssf << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[1];
        ssf << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[1];
        ssf << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }

    ssf.close();
}

void PedalRobotUI::RefreshOriginFile()
{
    std::fstream ogf(Configuration::originFilePath.c_str(), std::fstream::out | std::fstream::binary);
    if(ogf.fail()){
        PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, Configuration::originFilePath.c_str());
        return;
    }
    ogf << RobotParams::robotType << "\n";
    ogf << 'R' << "\n";
    ogf << std::right << std::setw(15) << 0;
    ogf << std::right << std::setw(15) << 0;
    ogf << std::right << std::setw(15) << 2000;
    ogf << std::right << std::setw(15) << 0;
    ogf << std::right << std::setw(15) << 0 << "\n";
    ogf << 'T' << "\n";
    ogf << Configuration::GetInstance()->translateSpeed << "\n";
    ogf << std::right << std::setw(15) << Configuration::GetInstance()->brakeThetaAfterGoHome;
    ogf << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
    ogf << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
    ogf << std::right << std::setw(15) << RobotParams::angleRealTime[3];
    ogf << std::right << std::setw(15) << RobotParams::angleRealTime[4];
    ogf << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    ogf.close();
}

void PedalRobotUI::on_pushButton_saveLoggerFile_clicked()
{
    QString filePath = QFileDialog::getSaveFileName(NULL, QObject::tr("日志文件保存地址:"), Configuration::logCurvePath.c_str());
    if(filePath == ""){
        return;
    }

    pdRobot->SaveLoggerFile(filePath.toStdString().c_str());
    QMessageBox::information(NULL, "提示", QObject::tr("日志文件已保存到:\n")+filePath);
}

//void PedalRobotUI::on_pushButton_slowlybrake_clicked()
//{
//    // 停止
//    on_pushButton_softStop_clicked();

//    // 上抬踏板 缓踩刹车
//    std::fstream sb(Configuration::slowlyBrakeFilePath.c_str(), std::fstream::out | std::fstream::binary);
//    if(sb.fail()){
//        PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, Configuration::slowlyBrakeFilePath.c_str());
//        return;
//    }
//    sb << RobotParams::robotType << "\n";
//    sb << 'R' << "\n";
//    sb << std::right << std::setw(15) << 0;
//    sb << std::right << std::setw(15) << 0;
//    sb << std::right << std::setw(15) << 2000;
//    sb << std::right << std::setw(15) << 0;
//    sb << std::right << std::setw(15) << 0 << "\n";
//    sb << 'T' << "\n";
//    sb << Configuration::GetInstance()->translateSpeed << "\n";
//    sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[0];
//    sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];

//    if (Configuration::GetInstance()->ifManualShift)
//    {
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[0];
//        sb << std::right << std::setw(15) << RobotParams::angleRealTime[3];
//        sb << std::right << std::setw(15) << RobotParams::angleRealTime[4];
//        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

//        // 踩刹车
//        sb << 'T' << "\n";
//        sb << Configuration::GetInstance()->translateSpeed/4 << "\n";
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->brakeThetaAfterGoHome;
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[0];
//        sb << std::right << std::setw(15) << RobotParams::angleRealTime[3];
//        sb << std::right << std::setw(15) << RobotParams::angleRealTime[4];
//        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
//    }
//    else
//    {
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[1];
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[1];
//        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

//        // 踩刹车
//        sb << 'T' << "\n";
//        sb << Configuration::GetInstance()->translateSpeed/4 << "\n";
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->brakeThetaAfterGoHome;
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[1];
//        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[1];
//        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
//    }

//    sb.close();

//    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::slowlyBrakeFilePath);
//    if(fileContent.empty()){
//        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
//        return;
//    }
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
//}

//void PedalRobotUI::on_pushButton_nvh_start_clicked()
//{
//    if (conf->ifManualShift)
//    {
//        QMessageBox::information(NULL, "提示", QString("手动档车辆不能进行NVH实验！"));
//        PRINTF(LOG_DEBUG, "%s: no NVH for manul shift cars.\n", __func__);
//        return;
//    }

//    if (conf->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::NedcControl)
//    {
//        QMessageBox::information(NULL, "提示", QString("请设置位WLTC控制方式！"));
//        PRINTF(LOG_DEBUG, "%s: better WLTC for NVH.\n", __func__);
//        return;
//    }

//    if (!RobotParams::ifGoHome || !RobotParams::ifConfirmSC)
//    {
//        QMessageBox::information(NULL, "提示", QString("请先回原并且确认挡位信息！"));
//        PRINTF(LOG_DEBUG, "%s: no back to origin or confirm shift infomation for NVH.\n", __func__);
//        return;
//    }

//    if (!pdRobot->CheckIfAtReady())
//    {
//        QMessageBox::information(NULL,"提示", QString("请保证车辆在空档再运行NVH实验！"));
//        PRINTF(LOG_DEBUG, "%s: not at N shift for NVH.\n", __func__);
//        return;
//    }

//    if ( !pdRobot->isStateIdle() )
//    {
//        QMessageBox::information(NULL, "警告", QString("检测到有其他模式正在运行！\n请检查是否未退出测试或者其他运行模式！"));
//        PRINTF(LOG_WARNING, "%s: motion module confliction, state is\n\tswitchflag(%d,%d,%d,%d,%d,%d,%d,%d,%d,%d)\n\tisExaming(%d)\n\tisControlling(%d).\n", __func__, RobotParams::switchflag[0], RobotParams::switchflag[1], RobotParams::switchflag[2], RobotParams::switchflag[3], RobotParams::switchflag[4], RobotParams::switchflag[5], RobotParams::switchflag[6], RobotParams::switchflag[7], RobotParams::switchflag[8], RobotParams::switchflag[9], RobotParams::isExaming, pdRobot->GetIsControlling());
//        return;
//    }

//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

//    if ( !pdRobot->ReadyToNVH() )
//    {
//        return;
//    }

//    if ( !pdRobot->isStateIdle() )
//    {
//        if (!RobotParams::switchflag[3])
//        {
//        QMessageBox::information(NULL, "警告", QString("检测到有其他模式正在运行！\n请检查是否未退出测试或者其他运行模式！"));
//        PRINTF(LOG_WARNING, "%s: motion module confliction, state is\n\tswitchflag(%d,%d,%d,%d,%d,%d,%d,%d,%d,%d)\n\tisExaming(%d)\n\tisControlling(%d).\n", __func__, RobotParams::switchflag[0], RobotParams::switchflag[1], RobotParams::switchflag[2], RobotParams::switchflag[3], RobotParams::switchflag[4], RobotParams::switchflag[5], RobotParams::switchflag[6], RobotParams::switchflag[7], RobotParams::switchflag[8], RobotParams::switchflag[9], RobotParams::isExaming, pdRobot->GetIsControlling());
//        return;
//        }
//    }

//    RobotParams::switchflag[2] = true;
//    RobotParams::NVHcurvestate = 9;
//    ui->widget_nvh->setEnabled(true);
//    ui->pushButton_nvh_start->setEnabled(false);
//    ui->pushButton_nvh_stop->setEnabled(true);
//    ui->lineEdit_nvh_p1time->setEnabled(false);
//    ui->lineEdit_nvh_p1speed->setEnabled(false);
//    ui->lineEdit_nvh_p2time->setEnabled(false);
//    ui->lineEdit_nvh_p2speed->setEnabled(false);
//    ui->pushButton_confirmaim->setEnabled(false);

//    pdRobot->StartQCustomPlotSpeed();

//    // 更新NVH文件
//    RefreshNVHFile(0);
//    RefreshNVHFile(1);
//    RefreshNVHFile(2);
//    RefreshNVHFile(3);
//}

//void PedalRobotUI::on_pushButton_nvh_stop_clicked()
//{
//    // 进入退出状态
//    pdRobot->ifFirstToExitNVH = true;
//    RobotParams::NVHcurvestate = 4;

//    ui->widget_nvh->setEnabled(false);
//    ui->pushButton_nvh_start->setEnabled(true);
//    ui->pushButton_nvh_stop->setEnabled(false);
//}

//void PedalRobotUI::on_pushButton_nvh_log_clicked()
//{
//    QString filePath = QFileDialog::getSaveFileName(NULL, QObject::tr("日志文件保存地址:"), Configuration::logCurvePath.c_str());
//    if(filePath == ""){
//        return;
//    }

//    pdRobot->SaveLoggerFile(filePath.toStdString().c_str());
//    QMessageBox::information(NULL, "提示", QObject::tr("日志文件已保存到:\n")+filePath);
//}

//void PedalRobotUI::on_comboBox_nvh_mode_currentIndexChanged(int index)
//{
//    switch (index)
//    {
//    case 0:
//        ui->lineEdit_nvh_p1time->setText("/");
//        ui->lineEdit_nvh_p1speed->setText("/");
//        ui->lineEdit_nvh_p2time->setText("/");
//        ui->lineEdit_nvh_p2speed->setText("/");
//        ui->lineEdit_nvh_p1time->setEnabled(false);
//        ui->lineEdit_nvh_p1speed->setEnabled(false);
//        ui->lineEdit_nvh_p2time->setEnabled(false);
//        ui->lineEdit_nvh_p2speed->setEnabled(false);
//        ui->pushButton_confirmaim->setEnabled(false);
//        break;
//    case 1:
//        RobotParams::nvh_P2t = 34.0;
//        RobotParams::nvh_P2v = 140.0;
//        ui->lineEdit_nvh_p1time->setText("0.0");
//        ui->lineEdit_nvh_p1speed->setText("8.0");
//        ui->lineEdit_nvh_p2time->setText("34.0");
//        ui->lineEdit_nvh_p2speed->setText("140.0");
//        ui->lineEdit_nvh_p1time->setEnabled(false);
//        ui->lineEdit_nvh_p1speed->setEnabled(false);
//        ui->lineEdit_nvh_p2time->setEnabled(true);
//        ui->lineEdit_nvh_p2speed->setEnabled(true);
//        ui->pushButton_confirmaim->setEnabled(true);
//        break;
//    case 2:
//        ui->lineEdit_nvh_p1time->setText("/");
//        ui->lineEdit_nvh_p1speed->setText("/");
//        ui->lineEdit_nvh_p2time->setText("/");
//        ui->lineEdit_nvh_p2speed->setText("/");
//        ui->lineEdit_nvh_p1time->setEnabled(false);
//        ui->lineEdit_nvh_p1speed->setEnabled(false);
//        ui->lineEdit_nvh_p2time->setEnabled(false);
//        ui->lineEdit_nvh_p2speed->setEnabled(false);
//        ui->pushButton_confirmaim->setEnabled(false);
//        break;
//    case 3:
//        RobotParams::nvh_P1t = 0.0;
//        RobotParams::nvh_P1v = 70.0;
//        RobotParams::nvh_P2t = 100.0;
//        RobotParams::nvh_P2v = 140.0;
//        ui->lineEdit_nvh_p1time->setText("0.0");
//        ui->lineEdit_nvh_p1speed->setText("70.0");
//        ui->lineEdit_nvh_p2time->setText("100.0");
//        ui->lineEdit_nvh_p2speed->setText("140.0");
//        ui->lineEdit_nvh_p1time->setEnabled(false);
//        ui->lineEdit_nvh_p1speed->setEnabled(true);
//        ui->lineEdit_nvh_p2time->setEnabled(true);
//        ui->lineEdit_nvh_p2speed->setEnabled(true);
//        ui->pushButton_confirmaim->setEnabled(true);
//        break;
//    default:
//        break;
//    }

//    RefreshNVHFile(index);
//}

//void PedalRobotUI::on_pushButton_confirmaim_clicked()
//{
//    unsigned int index = ui->comboBox_nvh_mode->currentIndex();

//    switch (index)
//    {
//    case 1:
//        RobotParams::nvh_P2t = ui->lineEdit_nvh_p2time->text().toDouble();
//        RobotParams::nvh_P2v = ui->lineEdit_nvh_p2speed->text().toDouble();
//        break;
//    case 3:
//        RobotParams::nvh_P1t = ui->lineEdit_nvh_p1time->text().toDouble();
//        RobotParams::nvh_P1v = ui->lineEdit_nvh_p1speed->text().toDouble();
//        RobotParams::nvh_P2t = ui->lineEdit_nvh_p2time->text().toDouble();
//        RobotParams::nvh_P2v = ui->lineEdit_nvh_p2speed->text().toDouble();
//        break;
//    default:
//        break;
//    }

//    RefreshNVHFile(index);
//}

//void PedalRobotUI::on_pushButton_nvh_run_clicked()
//{
//    if (RobotParams::switchflag[3])
//    {
//        QMessageBox::information(NULL,"提示", QString("踏板移动，请稍后再试！"));
//        return;
//    }

//    unsigned int index = ui->comboBox_nvh_mode->currentIndex();

//    if ( !pdRobot->SelectSpeedCurveNVH( index ) )
//    {
//        return;
//    }

//    std::string fileContent = FileAssistantFunc::ReadFileContent(conf->defaultFile);
//    if(fileContent.empty()){
//        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
//        return;
//    }
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

//    PRINTF(LOG_DEBUG, "%s: starts.\n", __func__);
//    pdRobot->StartQCustomPlotNVH(conf->defaultFile, index);

//    // 显示NVH曲线类型
//    const std::string fileName = NormalFile::GetFileName(conf->defaultFile.c_str()); // XXX_ARM
//    const std::string curveType = fileName.substr(0,fileName.length()-4); // XXX
//    ui->lineEdit_curveType->setText(curveType.c_str());
//}

//void PedalRobotUI::on_pushButton_nvh_softstop_clicked()
//{
//    // 停止
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
//    pdRobot->FinishQCustomPlotNVH(false);

//    // 上抬踏板
//    std::fstream nssf(Configuration::nvhsoftStopFilePath.c_str(), std::fstream::out | std::fstream::binary);
//    if(nssf.fail()){
//        PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, Configuration::nvhsoftStopFilePath.c_str());
//        return;
//    }
//    nssf << RobotParams::robotType << "\n";
//    nssf << 'R' << "\n";
//    nssf << std::right << std::setw(15) << 0;
//    nssf << std::right << std::setw(15) << 0;
//    nssf << std::right << std::setw(15) << 2000;
//    nssf << std::right << std::setw(15) << 0;
//    nssf << std::right << std::setw(15) << 0 << "\n";
//    nssf << 'T' << "\n";
//    nssf << Configuration::GetInstance()->translateSpeed << "\n";
//    nssf << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[0];
//    nssf << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
//    nssf << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
//    nssf << std::right << std::setw(15) << RobotParams::angleRealTime[3];
//    nssf << std::right << std::setw(15) << RobotParams::angleRealTime[4];
//    nssf << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

//    nssf.close();

//    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::nvhsoftStopFilePath);
//    if(fileContent.empty()){
//        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
//        return;
//    }
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
//}

//void PedalRobotUI::on_pushButton_nvh_slowbrake_clicked()
//{
//    // 停止
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
//    pdRobot->FinishQCustomPlotNVH(false);

//    // 上抬踏板 缓踩刹车
//    std::fstream nsb(Configuration::nvhslowlyBrakeFilePath.c_str(), std::fstream::out | std::fstream::binary);
//    if(nsb.fail()){
//        PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, Configuration::nvhslowlyBrakeFilePath.c_str());
//        return;
//    }
//    nsb << RobotParams::robotType << "\n";
//    nsb << 'R' << "\n";
//    nsb << std::right << std::setw(15) << 0;
//    nsb << std::right << std::setw(15) << 0;
//    nsb << std::right << std::setw(15) << 2000;
//    nsb << std::right << std::setw(15) << 0;
//    nsb << std::right << std::setw(15) << 0 << "\n";
//    nsb << 'T' << "\n";
//    nsb << Configuration::GetInstance()->translateSpeed << "\n";
//    nsb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[0];
//    nsb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
//    nsb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
//    nsb << std::right << std::setw(15) << RobotParams::angleRealTime[3];
//    nsb << std::right << std::setw(15) << RobotParams::angleRealTime[4];
//    nsb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

//    // 踩刹车
//    nsb << 'T' << "\n";
//    nsb << Configuration::GetInstance()->translateSpeed/4 << "\n";
//    nsb << std::right << std::setw(15) << Configuration::GetInstance()->brakeThetaAfterGoHome;
//    nsb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
//    nsb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
//    nsb << std::right << std::setw(15) << RobotParams::angleRealTime[3];
//    nsb << std::right << std::setw(15) << RobotParams::angleRealTime[4];
//    nsb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

//    nsb.close();

//    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::slowlyBrakeFilePath);
//    if(fileContent.empty()){
//        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
//        return;
//    }
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
//}

//bool PedalRobotUI::RefreshNVHFile(const int index)
//{
//    switch (index)
//    {
//    case 0:
//    {
//        std::fstream nvh0((Configuration::examFilePath + "NVH0").c_str(), std::fstream::out | std::fstream::binary);
//        if(nvh0.fail()){
//            PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, (Configuration::examFilePath + "NVH0").c_str());
//            return false;
//        }

//        nvh0 << RobotParams::robotType << "\n";
//        nvh0 << 'R' << "\n";
//        nvh0 << std::right << std::setw(15) << 0;
//        nvh0 << std::right << std::setw(15) << 0;
//        nvh0 << std::right << std::setw(15) << 300;
//        nvh0 << std::right << std::setw(15) << 0;
//        nvh0 << std::right << std::setw(15) << 0 << "\n";
//        nvh0 << 'M' << "\n";
//        nvh0 << std::right << std::setw(15) << 201 << "\n";
//        nvh0 << std::right << std::setw(15) << 0.00 << std::right << std::setw(15) << 8.00 << "\n";
//        nvh0 << std::right << std::setw(15) << 0.25 << std::right << std::setw(15) << 9.85 << "\n";
//        nvh0 << std::right << std::setw(15) << 0.50 << std::right << std::setw(15) << 11.70 << "\n";
//        nvh0 << std::right << std::setw(15) << 0.75 << std::right << std::setw(15) << 13.56 << "\n";
//        nvh0 << std::right << std::setw(15) << 1.00 << std::right << std::setw(15) << 15.41 << "\n";
//        nvh0 << std::right << std::setw(15) << 1.25 << std::right << std::setw(15) << 17.16 << "\n";
//        nvh0 << std::right << std::setw(15) << 1.50 << std::right << std::setw(15) << 18.92 << "\n";
//        nvh0 << std::right << std::setw(15) << 1.75 << std::right << std::setw(15) << 20.67 << "\n";
//        nvh0 << std::right << std::setw(15) << 2.00 << std::right << std::setw(15) << 22.42 << "\n";
//        nvh0 << std::right << std::setw(15) << 2.25 << std::right << std::setw(15) << 24.07 << "\n";
//        nvh0 << std::right << std::setw(15) << 2.50 << std::right << std::setw(15) << 25.71 << "\n";
//        nvh0 << std::right << std::setw(15) << 2.75 << std::right << std::setw(15) << 27.35 << "\n";
//        nvh0 << std::right << std::setw(15) << 3.00 << std::right << std::setw(15) << 29.00 << "\n";
//        nvh0 << std::right << std::setw(15) << 3.25 << std::right << std::setw(15) << 30.51 << "\n";
//        nvh0 << std::right << std::setw(15) << 3.50 << std::right << std::setw(15) << 32.03 << "\n";
//        nvh0 << std::right << std::setw(15) << 3.75 << std::right << std::setw(15) << 33.55 << "\n";
//        nvh0 << std::right << std::setw(15) << 4.00 << std::right << std::setw(15) << 35.07 << "\n";
//        nvh0 << std::right << std::setw(15) << 4.25 << std::right << std::setw(15) << 36.45 << "\n";
//        nvh0 << std::right << std::setw(15) << 4.50 << std::right << std::setw(15) << 37.83 << "\n";
//        nvh0 << std::right << std::setw(15) << 4.75 << std::right << std::setw(15) << 39.22 << "\n";
//        nvh0 << std::right << std::setw(15) << 5.00 << std::right << std::setw(15) << 40.60 << "\n";
//        nvh0 << std::right << std::setw(15) << 5.25 << std::right << std::setw(15) << 41.86 << "\n";
//        nvh0 << std::right << std::setw(15) << 5.50 << std::right << std::setw(15) << 43.11 << "\n";
//        nvh0 << std::right << std::setw(15) << 5.75 << std::right << std::setw(15) << 44.37 << "\n";
//        nvh0 << std::right << std::setw(15) << 6.00 << std::right << std::setw(15) << 45.63 << "\n";
//        nvh0 << std::right << std::setw(15) << 6.25 << std::right << std::setw(15) << 46.79 << "\n";
//        nvh0 << std::right << std::setw(15) << 6.50 << std::right << std::setw(15) << 47.95 << "\n";
//        nvh0 << std::right << std::setw(15) << 6.75 << std::right << std::setw(15) << 49.11 << "\n";
//        nvh0 << std::right << std::setw(15) << 7.00 << std::right << std::setw(15) << 50.27 << "\n";
//        nvh0 << std::right << std::setw(15) << 7.25 << std::right << std::setw(15) << 51.36 << "\n";
//        nvh0 << std::right << std::setw(15) << 7.50 << std::right << std::setw(15) << 52.44 << "\n";
//        nvh0 << std::right << std::setw(15) << 7.75 << std::right << std::setw(15) << 53.52 << "\n";
//        nvh0 << std::right << std::setw(15) << 8.00 << std::right << std::setw(15) << 54.60 << "\n";
//        nvh0 << std::right << std::setw(15) << 8.25 << std::right << std::setw(15) << 55.61 << "\n";
//        nvh0 << std::right << std::setw(15) << 8.50 << std::right << std::setw(15) << 56.62 << "\n";
//        nvh0 << std::right << std::setw(15) << 8.75 << std::right << std::setw(15) << 57.64 << "\n";
//        nvh0 << std::right << std::setw(15) << 9.00 << std::right << std::setw(15) << 58.65 << "\n";
//        nvh0 << std::right << std::setw(15) << 9.25 << std::right << std::setw(15) << 59.61 << "\n";
//        nvh0 << std::right << std::setw(15) << 9.50 << std::right << std::setw(15) << 60.58 << "\n";
//        nvh0 << std::right << std::setw(15) << 9.75 << std::right << std::setw(15) << 61.54 << "\n";
//        nvh0 << std::right << std::setw(15) << 10.00 << std::right << std::setw(15) << 62.50 << "\n";
//        nvh0 << std::right << std::setw(15) << 10.25 << std::right << std::setw(15) << 63.41 << "\n";
//        nvh0 << std::right << std::setw(15) << 10.50 << std::right << std::setw(15) << 64.33 << "\n";
//        nvh0 << std::right << std::setw(15) << 10.75 << std::right << std::setw(15) << 65.24 << "\n";
//        nvh0 << std::right << std::setw(15) << 11.00 << std::right << std::setw(15) << 66.15 << "\n";
//        nvh0 << std::right << std::setw(15) << 11.25 << std::right << std::setw(15) << 67.02 << "\n";
//        nvh0 << std::right << std::setw(15) << 11.50 << std::right << std::setw(15) << 67.88 << "\n";
//        nvh0 << std::right << std::setw(15) << 11.75 << std::right << std::setw(15) << 68.74 << "\n";
//        nvh0 << std::right << std::setw(15) << 12.00 << std::right << std::setw(15) << 69.60 << "\n";
//        nvh0 << std::right << std::setw(15) << 12.25 << std::right << std::setw(15) << 70.42 << "\n";
//        nvh0 << std::right << std::setw(15) << 12.50 << std::right << std::setw(15) << 71.24 << "\n";
//        nvh0 << std::right << std::setw(15) << 12.75 << std::right << std::setw(15) << 72.06 << "\n";
//        nvh0 << std::right << std::setw(15) << 13.00 << std::right << std::setw(15) << 72.88 << "\n";
//        nvh0 << std::right << std::setw(15) << 13.25 << std::right << std::setw(15) << 73.66 << "\n";
//        nvh0 << std::right << std::setw(15) << 13.50 << std::right << std::setw(15) << 74.44 << "\n";
//        nvh0 << std::right << std::setw(15) << 13.75 << std::right << std::setw(15) << 75.22 << "\n";
//        nvh0 << std::right << std::setw(15) << 14.00 << std::right << std::setw(15) << 76.00 << "\n";
//        nvh0 << std::right << std::setw(15) << 14.25 << std::right << std::setw(15) << 76.75 << "\n";
//        nvh0 << std::right << std::setw(15) << 14.50 << std::right << std::setw(15) << 77.50 << "\n";
//        nvh0 << std::right << std::setw(15) << 14.75 << std::right << std::setw(15) << 78.25 << "\n";
//        nvh0 << std::right << std::setw(15) << 15.00 << std::right << std::setw(15) << 79.00 << "\n";
//        nvh0 << std::right << std::setw(15) << 15.25 << std::right << std::setw(15) << 79.72 << "\n";
//        nvh0 << std::right << std::setw(15) << 15.50 << std::right << std::setw(15) << 80.44 << "\n";
//        nvh0 << std::right << std::setw(15) << 15.75 << std::right << std::setw(15) << 81.16 << "\n";
//        nvh0 << std::right << std::setw(15) << 16.00 << std::right << std::setw(15) << 81.88 << "\n";
//        nvh0 << std::right << std::setw(15) << 16.25 << std::right << std::setw(15) << 82.57 << "\n";
//        nvh0 << std::right << std::setw(15) << 16.50 << std::right << std::setw(15) << 83.26 << "\n";
//        nvh0 << std::right << std::setw(15) << 16.75 << std::right << std::setw(15) << 83.95 << "\n";
//        nvh0 << std::right << std::setw(15) << 17.00 << std::right << std::setw(15) << 84.64 << "\n";
//        nvh0 << std::right << std::setw(15) << 17.25 << std::right << std::setw(15) << 85.30 << "\n";
//        nvh0 << std::right << std::setw(15) << 17.50 << std::right << std::setw(15) << 85.97 << "\n";
//        nvh0 << std::right << std::setw(15) << 17.75 << std::right << std::setw(15) << 86.63 << "\n";
//        nvh0 << std::right << std::setw(15) << 18.00 << std::right << std::setw(15) << 87.29 << "\n";
//        nvh0 << std::right << std::setw(15) << 18.25 << std::right << std::setw(15) << 87.93 << "\n";
//        nvh0 << std::right << std::setw(15) << 18.50 << std::right << std::setw(15) << 88.57 << "\n";
//        nvh0 << std::right << std::setw(15) << 18.75 << std::right << std::setw(15) << 89.20 << "\n";
//        nvh0 << std::right << std::setw(15) << 19.00 << std::right << std::setw(15) << 89.84 << "\n";
//        nvh0 << std::right << std::setw(15) << 19.25 << std::right << std::setw(15) << 90.45 << "\n";
//        nvh0 << std::right << std::setw(15) << 19.50 << std::right << std::setw(15) << 91.07 << "\n";
//        nvh0 << std::right << std::setw(15) << 19.75 << std::right << std::setw(15) << 91.69 << "\n";
//        nvh0 << std::right << std::setw(15) << 20.00 << std::right << std::setw(15) << 92.30 << "\n";
//        nvh0 << std::right << std::setw(15) << 20.25 << std::right << std::setw(15) << 92.89 << "\n";
//        nvh0 << std::right << std::setw(15) << 20.50 << std::right << std::setw(15) << 93.49 << "\n";
//        nvh0 << std::right << std::setw(15) << 20.75 << std::right << std::setw(15) << 94.08 << "\n";
//        nvh0 << std::right << std::setw(15) << 21.00 << std::right << std::setw(15) << 94.67 << "\n";
//        nvh0 << std::right << std::setw(15) << 21.25 << std::right << std::setw(15) << 95.25 << "\n";
//        nvh0 << std::right << std::setw(15) << 21.50 << std::right << std::setw(15) << 95.82 << "\n";
//        nvh0 << std::right << std::setw(15) << 21.75 << std::right << std::setw(15) << 96.39 << "\n";
//        nvh0 << std::right << std::setw(15) << 22.00 << std::right << std::setw(15) << 96.96 << "\n";
//        nvh0 << std::right << std::setw(15) << 22.25 << std::right << std::setw(15) << 97.51 << "\n";
//        nvh0 << std::right << std::setw(15) << 22.50 << std::right << std::setw(15) << 98.07 << "\n";
//        nvh0 << std::right << std::setw(15) << 22.75 << std::right << std::setw(15) << 98.62 << "\n";
//        nvh0 << std::right << std::setw(15) << 23.00 << std::right << std::setw(15) << 99.17 << "\n";
//        nvh0 << std::right << std::setw(15) << 23.25 << std::right << std::setw(15) << 99.71 << "\n";
//        nvh0 << std::right << std::setw(15) << 23.50 << std::right << std::setw(15) << 100.24 << "\n";
//        nvh0 << std::right << std::setw(15) << 23.75 << std::right << std::setw(15) << 100.78 << "\n";
//        nvh0 << std::right << std::setw(15) << 24.00 << std::right << std::setw(15) << 101.31 << "\n";
//        nvh0 << std::right << std::setw(15) << 24.25 << std::right << std::setw(15) << 101.84 << "\n";
//        nvh0 << std::right << std::setw(15) << 24.50 << std::right << std::setw(15) << 102.36 << "\n";
//        nvh0 << std::right << std::setw(15) << 24.75 << std::right << std::setw(15) << 102.88 << "\n";
//        nvh0 << std::right << std::setw(15) << 25.00 << std::right << std::setw(15) << 103.40 << "\n";
//        nvh0 << std::right << std::setw(15) << 25.25 << std::right << std::setw(15) << 103.91 << "\n";
//        nvh0 << std::right << std::setw(15) << 25.50 << std::right << std::setw(15) << 104.42 << "\n";
//        nvh0 << std::right << std::setw(15) << 25.75 << std::right << std::setw(15) << 104.92 << "\n";
//        nvh0 << std::right << std::setw(15) << 26.00 << std::right << std::setw(15) << 105.43 << "\n";
//        nvh0 << std::right << std::setw(15) << 26.25 << std::right << std::setw(15) << 105.92 << "\n";
//        nvh0 << std::right << std::setw(15) << 26.50 << std::right << std::setw(15) << 106.42 << "\n";
//        nvh0 << std::right << std::setw(15) << 26.75 << std::right << std::setw(15) << 106.91 << "\n";
//        nvh0 << std::right << std::setw(15) << 27.00 << std::right << std::setw(15) << 107.41 << "\n";
//        nvh0 << std::right << std::setw(15) << 27.25 << std::right << std::setw(15) << 107.89 << "\n";
//        nvh0 << std::right << std::setw(15) << 27.50 << std::right << std::setw(15) << 108.37 << "\n";
//        nvh0 << std::right << std::setw(15) << 27.75 << std::right << std::setw(15) << 108.85 << "\n";
//        nvh0 << std::right << std::setw(15) << 28.00 << std::right << std::setw(15) << 109.33 << "\n";
//        nvh0 << std::right << std::setw(15) << 28.25 << std::right << std::setw(15) << 109.79 << "\n";
//        nvh0 << std::right << std::setw(15) << 28.50 << std::right << std::setw(15) << 110.26 << "\n";
//        nvh0 << std::right << std::setw(15) << 28.75 << std::right << std::setw(15) << 110.73 << "\n";
//        nvh0 << std::right << std::setw(15) << 29.00 << std::right << std::setw(15) << 111.19 << "\n";
//        nvh0 << std::right << std::setw(15) << 29.25 << std::right << std::setw(15) << 111.64 << "\n";
//        nvh0 << std::right << std::setw(15) << 29.50 << std::right << std::setw(15) << 112.10 << "\n";
//        nvh0 << std::right << std::setw(15) << 29.75 << std::right << std::setw(15) << 112.55 << "\n";
//        nvh0 << std::right << std::setw(15) << 30.00 << std::right << std::setw(15) << 113.00 << "\n";
//        nvh0 << std::right << std::setw(15) << 30.25 << std::right << std::setw(15) << 113.44 << "\n";
//        nvh0 << std::right << std::setw(15) << 30.50 << std::right << std::setw(15) << 113.88 << "\n";
//        nvh0 << std::right << std::setw(15) << 30.75 << std::right << std::setw(15) << 114.31 << "\n";
//        nvh0 << std::right << std::setw(15) << 31.00 << std::right << std::setw(15) << 114.75 << "\n";
//        nvh0 << std::right << std::setw(15) << 31.25 << std::right << std::setw(15) << 115.17 << "\n";
//        nvh0 << std::right << std::setw(15) << 31.50 << std::right << std::setw(15) << 115.60 << "\n";
//        nvh0 << std::right << std::setw(15) << 31.75 << std::right << std::setw(15) << 116.02 << "\n";
//        nvh0 << std::right << std::setw(15) << 32.00 << std::right << std::setw(15) << 116.45 << "\n";
//        nvh0 << std::right << std::setw(15) << 32.25 << std::right << std::setw(15) << 116.86 << "\n";
//        nvh0 << std::right << std::setw(15) << 32.50 << std::right << std::setw(15) << 117.27 << "\n";
//        nvh0 << std::right << std::setw(15) << 32.75 << std::right << std::setw(15) << 117.69 << "\n";
//        nvh0 << std::right << std::setw(15) << 33.00 << std::right << std::setw(15) << 118.10 << "\n";
//        nvh0 << std::right << std::setw(15) << 33.25 << std::right << std::setw(15) << 118.50 << "\n";
//        nvh0 << std::right << std::setw(15) << 33.50 << std::right << std::setw(15) << 118.91 << "\n";
//        nvh0 << std::right << std::setw(15) << 33.75 << std::right << std::setw(15) << 119.31 << "\n";
//        nvh0 << std::right << std::setw(15) << 34.00 << std::right << std::setw(15) << 119.71 << "\n";
//        nvh0 << std::right << std::setw(15) << 34.25 << std::right << std::setw(15) << 120.11 << "\n";
//        nvh0 << std::right << std::setw(15) << 34.50 << std::right << std::setw(15) << 120.51 << "\n";
//        nvh0 << std::right << std::setw(15) << 34.75 << std::right << std::setw(15) << 120.90 << "\n";
//        nvh0 << std::right << std::setw(15) << 35.00 << std::right << std::setw(15) << 121.30 << "\n";
//        nvh0 << std::right << std::setw(15) << 35.25 << std::right << std::setw(15) << 121.69 << "\n";
//        nvh0 << std::right << std::setw(15) << 35.50 << std::right << std::setw(15) << 122.08 << "\n";
//        nvh0 << std::right << std::setw(15) << 35.75 << std::right << std::setw(15) << 122.47 << "\n";
//        nvh0 << std::right << std::setw(15) << 36.00 << std::right << std::setw(15) << 122.86 << "\n";
//        nvh0 << std::right << std::setw(15) << 36.25 << std::right << std::setw(15) << 123.24 << "\n";
//        nvh0 << std::right << std::setw(15) << 36.50 << std::right << std::setw(15) << 123.63 << "\n";
//        nvh0 << std::right << std::setw(15) << 36.75 << std::right << std::setw(15) << 124.01 << "\n";
//        nvh0 << std::right << std::setw(15) << 37.00 << std::right << std::setw(15) << 124.39 << "\n";
//        nvh0 << std::right << std::setw(15) << 37.25 << std::right << std::setw(15) << 124.77 << "\n";
//        nvh0 << std::right << std::setw(15) << 37.50 << std::right << std::setw(15) << 125.15 << "\n";
//        nvh0 << std::right << std::setw(15) << 37.75 << std::right << std::setw(15) << 125.52 << "\n";
//        nvh0 << std::right << std::setw(15) << 38.00 << std::right << std::setw(15) << 125.90 << "\n";
//        nvh0 << std::right << std::setw(15) << 38.25 << std::right << std::setw(15) << 126.27 << "\n";
//        nvh0 << std::right << std::setw(15) << 38.50 << std::right << std::setw(15) << 126.63 << "\n";
//        nvh0 << std::right << std::setw(15) << 38.75 << std::right << std::setw(15) << 127.00 << "\n";
//        nvh0 << std::right << std::setw(15) << 39.00 << std::right << std::setw(15) << 127.37 << "\n";
//        nvh0 << std::right << std::setw(15) << 39.25 << std::right << std::setw(15) << 127.73 << "\n";
//        nvh0 << std::right << std::setw(15) << 39.50 << std::right << std::setw(15) << 128.08 << "\n";
//        nvh0 << std::right << std::setw(15) << 39.75 << std::right << std::setw(15) << 128.44 << "\n";
//        nvh0 << std::right << std::setw(15) << 40.00 << std::right << std::setw(15) << 128.80 << "\n";
//        nvh0 << std::right << std::setw(15) << 40.25 << std::right << std::setw(15) << 129.15 << "\n";
//        nvh0 << std::right << std::setw(15) << 40.50 << std::right << std::setw(15) << 129.50 << "\n";
//        nvh0 << std::right << std::setw(15) << 40.75 << std::right << std::setw(15) << 129.85 << "\n";
//        nvh0 << std::right << std::setw(15) << 41.00 << std::right << std::setw(15) << 130.20 << "\n";
//        nvh0 << std::right << std::setw(15) << 41.25 << std::right << std::setw(15) << 130.54 << "\n";
//        nvh0 << std::right << std::setw(15) << 41.50 << std::right << std::setw(15) << 130.88 << "\n";
//        nvh0 << std::right << std::setw(15) << 41.75 << std::right << std::setw(15) << 131.22 << "\n";
//        nvh0 << std::right << std::setw(15) << 42.00 << std::right << std::setw(15) << 131.56 << "\n";
//        nvh0 << std::right << std::setw(15) << 42.25 << std::right << std::setw(15) << 131.89 << "\n";
//        nvh0 << std::right << std::setw(15) << 42.50 << std::right << std::setw(15) << 132.23 << "\n";
//        nvh0 << std::right << std::setw(15) << 42.75 << std::right << std::setw(15) << 132.56 << "\n";
//        nvh0 << std::right << std::setw(15) << 43.00 << std::right << std::setw(15) << 132.89 << "\n";
//        nvh0 << std::right << std::setw(15) << 43.25 << std::right << std::setw(15) << 133.22 << "\n";
//        nvh0 << std::right << std::setw(15) << 43.50 << std::right << std::setw(15) << 133.55 << "\n";
//        nvh0 << std::right << std::setw(15) << 43.75 << std::right << std::setw(15) << 133.88 << "\n";
//        nvh0 << std::right << std::setw(15) << 44.00 << std::right << std::setw(15) << 134.21 << "\n";
//        nvh0 << std::right << std::setw(15) << 44.25 << std::right << std::setw(15) << 134.53 << "\n";
//        nvh0 << std::right << std::setw(15) << 44.50 << std::right << std::setw(15) << 134.85 << "\n";
//        nvh0 << std::right << std::setw(15) << 44.75 << std::right << std::setw(15) << 135.18 << "\n";
//        nvh0 << std::right << std::setw(15) << 45.00 << std::right << std::setw(15) << 135.50 << "\n";
//        nvh0 << std::right << std::setw(15) << 45.25 << std::right << std::setw(15) << 135.82 << "\n";
//        nvh0 << std::right << std::setw(15) << 45.50 << std::right << std::setw(15) << 136.14 << "\n";
//        nvh0 << std::right << std::setw(15) << 45.75 << std::right << std::setw(15) << 136.46 << "\n";
//        nvh0 << std::right << std::setw(15) << 46.00 << std::right << std::setw(15) << 136.78 << "\n";
//        nvh0 << std::right << std::setw(15) << 46.25 << std::right << std::setw(15) << 137.09 << "\n";
//        nvh0 << std::right << std::setw(15) << 46.50 << std::right << std::setw(15) << 137.41 << "\n";
//        nvh0 << std::right << std::setw(15) << 46.75 << std::right << std::setw(15) << 137.72 << "\n";
//        nvh0 << std::right << std::setw(15) << 47.00 << std::right << std::setw(15) << 138.04 << "\n";
//        nvh0 << std::right << std::setw(15) << 47.25 << std::right << std::setw(15) << 138.35 << "\n";
//        nvh0 << std::right << std::setw(15) << 47.50 << std::right << std::setw(15) << 138.66 << "\n";
//        nvh0 << std::right << std::setw(15) << 47.75 << std::right << std::setw(15) << 138.97 << "\n";
//        nvh0 << std::right << std::setw(15) << 48.00 << std::right << std::setw(15) << 139.28 << "\n";
//        nvh0 << std::right << std::setw(15) << 48.25 << std::right << std::setw(15) << 139.58 << "\n";
//        nvh0 << std::right << std::setw(15) << 48.50 << std::right << std::setw(15) << 139.89 << "\n";
//        nvh0 << std::right << std::setw(15) << 48.75 << std::right << std::setw(15) << 140.19 << "\n";
//        nvh0 << std::right << std::setw(15) << 49.00 << std::right << std::setw(15) << 140.50 << "\n";
//        nvh0 << std::right << std::setw(15) << 49.25 << std::right << std::setw(15) << 140.80 << "\n";
//        nvh0 << std::right << std::setw(15) << 49.50 << std::right << std::setw(15) << 141.10 << "\n";
//        nvh0 << std::right << std::setw(15) << 49.75 << std::right << std::setw(15) << 141.40 << "\n";
//        nvh0 << std::right << std::setw(15) << 50.00 << std::right << std::setw(15) << 141.70;

//        nvh0.close();

//        std::fstream nvh0_arm((Configuration::examFilePath + "NVH0_ARM").c_str(), std::fstream::out | std::fstream::binary);
//        if(nvh0_arm.fail()){
//            PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, (Configuration::examFilePath + "NVH0_ARM").c_str());
//            return false;
//        }

//        nvh0_arm << RobotParams::robotType << "\n";
//        nvh0_arm << 'R' << "\n";
//        nvh0_arm << std::right << std::setw(15) << 0;
//        nvh0_arm << std::right << std::setw(15) << 0;
//        nvh0_arm << std::right << std::setw(15) << 300;
//        nvh0_arm << std::right << std::setw(15) << 0;
//        nvh0_arm << std::right << std::setw(15) << 0 << "\n";
//        nvh0_arm << 'M' << "\n";
//        nvh0_arm << std::right << std::setw(15) << 55 << std::right << std::setw(15) << 1200 << "\n";
//        nvh0_arm << std::right << std::setw(15) << 1 << std::right << std::setw(15) << 1000000 << std::right << std::setw(15) << 1 << "\n";
//        nvh0_arm << std::right << std::setw(15) << 10 << std::right << std::setw(15) << 10 << "\n";
//        nvh0_arm << std::right << std::setw(15) << 0 << std::right << std::setw(15) << 0;

//        nvh0_arm.close();
//        break;
//    }
//    case 1:
//    {
//        std::fstream nvh1((Configuration::examFilePath + "NVH1").c_str(), std::fstream::out | std::fstream::binary);
//        if(nvh1.fail()){
//            PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, (Configuration::examFilePath + "NVH1").c_str());
//            return false;
//        }

//        nvh1 << RobotParams::robotType << "\n";
//        nvh1 << 'R' << "\n";
//        nvh1 << std::right << std::setw(15) << 0;
//        nvh1 << std::right << std::setw(15) << 0;
//        nvh1 << std::right << std::setw(15) << 300;
//        nvh1 << std::right << std::setw(15) << 0;
//        nvh1 << std::right << std::setw(15) << 0 << "\n";
//        nvh1 << 'M' << "\n";
//        nvh1 << std::right << std::setw(15) << 2 << "\n";
//        nvh1 << std::right << std::setw(15) << 0.00 << std::right << std::setw(15) << 8.00 << "\n";
//        nvh1 << std::right << std::setw(15) << QString::number(RobotParams::nvh_P2t, 'f', 2).toStdString() << std::right << std::setw(15) << QString::number(RobotParams::nvh_P2v, 'f', 2).toStdString();

//        nvh1.close();

//        std::fstream nvh1_arm((Configuration::examFilePath + "NVH1_ARM").c_str(), std::fstream::out | std::fstream::binary);
//        if(nvh1_arm.fail()){
//            PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, (Configuration::examFilePath + "NVH1_ARM").c_str());
//            return false;
//        }

//        nvh1_arm << RobotParams::robotType << "\n";
//        nvh1_arm << 'R' << "\n";
//        nvh1_arm << std::right << std::setw(15) << 0;
//        nvh1_arm << std::right << std::setw(15) << 0;
//        nvh1_arm << std::right << std::setw(15) << 300;
//        nvh1_arm << std::right << std::setw(15) << 0;
//        nvh1_arm << std::right << std::setw(15) << 0 << "\n";
//        nvh1_arm << 'M' << "\n";
//        nvh1_arm << std::right << std::setw(15) << QString::number(RobotParams::nvh_P2t + 5.0, 'f', 2).toStdString() << std::right << std::setw(15) << 1200 << "\n";
//        nvh1_arm << std::right << std::setw(15) << 1 << std::right << std::setw(15) << 1000000 << std::right << std::setw(15) << 1 << "\n";
//        nvh1_arm << std::right << std::setw(15) << 10 << std::right << std::setw(15) << 10 << "\n";
//        nvh1_arm << std::right << std::setw(15) << 0 << std::right << std::setw(15) << 0;

//        nvh1_arm.close();
//        break;
//    }
//    case 2:
//    {
//        std::fstream nvh2((Configuration::examFilePath + "NVH2").c_str(), std::fstream::out | std::fstream::binary);
//        if(nvh2.fail()){
//            PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, (Configuration::examFilePath + "NVH2").c_str());
//            return false;
//        }

//        nvh2 << RobotParams::robotType << "\n";
//        nvh2 << 'R' << "\n";
//        nvh2 << std::right << std::setw(15) << 0;
//        nvh2 << std::right << std::setw(15) << 0;
//        nvh2 << std::right << std::setw(15) << 300;
//        nvh2 << std::right << std::setw(15) << 0;
//        nvh2 << std::right << std::setw(15) << 0 << "\n";
//        nvh2 << 'M' << "\n";
//        nvh2 << std::right << std::setw(15) << 2 << "\n";
//        nvh2 << std::right << std::setw(15) << 0.00 << std::right << std::setw(15) << 140.00 << "\n";
//        nvh2 << std::right << std::setw(15) << 50.00 << std::right << std::setw(15) << 140.00;

//        nvh2.close();

//        std::fstream nvh2_arm((Configuration::examFilePath + "NVH2_ARM").c_str(), std::fstream::out | std::fstream::binary);
//        if(nvh2_arm.fail()){
//            PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, (Configuration::examFilePath + "NVH2_ARM").c_str());
//            return false;
//        }

//        nvh2_arm << RobotParams::robotType << "\n";
//        nvh2_arm << 'R' << "\n";
//        nvh2_arm << std::right << std::setw(15) << 0;
//        nvh2_arm << std::right << std::setw(15) << 0;
//        nvh2_arm << std::right << std::setw(15) << 300;
//        nvh2_arm << std::right << std::setw(15) << 0;
//        nvh2_arm << std::right << std::setw(15) << 0 << "\n";
//        nvh2_arm << 'M' << "\n";
//        nvh2_arm << std::right << std::setw(15) << 50 << std::right << std::setw(15) << 1200 << "\n";
//        nvh2_arm << std::right << std::setw(15) << 1 << std::right << std::setw(15) << 1000000 << std::right << std::setw(15) << 1 << "\n";
//        nvh2_arm << std::right << std::setw(15) << 10 << std::right << std::setw(15) << 10 << "\n";
//        nvh2_arm << std::right << std::setw(15) << 0 << std::right << std::setw(15) << 0;

//        nvh2_arm.close();
//        break;
//    }
//    case 3:
//    {
//        std::fstream nvh3((Configuration::examFilePath + "NVH3").c_str(), std::fstream::out | std::fstream::binary);
//        if(nvh3.fail()){
//            PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, (Configuration::examFilePath + "NVH3").c_str());
//            return false;
//        }

//        nvh3 << RobotParams::robotType << "\n";
//        nvh3 << 'R' << "\n";
//        nvh3 << std::right << std::setw(15) << 0;
//        nvh3 << std::right << std::setw(15) << 0;
//        nvh3 << std::right << std::setw(15) << 300;
//        nvh3 << std::right << std::setw(15) << 0;
//        nvh3 << std::right << std::setw(15) << 0 << "\n";
//        nvh3 << 'M' << "\n";
//        nvh3 << std::right << std::setw(15) << 2 << "\n";
//        nvh3 << std::right << std::setw(15) << 0.00 << std::right << std::setw(15) << QString::number(RobotParams::nvh_P1v, 'f', 2).toStdString() << "\n";
//        nvh3 << std::right << std::setw(15) << QString::number(RobotParams::nvh_P2t, 'f', 2).toStdString() << std::right << std::setw(15) << QString::number(RobotParams::nvh_P2v, 'f', 2).toStdString();

//        nvh3.close();

//        std::fstream nvh3_arm((Configuration::examFilePath + "NVH3_ARM").c_str(), std::fstream::out | std::fstream::binary);
//        if(nvh3_arm.fail()){
//            PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, (Configuration::examFilePath + "NVH3_ARM").c_str());
//            return false;
//        }

//        nvh3_arm << RobotParams::robotType << "\n";
//        nvh3_arm << 'R' << "\n";
//        nvh3_arm << std::right << std::setw(15) << 0;
//        nvh3_arm << std::right << std::setw(15) << 0;
//        nvh3_arm << std::right << std::setw(15) << 300;
//        nvh3_arm << std::right << std::setw(15) << 0;
//        nvh3_arm << std::right << std::setw(15) << 0 << "\n";
//        nvh3_arm << 'M' << "\n";
//        nvh3_arm << std::right << std::setw(15) << QString::number(RobotParams::nvh_P2t + 5.0 + 50.0, 'f', 2).toStdString() << std::right << std::setw(15) << 1200 << "\n";
//        nvh3_arm << std::right << std::setw(15) << 1 << std::right << std::setw(15) << 1000000 << std::right << std::setw(15) << 1 << "\n";
//        nvh3_arm << std::right << std::setw(15) << 10 << std::right << std::setw(15) << 10 << "\n";
//        nvh3_arm << std::right << std::setw(15) << 0 << std::right << std::setw(15) << 0;

//        nvh3_arm.close();
//        break;
//    }
//    default:
//        break;
//    }

//    return true;
//}

///* <ACD> */

////修改1//////////////////////////////////////////////////////////////
//void PedalRobotUI::on_pushButton_saveModelSelect_clicked()
//{
//    pdRobot->modelSelect=ui->comboBox_modelSelect->currentIndex();
//    std::cout<<ui->comboBox_modelSelect->currentIndex()<<std::endl;

//    //mySysControl->systemModelSelect=pdRobot->modelSelect;

//    if(pdRobot->modelSelect==1 || pdRobot->modelSelect==2){
//        pdRobot->intialMySysControl();
//    }

//    //修改6/////////////////////////////////////////////////////////////////////
//    if(pdRobot->modelSelect==1){//ACD模式1 Online
//        std::cout<<" myGetMTApp "<<pdRobot->myGetMTApp<<std::endl;
//        std::cout<<" myGetTransReqGr "<<pdRobot->myGetTransReqGr<<std::endl;
//        std::cout<<" myGetCmndMd "<<pdRobot->myGetCmndMd<<std::endl;
//        std::cout<<" myGetVehStSpd "<<pdRobot->myGetVehStSpd<<std::endl;
//        std::cout<<" myGetAcclStPos "<<pdRobot->myGetAcclStPos<<std::endl;
//        std::cout<<" myGetAcclDlyTmr "<<pdRobot->myGetAcclDlyTmr<<std::endl;
//        std::cout<<" myGetBrkPdlStPos "<<pdRobot->myGetBrkPdlStPos<<std::endl;
//        std::cout<<" myGetBrkPdlDlyTmr "<<pdRobot->myGetBrkPdlDlyTmr<<std::endl;
//        //测试can 输出之前的数据
//        std::cout<<" brakeOpenValue "<<pdRobot->GetBrakePosition()<<std::endl;
//        std::cout<<" acceleratorOpenValue "<<pdRobot->GetAcceleratorPosition()<<std::endl;
//        std::cout<<" usbCanCarSpeed "<<pdRobot->GetCanCarSpeed()<<std::endl;
//        std::cout<<" powerMode "<<pdRobot->GetPowerMode()<<std::endl;

//    }

//    //int myGetMTApp; //是否手动挡
//    //int myGetTransReqGr; //手动档位
//    //int myGetCmndMd; //控制模式
//    //double myGetVehStSpd; //目标速度
//    //double myGetAcclStPos; //目标油门开度
//    //double myGetAcclDlyTmr; //油门延迟时间
//    //double myGetBrkPdlStPos; //目标刹车开度
//    //double myGetBrkPdlDlyTmr; //刹车延迟时间
//    //修改6/////////////////////////////////////////////////////////////////////
//}
////修改1//////////////////////////////////////////////////////////////

////修改1//////////////////////////////////////////////////////////////
//void PedalRobotUI::on_pushButton_refresh_clicked()  //ACD模式2 Predefined模式 //“保存”按钮
//{
//    pdRobot->controlSelect=ui->comboBox_controlSelect->currentIndex();
//    std::cout<<ui->comboBox_controlSelect->currentIndex()<<std::endl;

//    pdRobot->carSelect=ui->comboBox_carSelect->currentIndex();
//    std::cout<<ui->comboBox_carSelect->currentIndex()<<std::endl;

//    QString readPredefSetValueString=ui->lineEdit_predefSetValue->text();
//    pdRobot->predefSetValue=readPredefSetValueString.toDouble();
//    QString readPredefDelayTimeString=ui->lineEdit_predefDelayTime->text();
//    pdRobot->predefDelayTime=readPredefDelayTimeString.toDouble();
//    std::cout<<pdRobot->predefSetValue<<std::endl;
//    std::cout<<pdRobot->predefDelayTime<<std::endl;

///*
//    if(pdRobot->controlSelect==0)
//    {
//        pdRobot->startCarSpeed=true;
//        pdRobot->startAccOpenValue=false;
//        pdRobot->startBrakeOpenValue=false;
//        pdRobot->readCarSpeed=pdRobot->predefSetValue;
//    }
//    else if(pdRobot->controlSelect==1)
//    {
//        pdRobot->startCarSpeed=false;
//        pdRobot->startAccOpenValue=true;
//        pdRobot->startBrakeOpenValue=false;
//        pdRobot->readAccOpenValue=pdRobot->predefSetValue;
//        pdRobot->readAccDelayTime=pdRobot->predefDelayTime;
//        pdRobot->initConTime=QTime::currentTime();
//    }
//    else if(pdRobot->controlSelect==2)
//    {
//        pdRobot->startCarSpeed=false;
//        pdRobot->startAccOpenValue=false;
//        pdRobot->startBrakeOpenValue=true;
//        pdRobot->readBrakeOpenValue=pdRobot->predefSetValue;
//        pdRobot->readBrakeDelayTime=pdRobot->predefDelayTime;
//        pdRobot->initConTime=QTime::currentTime();
//    }
//    else
//    {
//        pdRobot->startCarSpeed=false;
//        pdRobot->startAccOpenValue=false;
//        pdRobot->startBrakeOpenValue=false;
//        QMessageBox::information(NULL, "Warning", QObject::tr("请选择并保存刷新控制方式!"));
//    }
//*/
//}
////修改1//////////////////////////////////////////////////////////////

////修改1//////////////////////////////////////////////////////////////
//void PedalRobotUI::on_pushButton_startACD_clicked()  //“Predefined运行”按钮
//{
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

//    if ( !pdRobot->SelectSpeedCurveACD( true ) )
//    {
//        return;
//    }

//    //修改2//////////////////////////////////////////////////////////////
//    if(pdRobot->controlSelect==0)
//    {
//        pdRobot->startCarSpeed=true;
//        pdRobot->startAccOpenValue=false;
//        pdRobot->startBrakeOpenValue=false;
//        pdRobot->readCarSpeed=pdRobot->predefSetValue;
//        pdRobot->timeStart=floor(pdRobot->GetElapsedSeconds());
//        pdRobot->speedStart=pdRobot->GetMyCarSpeed();
//        pdRobot->intialMySysSpeed();
//    }
//    else if(pdRobot->controlSelect==1)
//    {
//        pdRobot->startCarSpeed=false;
//        pdRobot->startAccOpenValue=true;
//        pdRobot->startBrakeOpenValue=false;
//        pdRobot->readAccOpenValue=pdRobot->predefSetValue;
//        pdRobot->readAccDelayTime=pdRobot->predefDelayTime;
//        pdRobot->initConTime=QTime::currentTime();

//        //修改11////////////////////////////////////////////////////////////////////
//        pdRobot->scara1AccStart=RobotParams::angleRealTime[1];
//        //修改11////////////////////////////////////////////////////////////////////

//    }
//    else if(pdRobot->controlSelect==2)
//    {
//        pdRobot->startCarSpeed=false;
//        pdRobot->startAccOpenValue=false;
//        pdRobot->startBrakeOpenValue=true;
//        pdRobot->readBrakeOpenValue=pdRobot->predefSetValue;
//        pdRobot->readBrakeDelayTime=pdRobot->predefDelayTime;
//        pdRobot->initConTime=QTime::currentTime();

//        //修改11////////////////////////////////////////////////////////////////////
//        pdRobot->scara0BrakeStart=RobotParams::angleRealTime[0];
//        //修改11////////////////////////////////////////////////////////////////////

//    }
//    else
//    {
//        pdRobot->startCarSpeed=false;
//        pdRobot->startAccOpenValue=false;
//        pdRobot->startBrakeOpenValue=false;
//        QMessageBox::information(NULL, "Warning", QObject::tr("请选择并保存控制方式!"));
//    }
//    //修改2//////////////////////////////////////////////////////////////

//    std::string fileContent = FileAssistantFunc::ReadFileContent(conf->defaultFile);
//    if(fileContent.empty()){
//        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
//        return;
//    }
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

//    PRINTF(LOG_DEBUG, "%s: starts.\n", __func__);
//    pdRobot->StartQCustomPlot(conf->defaultFile);

//    //显示NEDC/WLTC的曲线类型
//    //const std::string fileName = NormalFile::GetFileName(conf->defaultFile.c_str());//XXX_ARM
//    //const std::string curveType = fileName.substr(0,fileName.length()-4);//XXX
//    //std::string curveTypeLast = "";
//    //if(curveTypeLast != curveType){
//    //    curveTypeLast = curveType;
//    //    ui->lineEdit_curveType->setText(curveType.c_str());
//    //}
//}
////修改1//////////////////////////////////////////////////////////////

////修改1//////////////////////////////////////////////////////////////
//void PedalRobotUI::on_pushButton_stopAndLeft_clicked()
//{
//    // 停止
//    on_pushButton_softStop_clicked();
//    // 上抬踏板
//    RefreshSoftStopFile(); // 按照车型配置文件更新softStop.txt
//    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::softStopFilePath);
//    if(fileContent.empty()){
//        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
//        return;
//    }
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
//}
////修改1//////////////////////////////////////////////////////////////

////修改1//////////////////////////////////////////////////////////////
//void PedalRobotUI::on_pushButton_startACDonline_clicked()   //“Online运行”按钮
//{
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

//    if ( !pdRobot->SelectSpeedCurveACD( true ) )
//    {
//        return;
//    }
//    std::string fileContent = FileAssistantFunc::ReadFileContent(conf->defaultFile);
//    if(fileContent.empty()){
//        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
//        return;
//    }
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

//    PRINTF(LOG_DEBUG, "%s: starts.\n", __func__);
//    pdRobot->StartQCustomPlot(conf->defaultFile);

//    //显示NEDC/WLTC的曲线类型
//    //const std::string fileName = NormalFile::GetFileName(conf->defaultFile.c_str());//XXX_ARM
//    //const std::string curveType = fileName.substr(0,fileName.length()-4);//XXX
//    //std::string curveTypeLast = "";
//    //if(curveTypeLast != curveType){
//    //    curveTypeLast = curveType;
//    //    ui->lineEdit_curveType->setText(curveType.c_str());
//    //}
//}
////修改1//////////////////////////////////////////////////////////////

////修改2//////////////////////////////////////////////////////////////
//void PedalRobotUI::on_pushButton_refreshPredef_clicked()  //ACD模式2 Predefined模式 //“刷新”按钮
//{
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

//    if ( !pdRobot->SelectSpeedCurveACD( false ) )
//    {
//        return;
//    }

//    pdRobot->controlSelect=ui->comboBox_controlSelect->currentIndex();
//    std::cout<<ui->comboBox_controlSelect->currentIndex()<<std::endl;

//    pdRobot->carSelect=ui->comboBox_carSelect->currentIndex();
//    std::cout<<" carSelect "<<ui->comboBox_carSelect->currentIndex()<<std::endl;

//    QString readPredefSetValueString=ui->lineEdit_predefSetValue->text();
//    pdRobot->predefSetValue=readPredefSetValueString.toDouble();
//    QString readPredefDelayTimeString=ui->lineEdit_predefDelayTime->text();
//    pdRobot->predefDelayTime=readPredefDelayTimeString.toDouble();
//    std::cout<<pdRobot->predefSetValue<<std::endl;
//    std::cout<<pdRobot->predefDelayTime<<std::endl;

//    if(pdRobot->controlSelect==0)
//    {
//        pdRobot->startCarSpeed=true;
//        pdRobot->startAccOpenValue=false;
//        pdRobot->startBrakeOpenValue=false;
//        pdRobot->readCarSpeed=pdRobot->predefSetValue;
//        pdRobot->timeStart=floor(pdRobot->GetElapsedSeconds());
//        pdRobot->speedStart=pdRobot->GetMyCarSpeed();
//        pdRobot->intialMySysSpeed();
//    }
//    else if(pdRobot->controlSelect==1)
//    {
//        pdRobot->startCarSpeed=false;
//        pdRobot->startAccOpenValue=true;
//        pdRobot->startBrakeOpenValue=false;
//        pdRobot->readAccOpenValue=pdRobot->predefSetValue;
//        pdRobot->readAccDelayTime=pdRobot->predefDelayTime;
//        pdRobot->initConTime=QTime::currentTime();

//        //修改11////////////////////////////////////////////////////////////////////
//        pdRobot->scara1AccStart=RobotParams::angleRealTime[1];
//        //修改11////////////////////////////////////////////////////////////////////

//    }
//    else if(pdRobot->controlSelect==2)
//    {
//        pdRobot->startCarSpeed=false;
//        pdRobot->startAccOpenValue=false;
//        pdRobot->startBrakeOpenValue=true;
//        pdRobot->readBrakeOpenValue=pdRobot->predefSetValue;
//        pdRobot->readBrakeDelayTime=pdRobot->predefDelayTime;
//        pdRobot->initConTime=QTime::currentTime();

//        //修改11////////////////////////////////////////////////////////////////////
//        pdRobot->scara0BrakeStart=RobotParams::angleRealTime[0];
//        //修改11////////////////////////////////////////////////////////////////////

//    }
//    else
//    {
//        pdRobot->startCarSpeed=false;
//        pdRobot->startAccOpenValue=false;
//        pdRobot->startBrakeOpenValue=false;
//        QMessageBox::information(NULL, "Warning", QObject::tr("请选择并保存刷新控制方式!"));
//    }

//    std::string fileContent = FileAssistantFunc::ReadFileContent(conf->defaultFile);
//    if(fileContent.empty()){
//        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
//        return;
//    }
//    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

//    PRINTF(LOG_DEBUG, "%s: starts.\n", __func__);
//    pdRobot->StartQCustomPlot(conf->defaultFile);

//    //显示NEDC/WLTC的曲线类型
//    //const std::string fileName = NormalFile::GetFileName(conf->defaultFile.c_str());//XXX_ARM
//    //const std::string curveType = fileName.substr(0,fileName.length()-4);//XXX
//    //std::string curveTypeLast = "";
//    //if(curveTypeLast != curveType){
//    //    curveTypeLast = curveType;
//    //    ui->lineEdit_curveType->setText(curveType.c_str());
//    //}
//}
////修改2//////////////////////////////////////////////////////////////

////修改sy//////////////////////////////////////////////////////////////
//void PedalRobotUI::on_tabWidget_currentChanged(int index)
//{
//    if (index == 4)
//    {
//        if (RobotParams::currentshiftvalue == "/")
//        {
//            QMessageBox::warning(NULL, tr("警告"), tr("手动挡车辆请先正确测试挡位！"));
//            ui->tabWidget->setCurrentIndex(0);
//            return;
//        }

//        bool re = true;
//        if (!RobotParams::ifGoHome) re = false;
//        if (!RobotParams::ifConfirmSC) re = false;
//        if ( RobotParams::switchflag[0] || RobotParams::switchflag[1] || RobotParams::switchflag[2] || RobotParams::switchflag[3] || RobotParams::switchflag[4] || RobotParams::switchflag[6] || RobotParams::switchflag[7] || RobotParams::switchflag[8] || RobotParams::switchflag[9] || RobotParams::isExaming ) re = false;
//        if (RobotParams::powerMode!=PedalRobot::Run) re = false;
//        if (!re)
//        {
//            QMessageBox::information(NULL,"提示", QString("请检查回原状态、挡位离合信息确认状态以及是否有其他模式正在运行后，发动车辆再重试！"));
//            ui->tabWidget->setCurrentIndex(0);
//        }
//        else
//        {
//            if (!RobotParams::switchflag[5])
//            {
//                QMessageBox::information(NULL,"提示", QString("ACD模式下请按停止回抬按钮后再尝试转到其他模式！"));
//                RobotParams::switchflag[5] = true;

//                //ACD挡位界面设置
//                QString shiftnow = QString::fromStdString(RobotParams::currentshiftvalue);
//                if (shiftnow == QString("N_1&2") || shiftnow == QString("N_3&4") || shiftnow == QString("N_5&6"))
//                {
//                    shiftnow = QString("N");
//                }
//                ui->lineEdit_cshift->setText( shiftnow );
//                ui->lineEdit_cc->setText( QString::fromStdString(RobotParams::currentclutchvalue) );
//                ui->radioButton_cs2->setChecked(true);

//                ui->comboBox_ashift->clear();
//                ui->comboBox_ashift->insertItem(0, "N");
//                ui->comboBox_ashift->insertItem(1, "1"); // 3 = 1 + 2
//                ui->comboBox_ashift->insertItem(2, "2");
//                ui->comboBox_ashift->insertItem(3, "3");
//                ui->comboBox_ashift->insertItem(4, "4");
//                ui->comboBox_ashift->insertItem(5, "5");
//                ui->comboBox_ashift->insertItem(6, "6");

//                if (conf->ifManualShift)
//                {
//                    ui->frame_shift->setEnabled(true);
//                }
//                else
//                {
//                    ui->frame_shift->setEnabled(false);
//                }
//            }
//        }
//    }
//    else if (index == 0 || index == 2)
//    {

//    }
//    else
//    {
//        if (RobotParams::switchflag[5])
//        {
//            ui->tabWidget->setCurrentIndex(4);
//        }
//    }
//}

//void PedalRobotUI::on_pushButton_change_clicked()
//{
//    if (!pdRobot->GetIsControlling())
//    {
//        QMessageBox::warning(NULL, tr("警告"), tr("ACD模式尚未运行，不能进行换挡，请先执行开度或速度跟踪！"));
//        return;
//    }

//    if (RobotParams::ifCSACD)
//    {
//        QMessageBox::warning(NULL, tr("警告"), tr("正在换挡，请稍后重试！"));
//        return;
//    }

//    unsigned int index_combo = ui->comboBox_ashift->currentIndex();
//    unsigned int index;

//    if (Configuration::GetInstance()->ifManualShift)
//    {
//        if (index_combo > 0)
//        {
//            index = index_combo + 2;
//        }
//        else
//        {
//            index = 1;
//        }

//        if ( (index == 3) && (RobotParams::currentshiftindex == 1) )
//        {
//            RobotParams::iffromNto1 = true;
//        }
//    }
//    else
//    {
//        index = index_combo;
//    }

//    // 记录目前踏板位置
//    RobotParams::tempVars[8] = RobotParams::angleRealTime[0];
//    RobotParams::tempVars[9] = RobotParams::angleRealTime[1];

//    // 规划路径
//    RobotParams::aimshiftindex = index;
//    if (RobotParams::aimshiftindex == RobotParams::currentshiftindex)
//    {
//        QMessageBox::information(NULL, tr("提示"), tr("已经到达该挡位！"));
//        return;
//    }

//    RobotParams::ifREBA = ui->radioButton_cs2->isChecked();

//    RobotParams::changeshiftstart = true;
//    RobotParams::changeshiftend = false;
//    RobotParams::round = 1;
//    RobotParams::round2 = 1;
//    RobotParams::changeshiftprocess = 0;
//    RobotParams::startchangeshifttimeflag = false;
//    RobotParams::ifCSACD = true;

//    return;
//}
////修改sy//////////////////////////////////////////////////////////////

///* </ACD> */





