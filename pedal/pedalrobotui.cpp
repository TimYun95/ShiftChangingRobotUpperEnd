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

    pdTimer = new QTimer(this);
    connect( pdTimer, SIGNAL( timeout() ), this, SLOT( PedalTimerDone() ) );
    pdTimer->start(RobotParams::UITimerMs);

    pdRobot = new PedalRobot(ui->pQCustomPlot, conf);

    InitWidgets();
    UpdateCarTypeWidget();

    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
}

PedalRobotUI::~PedalRobotUI()
{
    pdTimer->stop();

    delete pdRobot;
    delete pdTimer;
    delete stui;
    delete scui;

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

        // 运行曲线前换挡
        DoWorkCheckStatusBeforeCurveRunning();

        // rem=0 即18ms*3的控制周期 进行机器人的控制
        pdRobot->UpdatePart2();

        // 一轮曲线是否运行结束
        if (pdRobot->ifControllingOver() && !pdRobot->ifUnderControlling())
        {
            pdRobot->changeOverState();
            pauseflag = 0;
            checkflag = 11;
        }
        DoWorkAfterCurveRunning();

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
    if(sender() == ui->pushButton_plus1){
        axis=0; direction=1;
    }else if(sender() == ui->pushButton_plus2){
        axis=1; direction=1;
    }else if(sender() == ui->pushButton_minus1){
        axis=0; direction=-1;
    }else if(sender() == ui->pushButton_minus2){
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
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

    if ( !pdRobot->SelectSpeedCurve( !ui->checkBox_repeatCurveFile->isChecked() ) )
    {
        return;
    }

    checkIsOK = false;
    if (!CheckStatusBeforeCurveRunning()) return;

    QElapsedTimer et;
    et.start();
    while (et.elapsed() < 6000)
    {
        if (!checkIsOK) // 等待检查完成
        {
            QApplication::processEvents();
        }
        else break;
    }
    if (!checkIsOK)
    {
        QMessageBox::warning(NULL, tr("警告"), tr("挡位状态矫正失败\r\n请检查相关信息并重试！"));
        return;
    }

    if (QMessageBox::information(
                NULL,"提示", QObject::tr("点击确认后开始运行曲线!"),
                QObject::tr("确认"),QObject::tr("取消")) == 1) return;

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

bool PedalRobotUI::CheckStatusBeforeCurveRunning()
{
    // 挡位离合信息确认
    if (!scui->ifConfirmShiftClutchInfo)
    {
        QMessageBox::warning(NULL, tr("警告"), tr("请确认挡位信息后再运行曲线！"));
        return false;
    }

    // 手动挡确认换挡时刻信息
    if (Configuration::GetInstance()->ifManualShift && !scui->ifConfirmChangingShiftTime)
    {
        QMessageBox::warning(NULL, tr("警告"), tr("手动挡请确认换挡时刻后再运行曲线！"));
        return false;
    }

    // 手动归空挡提示
    int ret = QMessageBox::information(NULL, tr("提示"), tr("请把挡位拨到空挡附近！"), tr("确认"), tr("取消"));
    if(ret == 1)
    {
        return false;
    }

    // 保证在空挡
    if (!scui->GetIfAtNullShift())
    {
        QMessageBox::warning(NULL, tr("警告"), tr("挡位不在空挡附近\r\n请手动调整！"));
        return false;
    }

    pauseflag = 0;

    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

    // 空挡纠正
    checkflag = 1;

    return true;
}

void PedalRobotUI::DoWorkCheckStatusBeforeCurveRunning()
{
    if (checkflag < 0) return;

    switch (checkflag) {
    case 1:
        if (pauseflag == 0)
        {
            bool ifsuccesschangefile = false;
            if (Configuration::GetInstance()->ifManualShift)
            {
                QVector<QVector<double>> anglelist;
                QVector<double> liftpedalandclutch = {
                    Configuration::GetInstance()->deathPos[0],
                    Configuration::GetInstance()->deathPos[1],
                    Configuration::GetInstance()->clutchAngles[(int)scui->ClutchState::Released],
                    RobotParams::angleRealTime[3],
                    RobotParams::angleRealTime[4],
                    RobotParams::angleRealTime[5]};
                QVector<double> dragtoNshift = {
                    Configuration::GetInstance()->deathPos[0],
                    Configuration::GetInstance()->deathPos[1],
                    Configuration::GetInstance()->clutchAngles[(int)scui->ClutchState::Released],
                    Configuration::GetInstance()->shiftAxisAngles1[(int)scui->ManualShiftState::Gear_N],
                    Configuration::GetInstance()->shiftAxisAngles2[(int)scui->ManualShiftState::Gear_N],
                    RobotParams::angleRealTime[5]};
                anglelist.append(liftpedalandclutch);
                anglelist.append(dragtoNshift);
                ifsuccesschangefile = scui->SetTempArrivalFile(anglelist, 2);
            }
            else
            {
                QVector<QVector<double>> anglelist;
                QVector<double> pressbrkanddragtoNshift = {
                    Configuration::GetInstance()->limPos[0],
                    Configuration::GetInstance()->deathPos[1],
                    Configuration::GetInstance()->clutchAngles[(int)scui->ClutchState::Released],
                    Configuration::GetInstance()->shiftAxisAngles1[(int)scui->AutoShiftState::Gear_N],
                    Configuration::GetInstance()->shiftAxisAngles2[(int)scui->AutoShiftState::Gear_N],
                    RobotParams::angleRealTime[5]};
                QVector<double> dragtoDshift = {
                    Configuration::GetInstance()->limPos[0],
                    Configuration::GetInstance()->deathPos[1],
                    Configuration::GetInstance()->clutchAngles[(int)scui->ClutchState::Released],
                    Configuration::GetInstance()->shiftAxisAngles1[(int)scui->AutoShiftState::Gear_D],
                    Configuration::GetInstance()->shiftAxisAngles2[(int)scui->AutoShiftState::Gear_D],
                    RobotParams::angleRealTime[5]};
                anglelist.append(pressbrkanddragtoNshift);
                anglelist.append(dragtoDshift);
                ifsuccesschangefile = scui->SetTempArrivalFile(anglelist, 2);
            }

            if (!ifsuccesschangefile)
            {
                checkflag = -1;
                PRINTF(LOG_WARNING, "%s: initial file refresh failed.\n", __func__);
                return;
            }
            else
            {
                std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::temparrivalFilePath);
                if(fileContent.empty()){
                    checkflag = -1;
                    PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
                    return;
                }
                AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
                pauseflag++;
                return;
            }
        }

        if (pauseflag <= 8)
        {
            pauseflag++;
            return;
        }

        if (RobotParams::statusStrIndex != 28)
        {
            if (pauseflag > 100)
            {
                pauseflag = 0;
                checkflag = -1;
                PRINTF(LOG_WARNING, "%s: initial failed.\n", __func__);
                return;
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
                checkflag = -1;
                PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
                return;
            }
            AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

            checkflag = 2;
            PRINTF(LOG_INFO, "%s: be ready to initial.\n", __func__);
        }
        break;
    case 2:
        if (pauseflag <= 4) pauseflag++;
        else pauseflag = 0;
        checkflag = -1;
        checkIsOK = true;
        break;
    case -1:
    default:
        break;
    }
}

void PedalRobotUI::DoWorkAfterCurveRunning()
{
    if (checkflag < 0) return;

    switch (checkflag) {
    case 11:
        if (pauseflag == 0)
        {
            bool ifsuccesschangefile = false;
            if (Configuration::GetInstance()->ifManualShift)
            {
                QVector<QVector<double>> anglelist;
                QVector<double> liftpedalandclutch = {
                    Configuration::GetInstance()->deathPos[0],
                    Configuration::GetInstance()->deathPos[1],
                    Configuration::GetInstance()->clutchAngles[(int)scui->ClutchState::Released],
                    RobotParams::angleRealTime[3],
                    RobotParams::angleRealTime[4],
                    RobotParams::angleRealTime[5]};
                anglelist.append(liftpedalandclutch);
                ifsuccesschangefile = scui->SetTempArrivalFile(anglelist, 1);
            }
            else
            {
                QVector<QVector<double>> anglelist;
                QVector<double> pressbrkanddragtoNshift = {
                    Configuration::GetInstance()->limPos[0],
                    Configuration::GetInstance()->deathPos[1],
                    Configuration::GetInstance()->clutchAngles[(int)scui->ClutchState::Released],
                    Configuration::GetInstance()->shiftAxisAngles1[(int)scui->AutoShiftState::Gear_N],
                    Configuration::GetInstance()->shiftAxisAngles2[(int)scui->AutoShiftState::Gear_N],
                    RobotParams::angleRealTime[5]};
                anglelist.append(pressbrkanddragtoNshift);
                ifsuccesschangefile = scui->SetTempArrivalFile(anglelist, 1);
            }

            if (!ifsuccesschangefile)
            {
                checkflag = -1;
                PRINTF(LOG_WARNING, "%s: initial file refresh failed.\n", __func__);
                return;
            }
            else
            {
                std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::temparrivalFilePath);
                if(fileContent.empty()){
                    checkflag = -1;
                    PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
                    return;
                }
                AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
                pauseflag++;
                return;
            }
        }

        if (pauseflag <= 8)
        {
            pauseflag++;
            return;
        }

        if (RobotParams::statusStrIndex != 28)
        {
            if (pauseflag > 100)
            {
                pauseflag = 0;
                checkflag = -1;
                PRINTF(LOG_WARNING, "%s: initial failed.\n", __func__);
                return;
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
            checkflag = -1;
            PRINTF(LOG_INFO, "%s: be ready to over.\n", __func__);

            QMessageBox::information(NULL,"提示","本轮曲线运行已完成!");
        }
        break;
    case -1:
    default:
        break;
    }
}

void PedalRobotUI::InitWidgets()
{
    QPalette palette;
    palette.setColor(QPalette::Text, Qt::white);
    ui->lineEdit_powerOnOff->setPalette(palette);

    EnableButtonsForGoHome(false);
}

void PedalRobotUI::UpdateWidgets()
{
    // 油门踏板位置
    ui->lineEdit_d1->setText( QString::number(RobotParams::angleRealTime[0], 'f', 2) );
    ui->lineEdit_d2->setText( QString::number(RobotParams::angleRealTime[1], 'f', 2) );
    ui->progressBar_brake->setValue( qRound(pdRobot->GetBrakePosition()) );
    ui->progressBar_accelerator->setValue( qRound(pdRobot->GetAcceleratorPosition()) );

    // 挡位离合位置
    if (pdRobot->ifUnderControlling())
    {
        if (Configuration::GetInstance()->ifManualShift)
        {
            ui->lineEdit_shiftcurrent->setText( scui->GetCurrentShiftString() );
            ui->lineEdit_clutchcurrent->setText( scui->GetCurrentClutchString() );
        }
        else
        {
            ui->lineEdit_shiftcurrent->setText( scui->GetCurrentShiftString((int)scui->AutoShiftState::Gear_D) );
            ui->lineEdit_clutchcurrent->setText( scui->GetCurrentClutchString((int)scui->ClutchState::Released) );
        }
    }
    else
    {
        ui->lineEdit_shiftcurrent->setText( "/" );
        ui->lineEdit_clutchcurrent->setText( "/" );
    }
    scui->UpdateAnglesForShiftClutch();

    // CAN数据
    ui->lineEdit_CanBrakeOpenValue->setText(QString::number(pdRobot->GetBrakePosition(), 'f', 2));
    ui->lineEdit_CanAccOpenValue->setText(QString::number(pdRobot->GetAcceleratorPosition(), 'f', 2));
    ui->lineEdit_CanSpeed->setText(QString::number(pdRobot->GetCanCarSpeed(), 'f', 2));
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
    ui->lineEdit_UsbMP412Speed->setText( QString::number( pdRobot->GetMP412CarSpeed(), 'f', 2 ) );

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
    ui->pushButton_softStop_liftPedals->setEnabled(enable);
    ui->pushButton_startAction->setEnabled(enable);
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
    ssf << std::right << std::setw(15) << RobotParams::normalMotionAccuracy << "\n";
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
    ogf << std::right << std::setw(15) << RobotParams::normalMotionAccuracy << "\n";
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
