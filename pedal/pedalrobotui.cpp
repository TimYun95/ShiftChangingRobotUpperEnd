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

    //connect(pUri, SIGNAL(EmergencyStopSignal()), this, SLOT(EmergencyStopSlot()));
    // 急停 should be solved
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

        int usingnum;
        timeval time_last;
        double timeduring;
        double errt;

        case 1: // rem=1
            AutoDriveRobotApiClient::GetInstance()->Send_GetPedalRobotDeviceDataMsg(); // CAN/MP412信息

            usingnum = 3;
            time_last = RobotParams::testingtime[usingnum];
            gettimeofday(&RobotParams::testingtime[usingnum], NULL);
            timeduring = (RobotParams::testingtime[usingnum].tv_sec-time_last.tv_sec)*1000.0 + (RobotParams::testingtime[usingnum].tv_usec-time_last.tv_usec)/1000.0;
            int interval = time11.elapsed();
            time11.restart();
            if (timeduring > 60 || timeduring < 50)
            {

                if (RobotParams::testingtimenum[usingnum] == 0)
                {
                    errt = 0;
                }
                else
                {
                    errt = 54 * RobotParams::testingtimenum[usingnum];
                }
                std::cout<< timeduring << " " << interval <<std::endl;
                //PRINTF(LOG_ERR, "%s: send GetCANMsg abnormal, using time %f ms, error time %f.\n", __func__, timeduring, errt);
                RobotParams::testingtimenum[usingnum] = 0;
            }
            else
            {
                RobotParams::testingtimenum[usingnum]++;
            }
            break;
        case 2: // rem=2
            AutoDriveRobotApiClient::GetInstance()->Send_GetRobotThetaMsg(); // 角度信息

            usingnum = 2;
            time_last = RobotParams::testingtime[usingnum];
            gettimeofday(&RobotParams::testingtime[usingnum], NULL);
            timeduring = (RobotParams::testingtime[usingnum].tv_sec-time_last.tv_sec)*1000.0 + (RobotParams::testingtime[usingnum].tv_usec-time_last.tv_usec)/1000.0;
            if (timeduring > 60 || timeduring < 50)
            {
                if (RobotParams::testingtimenum[usingnum] == 0)
                {
                    errt = 0;
                }
                else
                {
                    errt = 54 * RobotParams::testingtimenum[usingnum];
                }
                PRINTF(LOG_ERR, "%s: send GetThetaMsg abnormal, using time %f ms, error time %f.\n", __func__, timeduring, errt);
                RobotParams::testingtimenum[usingnum] = 0;
            }
            else
            {
                RobotParams::testingtimenum[usingnum]++;
            }

            break;
        default:
            break;
        }
    }else{
        // 控制准备+时间差校验
        pdRobot->UpdatePart1();

        // rem=0 即18ms*3的控制周期 进行机器人的控制
        pdRobot->UpdatePart2();

        // NVH控制
        pdRobot->UpdatePart3();

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
                    GoHomeRound = RobotParams::waitForGoHomeRound;
                    PRINTF(LOG_INFO, "%s: go home success.\n", __func__);
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
    pdRobot->FinishQCustomPlotNVH(false);
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
    if (!pdRobot->CheckIfAtReady())
    {
        QMessageBox::information(NULL,"提示", QString("请正常测试后尝试运行曲线！"));
        PRINTF(LOG_DEBUG, "%s: not at N shift or free clutch.\n", __func__);
        return;
    }

    if ( !pdRobot->isStateIdle() )
    {
        QMessageBox::information(NULL, "警告", QString("检测到有其他模式正在运行！\n请检查是否未退出测试或者其他运行模式！"));
        PRINTF(LOG_WARNING, "%s: motion module confliction, state is\n\tswitchflag(%d,%d,%d,%d,%d)\n\tisExaming(%d)\n\tisControlling(%d).\n", __func__, RobotParams::switchflag[0], RobotParams::switchflag[1], RobotParams::switchflag[2], RobotParams::switchflag[3], RobotParams::switchflag[4], RobotParams::isExaming, pdRobot->GetIsControlling());
        return;
    }

    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

    if ( !pdRobot->SelectSpeedCurve( !ui->checkBox_repeatCurveFile->isChecked() ) )
    {
        return;
    }

    if ( !pdRobot->isStateIdle() )
    {
        QMessageBox::information(NULL, "警告", QString("检测到有其他模式正在运行！\n请检查是否未退出测试或者其他运行模式！"));
        PRINTF(LOG_WARNING, "%s: motion module confliction, state is\n\tswitchflag(%d,%d,%d,%d,%d)\n\tisExaming(%d)\n\tisControlling(%d).\n", __func__, RobotParams::switchflag[0], RobotParams::switchflag[1], RobotParams::switchflag[2], RobotParams::switchflag[3], RobotParams::switchflag[4], RobotParams::isExaming, pdRobot->GetIsControlling());
        return;
    }

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
    pdRobot->GetPIDParams(PID);
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
    QString shiftnow = QString::fromStdString(RobotParams::currentshiftvalue);
    if (shiftnow == QString("N_1&2") || shiftnow == QString("N_3&4") || shiftnow == QString("N_5&6"))
    {
        shiftnow = QString("N");
    }
    ui->lineEdit_shiftcurrent->setText( shiftnow );
    ui->lineEdit_clutchcurrent->setText( QString::fromStdString(RobotParams::currentclutchvalue) );
    scui->UpdateSC();

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

    // NVH状态
    ui->lineEdit_nvh_brakeopenvalue->setText(QString::number(pdRobot->GetBrakePosition(), 'g', 4));
    ui->lineEdit_nvh_accopenvalue->setText(QString::number(pdRobot->GetAcceleratorPosition(), 'g', 4));
    ui->lineEdit_nvh_brakevalue->setText( QString::number(RobotParams::angleRealTime[0], 'g', 4) );
    ui->lineEdit_nvh_accvalue->setText( QString::number(RobotParams::angleRealTime[1], 'g', 4) );

    // 确认回原等状态
    static bool isGoHomedLast = !RobotParams::ifGoHome; // 初始值 保证进入if判断
    static bool isConfirmSCLast = !RobotParams::ifConfirmSC;
    static bool isConfirmCSLast = !RobotParams::ifConfirmCS;
    static bool isswitchflag[5] = {!RobotParams::switchflag[0], !RobotParams::switchflag[1], !RobotParams::switchflag[2], !RobotParams::switchflag[3], !RobotParams::switchflag[4]};
    static bool isisExaming = !RobotParams::isExaming;

    // 模式不在空闲时 不能进行曲线跟踪
    if ( !pdRobot->isStateIdle() )
    {
        if(RobotParams::ifGoHome != isGoHomedLast || RobotParams::ifConfirmSC != isConfirmSCLast || RobotParams::ifConfirmCS != isConfirmCSLast ||
           RobotParams::switchflag[0] != isswitchflag[0] || RobotParams::switchflag[1] != isswitchflag[1] || RobotParams::switchflag[2] != isswitchflag[2] ||
           RobotParams::switchflag[3] != isswitchflag[3] || RobotParams::switchflag[4] != isswitchflag[4] || RobotParams::isExaming != isisExaming){ // 状态有变化
            isGoHomedLast = RobotParams::ifGoHome;
            isConfirmSCLast = RobotParams::ifConfirmSC;
            isConfirmCSLast = RobotParams::ifConfirmCS;
            isswitchflag[0] = RobotParams::switchflag[0];
            isswitchflag[1] = RobotParams::switchflag[1];
            isswitchflag[2] = RobotParams::switchflag[2];
            isswitchflag[3] = RobotParams::switchflag[3];
            isswitchflag[4] = RobotParams::switchflag[4];
            isisExaming = RobotParams::isExaming;
        }

        if (pdRobot->GetIsControlling())
        {
            EnableButtonsForGoHome(true);
        }
        else
        {
            EnableButtonsForGoHome(false);
        }
    }
    else
    {
        if(RobotParams::ifGoHome != isGoHomedLast || RobotParams::ifConfirmSC != isConfirmSCLast || RobotParams::ifConfirmCS != isConfirmCSLast ||
           RobotParams::switchflag[0] != isswitchflag[0] || RobotParams::switchflag[1] != isswitchflag[1] || RobotParams::switchflag[2] != isswitchflag[2] ||
           RobotParams::switchflag[3] != isswitchflag[3] || RobotParams::switchflag[4] != isswitchflag[4] || RobotParams::isExaming != isisExaming){ // 状态有变化
            isGoHomedLast = RobotParams::ifGoHome;
            isConfirmSCLast = RobotParams::ifConfirmSC;
            isConfirmCSLast = RobotParams::ifConfirmCS;
            isswitchflag[0] = RobotParams::switchflag[0];
            isswitchflag[1] = RobotParams::switchflag[1];
            isswitchflag[2] = RobotParams::switchflag[2];
            isswitchflag[3] = RobotParams::switchflag[3];
            isswitchflag[4] = RobotParams::switchflag[4];
            isisExaming = RobotParams::isExaming;
            EnableButtonsForGoHome(true);
        }
    }
}

void PedalRobotUI::UpdateCarTypeWidget()
{
    std::string carTypewithxml = NormalFile::GetFileName( (conf->carTypeFilePath + conf->carTypeName).c_str() );
    const std::string carType = carTypewithxml.substr(0, carTypewithxml.length() - 4);
    static std::string carTypeLast = "";
    if(carTypeLast != carType){
        carTypeLast = carType;
        ui->lineEdit_carType->setText(carType.c_str());
    }

    if (stui->haveReadXML)
    {
        scui->UpdateCar();
        stui->haveReadXML = false;
    }

    if (scui->haveReadXML)
    {
        stui->UpdateAllSetUI();
        scui->haveReadXML = false;
    }

    if (stui->haveChangeUsage)
    {
        scui->UpdateUsage();
        stui->haveChangeUsage = false;
    }
}

void PedalRobotUI::UpdateGetSpeedWidget()
{
    QString content="";
    switch (conf->getSpeedMethod){
    case 0:
    case 1:
        content = tr("USB AD/脉冲的车速");
        break;
    case 2:
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
        if ( RobotParams::ifGoHome && RobotParams::ifConfirmSC && ( !conf->ifManualShift || (conf->ifManualShift && RobotParams::ifConfirmCS) ) )
        {
            ui->pushButton_softStop_liftPedals->setEnabled(enable);
            ui->pushButton_slowlybrake->setEnabled(enable);
            ui->pushButton_startAction->setEnabled(enable);
        }
        else
        {
            ui->pushButton_softStop_liftPedals->setEnabled(false);
            ui->pushButton_slowlybrake->setEnabled(false);
            ui->pushButton_startAction->setEnabled(false);
        }
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
    pdRobot->SoftStop();

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

void PedalRobotUI::on_pushButton_slowlybrake_clicked()
{
    // 停止
    on_pushButton_softStop_clicked();

    // 上抬踏板 缓踩刹车
    std::fstream sb(Configuration::slowlyBrakeFilePath.c_str(), std::fstream::out | std::fstream::binary);
    if(sb.fail()){
        PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, Configuration::slowlyBrakeFilePath.c_str());
        return;
    }
    sb << RobotParams::robotType << "\n";
    sb << 'R' << "\n";
    sb << std::right << std::setw(15) << 0;
    sb << std::right << std::setw(15) << 0;
    sb << std::right << std::setw(15) << 2000;
    sb << std::right << std::setw(15) << 0;
    sb << std::right << std::setw(15) << 0 << "\n";
    sb << 'T' << "\n";
    sb << Configuration::GetInstance()->translateSpeed << "\n";
    sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[0];
    sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];

    if (Configuration::GetInstance()->ifManualShift)
    {
        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[0];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[3];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[4];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

        // 踩刹车
        sb << 'T' << "\n";
        sb << Configuration::GetInstance()->translateSpeed/4 << "\n";
        sb << std::right << std::setw(15) << Configuration::GetInstance()->brakeThetaAfterGoHome;
        sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[0];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[3];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[4];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }
    else
    {
        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[1];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

        // 踩刹车
        sb << 'T' << "\n";
        sb << Configuration::GetInstance()->translateSpeed/4 << "\n";
        sb << std::right << std::setw(15) << Configuration::GetInstance()->brakeThetaAfterGoHome;
        sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[1];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }

    sb.close();

    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::slowlyBrakeFilePath);
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
}

void PedalRobotUI::on_pushButton_nvh_start_clicked()
{
    if (conf->ifManualShift)
    {
        QMessageBox::information(NULL, "提示", QString("手动档车辆不能进行NVH实验！"));
        PRINTF(LOG_DEBUG, "%s: no NVH for manul shift cars.\n", __func__);
        return;
    }

    if (conf->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::NedcControl)
    {
        QMessageBox::information(NULL, "提示", QString("请设置位WLTC控制方式！"));
        PRINTF(LOG_DEBUG, "%s: better WLTC for NVH.\n", __func__);
        return;
    }

    if (!RobotParams::ifGoHome || !RobotParams::ifConfirmSC)
    {
        QMessageBox::information(NULL, "提示", QString("请先回原并且确认挡位信息！"));
        PRINTF(LOG_DEBUG, "%s: no back to origin or confirm shift infomation for NVH.\n", __func__);
        return;
    }

    if (!pdRobot->CheckIfAtReady())
    {
        QMessageBox::information(NULL,"提示", QString("请保证车辆在空档再运行NVH实验！"));
        PRINTF(LOG_DEBUG, "%s: not at N shift for NVH.\n", __func__);
        return;
    }

    if ( !pdRobot->isStateIdle() )
    {
        QMessageBox::information(NULL, "警告", QString("检测到有其他模式正在运行！\n请检查是否未退出测试或者其他运行模式！"));
        PRINTF(LOG_WARNING, "%s: motion module confliction, state is\n\tswitchflag(%d,%d,%d,%d,%d)\n\tisExaming(%d)\n\tisControlling(%d).\n", __func__, RobotParams::switchflag[0], RobotParams::switchflag[1], RobotParams::switchflag[2], RobotParams::switchflag[3], RobotParams::switchflag[4], RobotParams::isExaming, pdRobot->GetIsControlling());
        return;
    }

    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();

    if ( !pdRobot->ReadyToNVH() )
    {
        return;
    }

    if ( !pdRobot->isStateIdle() )
    {
        QMessageBox::information(NULL, "警告", QString("检测到有其他模式正在运行！\n请检查是否未退出测试或者其他运行模式！"));
        PRINTF(LOG_WARNING, "%s: motion module confliction, state is\n\tswitchflag(%d,%d,%d,%d,%d)\n\tisExaming(%d)\n\tisControlling(%d).\n", __func__, RobotParams::switchflag[0], RobotParams::switchflag[1], RobotParams::switchflag[2], RobotParams::switchflag[3], RobotParams::switchflag[4], RobotParams::isExaming, pdRobot->GetIsControlling());
        return;
    }

    RobotParams::switchflag[2] = true;
    RobotParams::NVHcurvestate = 9;
    ui->widget_nvh->setEnabled(true);
    ui->pushButton_nvh_start->setEnabled(false);
    ui->pushButton_nvh_stop->setEnabled(true);
    ui->lineEdit_nvh_p1time->setEnabled(false);
    ui->lineEdit_nvh_p1speed->setEnabled(false);
    ui->lineEdit_nvh_p2time->setEnabled(false);
    ui->lineEdit_nvh_p2speed->setEnabled(false);
    ui->pushButton_confirmaim->setEnabled(false);
}

void PedalRobotUI::on_pushButton_nvh_stop_clicked()
{
    // 进入退出状态
    pdRobot->ifFirstToExitNVH = true;
    RobotParams::NVHcurvestate = 4;

    ui->widget_nvh->setEnabled(false);
    ui->pushButton_nvh_start->setEnabled(true);
    ui->pushButton_nvh_stop->setEnabled(false);
}

void PedalRobotUI::on_pushButton_nvh_log_clicked()
{
    QString filePath = QFileDialog::getSaveFileName(NULL, QObject::tr("日志文件保存地址:"), Configuration::logCurvePath.c_str());
    if(filePath == ""){
        return;
    }

    pdRobot->SaveLoggerFile(filePath.toStdString().c_str());
    QMessageBox::information(NULL, "提示", QObject::tr("日志文件已保存到:\n")+filePath);
}

void PedalRobotUI::on_comboBox_nvh_mode_currentIndexChanged(int index)
{
    switch (index)
    {
    case 0:
        ui->lineEdit_nvh_p1time->setText("/");
        ui->lineEdit_nvh_p1speed->setText("/");
        ui->lineEdit_nvh_p2time->setText("/");
        ui->lineEdit_nvh_p2speed->setText("/");
        ui->lineEdit_nvh_p1time->setEnabled(false);
        ui->lineEdit_nvh_p1speed->setEnabled(false);
        ui->lineEdit_nvh_p2time->setEnabled(false);
        ui->lineEdit_nvh_p2speed->setEnabled(false);
        ui->pushButton_confirmaim->setEnabled(false);
        break;
    case 1:
        RobotParams::nvh_P1t = 34.0;
        RobotParams::nvh_P1v = 140.0;
        ui->lineEdit_nvh_p1time->setText("34.0");
        ui->lineEdit_nvh_p1speed->setText("140.0");
        ui->lineEdit_nvh_p2time->setText("/");
        ui->lineEdit_nvh_p2speed->setText("/");
        ui->lineEdit_nvh_p1time->setEnabled(true);
        ui->lineEdit_nvh_p1speed->setEnabled(true);
        ui->lineEdit_nvh_p2time->setEnabled(false);
        ui->lineEdit_nvh_p2speed->setEnabled(false);
        ui->pushButton_confirmaim->setEnabled(true);
        break;
    case 2:
        ui->lineEdit_nvh_p1time->setText("/");
        ui->lineEdit_nvh_p1speed->setText("/");
        ui->lineEdit_nvh_p2time->setText("/");
        ui->lineEdit_nvh_p2speed->setText("/");
        ui->lineEdit_nvh_p1time->setEnabled(false);
        ui->lineEdit_nvh_p1speed->setEnabled(false);
        ui->lineEdit_nvh_p2time->setEnabled(false);
        ui->lineEdit_nvh_p2speed->setEnabled(false);
        ui->pushButton_confirmaim->setEnabled(false);
        break;
    case 3:
        RobotParams::nvh_P1t = 0.0;
        RobotParams::nvh_P1v = 70.0;
        RobotParams::nvh_P2t = 100.0;
        RobotParams::nvh_P2v = 140.0;
        ui->lineEdit_nvh_p1time->setText("0.0");
        ui->lineEdit_nvh_p1speed->setText("70.0");
        ui->lineEdit_nvh_p2time->setText("100.0");
        ui->lineEdit_nvh_p2speed->setText("140.0");
        ui->lineEdit_nvh_p1time->setEnabled(false);
        ui->lineEdit_nvh_p1speed->setEnabled(true);
        ui->lineEdit_nvh_p2time->setEnabled(true);
        ui->lineEdit_nvh_p2speed->setEnabled(true);
        ui->pushButton_confirmaim->setEnabled(true);
        break;
    default:
        break;
    }
}

void PedalRobotUI::on_pushButton_confirmaim_clicked()
{
    unsigned int index = ui->comboBox_nvh_mode->currentIndex();

    switch (index)
    {
    case 1:
        RobotParams::nvh_P1t = ui->lineEdit_nvh_p1time->text().toDouble();
        RobotParams::nvh_P1v = ui->lineEdit_nvh_p1speed->text().toDouble();
        break;
    case 3:
        RobotParams::nvh_P1t = ui->lineEdit_nvh_p1time->text().toDouble();
        RobotParams::nvh_P1v = ui->lineEdit_nvh_p1speed->text().toDouble();
        RobotParams::nvh_P2t = ui->lineEdit_nvh_p2time->text().toDouble();
        RobotParams::nvh_P2v = ui->lineEdit_nvh_p2speed->text().toDouble();
        break;
    default:
        break;
    }
}

void PedalRobotUI::on_pushButton_nvh_run_clicked()
{
    unsigned int index = ui->comboBox_nvh_mode->currentIndex();

    if ( !pdRobot->SelectSpeedCurveNVH( index ) )
    {
        return;
    }

    std::string fileContent = FileAssistantFunc::ReadFileContent(conf->defaultFile);
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

    PRINTF(LOG_DEBUG, "%s: starts.\n", __func__);
    pdRobot->StartQCustomPlotNVH(conf->defaultFile, index);

    // 显示NVH曲线类型
    const std::string fileName = NormalFile::GetFileName(conf->defaultFile.c_str()); // XXX_ARM
    const std::string curveType = fileName.substr(0,fileName.length()-4); // XXX
    ui->lineEdit_curveType->setText(curveType.c_str());
}

void PedalRobotUI::on_pushButton_nvh_softstop_clicked()
{
    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
    pdRobot->FinishQCustomPlotNVH(false);

    // 上抬踏板
    RefreshSoftStopFile(); // 按照车型配置文件更新softStop.txt
    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::softStopFilePath);
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
}

void PedalRobotUI::on_pushButton_nvh_slowbrake_clicked()
{
    // 停止
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
    pdRobot->FinishQCustomPlotNVH(false);

    // 上抬踏板 缓踩刹车
    std::fstream sb(Configuration::slowlyBrakeFilePath.c_str(), std::fstream::out | std::fstream::binary);
    if(sb.fail()){
        PRINTF(LOG_ERR, "%s: error open file=%s.\n", __func__, Configuration::slowlyBrakeFilePath.c_str());
        return;
    }
    sb << RobotParams::robotType << "\n";
    sb << 'R' << "\n";
    sb << std::right << std::setw(15) << 0;
    sb << std::right << std::setw(15) << 0;
    sb << std::right << std::setw(15) << 2000;
    sb << std::right << std::setw(15) << 0;
    sb << std::right << std::setw(15) << 0 << "\n";
    sb << 'T' << "\n";
    sb << Configuration::GetInstance()->translateSpeed << "\n";
    sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[0];
    sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];

    if (Configuration::GetInstance()->ifManualShift)
    {
        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[0];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[3];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[4];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

        // 踩刹车
        sb << 'T' << "\n";
        sb << Configuration::GetInstance()->translateSpeed/4 << "\n";
        sb << std::right << std::setw(15) << Configuration::GetInstance()->brakeThetaAfterGoHome;
        sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[0];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[3];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[4];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }
    else
    {
        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[1];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";

        // 踩刹车
        sb << 'T' << "\n";
        sb << Configuration::GetInstance()->translateSpeed/4 << "\n";
        sb << std::right << std::setw(15) << Configuration::GetInstance()->brakeThetaAfterGoHome;
        sb << std::right << std::setw(15) << Configuration::GetInstance()->deathPos[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->clutchAngles[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles1[1];
        sb << std::right << std::setw(15) << Configuration::GetInstance()->shiftAxisAngles2[1];
        sb << std::right << std::setw(15) << RobotParams::angleRealTime[5] << "\n";
    }

    sb.close();

    std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::slowlyBrakeFilePath);
    if(fileContent.empty()){
        PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
        return;
    }
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);
}
