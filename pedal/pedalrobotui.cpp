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

const double speedCurveSpeedErrorLimit = 2.0; // NEDC/WLTC的曲线的速度误差限制

PedalRobotUI::PedalRobotUI(QWidget *parent, QLabel *_status, QLabel *_time) :
    QWidget(parent),
    ui(new Ui::PedalRobotUI),
    statusL(_status),
    timeL(_time),
    m_timerMutex(),
    m_enableMutex(false),
    ifSendGoHome(false)
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
    LockMutex();

    // 软件倍频
    static int cnt = 0;
    const int rem = ++cnt%RobotParams::UITimerMultiplier;// ++cnt%3 rem=0/1/2

    // 从服务器拿数据
    AutoDriveRobotApiClient::GetInstance()->Send_GetGoHomeResultMsg(); // 回原信息
    AutoDriveRobotApiClient::GetInstance()->Send_GetRobotThetaMsg(); // 角度信息
    AutoDriveRobotApiClient::GetInstance()->Send_GetStatusStringMsg(true, true); // 状态信息
    AutoDriveRobotApiClient::GetInstance()->Send_GetPedalRobotDeviceDataMsg(); // CAN/MP412信息

    if(rem != 0){
        switch(rem){
        case 1: // rem=1
            // 暂时不用
            break;
        case 2: // rem=2
            pdRobot->UpdatePart1(); // 只校验时间差
            break;
        default:
            break;
        }
    }else{
        // rem=0 即18ms*3的控制周期 进行机器人的控制
        pdRobot->UpdatePart2();

        // 降低刷新次数 刷新界面
        static int cntUI = 0;
        if(++cntUI%RobotParams::updateUIFrequency == 0){
            UpdateWidgets();
        }
    }

    // 判断是否发送了回原指令并且回原了
    if (ifSendGoHome && RobotParams::ifGoHome)
    {
        RefreshOriginFile(); // 按照车型配置文件更新origin.txt
        std::string fileContent = FileAssistantFunc::ReadFileContent(Configuration::originFilePath);
        if(fileContent.empty()){
            PRINTF(LOG_WARNING, "%s: read file error.\n", __func__);
            return;
        }
        AutoDriveRobotApiClient::GetInstance()->Send_SwitchToActionMsg(fileContent);

        ifSendGoHome = false;
    }

    UnlockMutex();
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

void PedalRobotUI::EmergencyStopSlot()
{
    EnableButtonsForEmergencyStop(false);

    pdRobot->SoftStop();
}

void PedalRobotUI::on_pushButton_origin_clicked()
{
    pdRobot->SoftStop();

    ifSendGoHome = true;
    AutoDriveRobotApiClient::GetInstance()->Send_GoHomeMsg(false);
}

void PedalRobotUI::on_pushButton_softStop_clicked()
{
    AutoDriveRobotApiClient::GetInstance()->Send_SwitchToIdleStateMsg();
    pdRobot->SoftStop();
    pdRobot->FinishQCustomPlot(false);
}

void PedalRobotUI::on_pushButton_confirmEmergencyStop_clicked()
{
    EnableButtonsForEmergencyStop(true);
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

    pdRobot->SelectSpeedCurve( !ui->checkBox_repeatCurveFile->isChecked() );

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

    EnableButtonsForGoHome(false);
    ui->pushButton_confirmEmergencyStop->setEnabled(false);
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
    ui->progressBar_brake->setValue( pdRobot->GetBrakePosition() );
    ui->progressBar_accelerator->setValue( pdRobot->GetAcceleratorPosition() );

    // 挡位离合位置
    ui->lineEdit_shiftcurrent->setText( QString::fromStdString(RobotParams::currentshiftvalue) );
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

    // 回原状态
    static bool isGoHomedLast = !RobotParams::ifGoHome; // 初始值 保证进入if判断
    if(RobotParams::ifGoHome != isGoHomedLast){ // 回原的状态有变化
        isGoHomedLast = RobotParams::ifGoHome;
        EnableButtonsForGoHome(isGoHomedLast);
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
    //enable=true 回原完成
    ui->pushButton_softStop_liftPedals->setEnabled(enable);
    ui->pushButton_startAction->setEnabled(enable);
}

void PedalRobotUI::EnableButtonsForEmergencyStop(bool enable)
{
    ui->pushButton_origin->setEnabled(enable);
    ui->pushButton_softStop->setEnabled(enable);
    ui->pushButton_confirmEmergencyStop->setEnabled( !enable );//单独使能

    ui->pushButton_softStop_liftPedals->setEnabled(enable);
    ui->pushButton_startAction->setEnabled(enable);
    ui->pushButton_minus1->setEnabled(enable);
    ui->pushButton_plus1->setEnabled(enable);
    ui->pushButton_minus2->setEnabled(enable);
    ui->pushButton_plus2->setEnabled(enable);
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
    ssf << std::right << std::setw(15) << 0;
    ssf << std::right << std::setw(15) << 0;
    ssf << std::right << std::setw(15) << 0;
    ssf << std::right << std::setw(15) << 0 << "\n";
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
    ogf << std::right << std::setw(15) << 0;
    ogf << std::right << std::setw(15) << 0;
    ogf << std::right << std::setw(15) << 0;
    ogf << std::right << std::setw(15) << 0 << "\n";
    ogf.close();
}

void PedalRobotUI::on_pushButton_saveLoggerFile_clicked()
{
    QString filePath = QFileDialog::getSaveFileName(NULL, QObject::tr("日志文件保存地址:"), Configuration::logFilePath.c_str());
    if(filePath == ""){
        return;
    }

    pdRobot->SaveLoggerFile(filePath.toStdString().c_str());
    QMessageBox::information(NULL, "inform", QObject::tr("日志文件已保存到:\n")+filePath);
}
