#include "robotapiclientthreadsyncwrapper.h"

RobotApiClientThreadSyncWrapper::RobotApiClientThreadSyncWrapper(QObject *parent, bool threadSyncFlag)
    : QObject(parent),
      m_robotApiClientParent(Q_NULLPTR),
      m_robotApiClient(Q_NULLPTR),
      m_workingThread(Q_NULLPTR),
      m_threadSyncFlag(false),
      m_threadSyncSemaphore()
{
    m_robotApiClientParent = new QObject();
    m_robotApiClient = new PatacPedalRobotApiClient(m_robotApiClientParent);
    m_workingThread = new QThread(this);

    m_robotApiClient->SetSendQueueMaxSize(10);
    SetThreadSyncFlag(threadSyncFlag);
    InitApiClientSendSignalAndSlots();

    m_robotApiClientParent->moveToThread(m_workingThread);
    m_robotApiClient->moveToThread(m_workingThread);
}

RobotApiClientThreadSyncWrapper::~RobotApiClientThreadSyncWrapper()
{
    m_robotApiClient->deleteLater();
    m_robotApiClientParent->deleteLater();
    m_workingThread->deleteLater();
}

void RobotApiClientThreadSyncWrapper::SetThreadSyncFlag(bool threadSyncFlag)
{
    m_threadSyncFlag = threadSyncFlag;

    QSemaphore *threadSyncSemaphore = threadSyncFlag ? &m_threadSyncSemaphore : Q_NULLPTR;
    m_robotApiClient->SetThreadSyncSemaphore(threadSyncSemaphore);
}

bool RobotApiClientThreadSyncWrapper::GetThreadSyncFlag()
{
    return m_threadSyncFlag;
}

void RobotApiClientThreadSyncWrapper::SetEnableMsgDebugFlag(bool enableFlag)
{
    m_robotApiClient->SetEnableRpcMsgDebugFlag(enableFlag);
}

bool RobotApiClientThreadSyncWrapper::GetEnableMsgDebugFlag()
{
    return m_robotApiClient->GetEnableRpcMsgDebugFlag();
}

UnifiedRobotApiClient *RobotApiClientThreadSyncWrapper::GetUnifiedRobotApiClient()
{
    return m_robotApiClient;
}

PatacPedalRobotApiClient *RobotApiClientThreadSyncWrapper::GetPatacPedalRobotApiClient()
{
    return m_robotApiClient;
}

void RobotApiClientThreadSyncWrapper::StartSyncThreadWrapper(const QString &serverUrl, int sendTimerIntervalMs)
{
    m_workingThread->start();

    emit SignalStartUnifiedRobotApiClient(serverUrl, sendTimerIntervalMs);
}

void RobotApiClientThreadSyncWrapper::StopSyncThreadWrapper()
{
    emit SignalStopUnifiedRobotApiClient();
    m_workingThread->quit();
    m_workingThread->wait();
}

void RobotApiClientThreadSyncWrapper::ThreadSend_ShowWidgetMsg(bool showFlag)
{
    emit SignalSend_ShowWidgetMsg(showFlag);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_GoHomeMsg(bool popupConfirmBoxFlag)
{
    emit SignalSend_GoHomeMsg(popupConfirmBoxFlag);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_StopSingleAxisMsg(const std::vector<int> &stopAxes)
{
    emit SignalSend_StopSingleAxisMsg(stopAxes);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_MoveSingleAxisMsg(const std::vector<int> &moveAxes, const std::vector<double> &moveSpeed)
{
    emit SignalSend_MoveSingleAxisMsg(moveAxes, moveSpeed);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SwitchToActionMsg(const std::__cxx11::string &actionFileContent)
{
    emit SignalSend_SwitchToActionMsg(actionFileContent);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SetMonitorActionThetaMsg(const std::vector<int> &actionMethod, const std::vector<int> &actionAxes, const std::vector<double> &actionTheta, const int customVariable)
{
    emit SignalSend_SetMonitorActionThetaMsg(actionMethod, actionAxes, actionTheta, customVariable);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SwitchToIdleStateMsg()
{
    emit SignalSend_SwitchToIdleStateMsg();
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SetVelocityActionSpeedMsg(const std::vector<double> &actionSpeed)
{
    emit SignalSend_SetVelocityActionSpeedMsg(actionSpeed);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SetSerialPort(int serialDeviceIndex)
{
    emit SignalSend_SetSerialPort(serialDeviceIndex);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SetPositionLimitConf(const std::vector<double> &positiveLimit, const std::vector<double> &negativeLimit)
{
    emit SignalSend_SetPositionLimitConf(positiveLimit, negativeLimit);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SetReservedParamConf(const std::vector<double> &reservedParam)
{
    emit SignalSend_SetReservedParamConf(reservedParam);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SaveAndSendConfMsg(bool saveFlag, bool sendFlag)
{
    emit SignalSend_SaveAndSendConfMsg(saveFlag, sendFlag);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_MessageInformMsg(int informType, double informValue)
{
    emit SignalSend_MessageInformMsg(informType, informValue);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_PingMsg(int32_t timestamp, const std::__cxx11::string &content, const std::vector<double> &array)
{
    emit SignalSend_PingMsg(timestamp, content, array);
    AcquireThreadSyncSemaphore();
}

void RobotApiClientThreadSyncWrapper::ThreadSend_GetGoHomeResultMsg()
{
    emit SignalSend_GetGoHomeResultMsg();
    AcquireThreadSyncSemaphore();
}

void RobotApiClientThreadSyncWrapper::ThreadSend_GetRobotThetaMsg()
{
    emit SignalSend_GetRobotThetaMsg();
    AcquireThreadSyncSemaphore();
}

void RobotApiClientThreadSyncWrapper::ThreadSend_GetRobotMatrixMsg()
{
    emit SignalSend_GetRobotMatrixMsg();
    AcquireThreadSyncSemaphore();
}

void RobotApiClientThreadSyncWrapper::ThreadSend_GetStatusStringMsg(bool isRequestString, bool isRequestStringIndex)
{
    emit SignalSend_GetStatusStringMsg(isRequestString, isRequestStringIndex);
    AcquireThreadSyncSemaphore();
}

void RobotApiClientThreadSyncWrapper::ThreadSend_GetPositionLimitConfMsg()
{
    emit SignalSend_GetPositionLimitConfMsg();
    AcquireThreadSyncSemaphore();
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SetPedalRobotDeviceDataMsg(const std::vector<double> &canDataValues)
{
    emit SignalSend_SetPedalRobotDeviceDataMsg(canDataValues);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SetPedalRobotEmergencyStopThetaMsg(int emergencyStopType, const std::vector<double> &emergencyStopTheta)
{
    emit SignalSend_SetPedalRobotEmergencyStopThetaMsg(emergencyStopType, emergencyStopTheta);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_GetPedalRobotDeviceDataMsg()
{
    emit SignalSend_GetPedalRobotDeviceDataMsg();
}

void RobotApiClientThreadSyncWrapper::InitApiClientSendSignalAndSlots()
{
    qRegisterMetaType<int32_t>( "int32_t" );
    qRegisterMetaType<std::string>( "std::string" );
    qRegisterMetaType<std::vector<int>>( "std::vector<int>" );
    qRegisterMetaType<std::vector<double>>( "std::vector<double>" );

    connect(this, SIGNAL(SignalStartUnifiedRobotApiClient(QString,int)), m_robotApiClient, SLOT(StartUnifiedRobotApiClient(QString,int)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalStopUnifiedRobotApiClient()), m_robotApiClient, SLOT(StopUnifiedRobotApiClient()), Qt::ConnectionType::QueuedConnection);

    connect(this, SIGNAL(SignalSend_ShowWidgetMsg(bool)), m_robotApiClient, SLOT(Send_ShowWidgetMsg(bool)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_GoHomeMsg(bool)), m_robotApiClient, SLOT(Send_GoHomeMsg(bool)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_StopSingleAxisMsg(std::vector<int>)), m_robotApiClient, SLOT(Send_StopSingleAxisMsg(std::vector<int>)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_MoveSingleAxisMsg(std::vector<int>, std::vector<double>)), m_robotApiClient, SLOT(Send_MoveSingleAxisMsg(std::vector<int>, std::vector<double>)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_SwitchToActionMsg(std::string)), m_robotApiClient, SLOT(Send_SwitchToActionMsg(std::string)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_SetMonitorActionThetaMsg(std::vector<int>, std::vector<int>, std::vector<double>, int)), m_robotApiClient, SLOT(Send_SetMonitorActionThetaMsg(std::vector<int>, std::vector<int>, std::vector<double>, int)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_SwitchToIdleStateMsg()), m_robotApiClient, SLOT(Send_SwitchToIdleStateMsg()), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_SetVelocityActionSpeedMsg(std::vector<double>)), m_robotApiClient, SLOT(Send_SetVelocityActionSpeedMsg(std::vector<double>)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_SetSerialPort(int)), m_robotApiClient, SLOT(Send_SetSerialPort(int)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_SetPositionLimitConf(std::vector<double>,std::vector<double>)), m_robotApiClient, SLOT(Send_SetPositionLimitConf(std::vector<double>,std::vector<double>)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_SetReservedParamConf(std::vector<double>)), m_robotApiClient, SLOT(Send_SetReservedParamConf(std::vector<double>)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_SaveAndSendConfMsg(bool,bool)), m_robotApiClient, SLOT(Send_SaveAndSendConfMsg(bool,bool)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_MessageInformMsg(int,double)), m_robotApiClient, SLOT(Send_MessageInformMsg(int,double)), Qt::ConnectionType::QueuedConnection);

    connect(this, SIGNAL(SignalSend_PingMsg(int32_t,std::string,std::vector<double>)), m_robotApiClient, SLOT(Send_PingMsg(int32_t,std::string,std::vector<double>)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_GetGoHomeResultMsg()), m_robotApiClient, SLOT(Send_GetGoHomeResultMsg()), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_GetRobotThetaMsg()), m_robotApiClient, SLOT(Send_GetRobotThetaMsg()), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_GetRobotMatrixMsg()), m_robotApiClient, SLOT(Send_GetRobotMatrixMsg()), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_GetStatusStringMsg(bool, bool)), m_robotApiClient, SLOT(Send_GetStatusStringMsg(bool, bool)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_GetPositionLimitConfMsg()), m_robotApiClient, SLOT(Send_GetPositionLimitConfMsg()), Qt::ConnectionType::QueuedConnection);

    connect(this, SIGNAL(SignalSend_SetPedalRobotDeviceDataMsg(std::vector<double>)), m_robotApiClient, SLOT(Send_SetPedalRobotDeviceDataMsg(std::vector<double>)), Qt::ConnectionType::QueuedConnection);
    connect(this, SIGNAL(SignalSend_SetPedalRobotEmergencyStopThetaMsg(int, std::vector<double>)), m_robotApiClient, SLOT(Send_SetPedalRobotEmergencyStopThetaMsg(int, std::vector<double>)), Qt::ConnectionType::QueuedConnection);

    connect(this, SIGNAL(SignalSend_GetPedalRobotDeviceDataMsg()), m_robotApiClient, SLOT(Send_GetPedalRobotDeviceDataMsg()), Qt::ConnectionType::QueuedConnection);
}

void RobotApiClientThreadSyncWrapper::AcquireThreadSyncSemaphore(int num)
{
    if(!GetThreadSyncFlag()){
        return;
    }

    m_threadSyncSemaphore.acquire(num);
}

