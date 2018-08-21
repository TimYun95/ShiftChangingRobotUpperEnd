#include "robotapiclientthreadsyncwrapper.h"

#define WRAPPER_CONNECT_SEND(SendMsgFunc)\
    connect(this, SIGNAL(Signal##SendMsgFunc), m_robotApiClient, SLOT(SendMsgFunc), Qt::ConnectionType::QueuedConnection)

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

void RobotApiClientThreadSyncWrapper::ThreadSend_SetMonitorActionThetaMsg(const std::vector<int> &actionMethod, const std::vector<int> &actionAxes, const std::vector<double> &actionTheta)
{
    emit SignalSend_SetMonitorActionThetaMsg(actionMethod, actionAxes, actionTheta);
}

void RobotApiClientThreadSyncWrapper::ThreadSend_SwitchToIdleStateMsg()
{
    emit SignalSend_SwitchToIdleStateMsg();
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

    WRAPPER_CONNECT_SEND( StartUnifiedRobotApiClient(QString,int) );
    WRAPPER_CONNECT_SEND( StopUnifiedRobotApiClient() );

    WRAPPER_CONNECT_SEND( Send_ShowWidgetMsg(bool) );
    WRAPPER_CONNECT_SEND( Send_GoHomeMsg(bool) );
    WRAPPER_CONNECT_SEND( Send_StopSingleAxisMsg(std::vector<int>) );
    WRAPPER_CONNECT_SEND( Send_MoveSingleAxisMsg(std::vector<int>, std::vector<double>) );
    WRAPPER_CONNECT_SEND( Send_SwitchToActionMsg(std::string) );
    WRAPPER_CONNECT_SEND( Send_SetMonitorActionThetaMsg(std::vector<int>, std::vector<int>, std::vector<double>) );
    WRAPPER_CONNECT_SEND( Send_SwitchToIdleStateMsg() );

    WRAPPER_CONNECT_SEND( Send_PingMsg(int32_t,std::string,std::vector<double>) );
    WRAPPER_CONNECT_SEND( Send_GetGoHomeResultMsg() );
    WRAPPER_CONNECT_SEND( Send_GetRobotThetaMsg() );
    WRAPPER_CONNECT_SEND( Send_GetRobotMatrixMsg() );
    WRAPPER_CONNECT_SEND( Send_GetStatusStringMsg(bool, bool) );
    WRAPPER_CONNECT_SEND( Send_GetPositionLimitConfMsg() );

    WRAPPER_CONNECT_SEND( Send_SetPedalRobotDeviceDataMsg(std::vector<double>) );
    WRAPPER_CONNECT_SEND( Send_SetPedalRobotEmergencyStopThetaMsg(int, std::vector<double>) );

    WRAPPER_CONNECT_SEND( Send_GetPedalRobotDeviceDataMsg() );
}

void RobotApiClientThreadSyncWrapper::AcquireThreadSyncSemaphore(int num)
{
    if(!GetThreadSyncFlag()){
        return;
    }

    m_threadSyncSemaphore.acquire(num);
}

