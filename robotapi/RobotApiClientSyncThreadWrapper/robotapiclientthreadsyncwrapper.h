#ifndef ROBOTAPICLIENTTHREADSYNCWRAPPER_H
#define ROBOTAPICLIENTTHREADSYNCWRAPPER_H

#include <QObject>
#include <QThread>
#include <QSemaphore>

#include "UnifiedRobotApiClient/unifiedrobotapiclient.h"
#include "RobotApiClientType/PatacPedalRobotApiClient/patacpedalrobotapiclient.h"

class RobotApiClientThreadSyncWrapper : public QObject
{
    Q_OBJECT
public:
    explicit RobotApiClientThreadSyncWrapper(QObject *parent, bool threadSyncFlag=false);
    ~RobotApiClientThreadSyncWrapper();

    void SetThreadSyncFlag(bool threadSyncFlag);
    bool GetThreadSyncFlag();

    void SetEnableMsgDebugFlag(bool enableFlag);
    bool GetEnableMsgDebugFlag();

    UnifiedRobotApiClient* GetUnifiedRobotApiClient();
    PatacPedalRobotApiClient* GetPatacPedalRobotApiClient();

    void StartSyncThreadWrapper(const QString &serverUrl, int sendTimerIntervalMs);
    void StopSyncThreadWrapper();

    //Send
public:
    //Unified Ptc msg
    void ThreadSend_ShowWidgetMsg(bool showFlag);
    void ThreadSend_GoHomeMsg(bool popupConfirmBoxFlag);
    void ThreadSend_StopSingleAxisMsg(const std::vector<int> &stopAxes);
    void ThreadSend_MoveSingleAxisMsg(const std::vector<int> &moveAxes, const std::vector<double> &moveSpeed);
    void ThreadSend_SwitchToActionMsg(const std::string &actionFileContent);
    void ThreadSend_SetMonitorActionThetaMsg(const std::vector<int> &actionMethod, const std::vector<int> &actionAxes, const std::vector<double> &actionTheta);
    void ThreadSend_SwitchToIdleStateMsg();

    //Unified Rpc msg
    void ThreadSend_PingMsg(int32_t timestamp, const std::string &content, const std::vector<double> &array);
    void ThreadSend_GetGoHomeResultMsg();
    void ThreadSend_GetRobotThetaMsg();
    void ThreadSend_GetRobotMatrixMsg();
    void ThreadSend_GetStatusStringMsg(bool isRequestString, bool isRequestStringIndex);
    void ThreadSend_GetPositionLimitConfMsg();

    //Specific Ptc msg
    void ThreadSend_SetPedalRobotDeviceDataMsg(const std::vector<double> &canDataValues);
    void ThreadSend_SetPedalRobotEmergencyStopThetaMsg(int emergencyStopType, const std::vector<double> &emergencyStopTheta);

    //Specific Rpc msg
    void ThreadSend_GetPedalRobotDeviceDataMsg();

signals:
    void SignalStartUnifiedRobotApiClient(const QString &serverUrl, int sendTimerIntervalMs);
    void SignalStopUnifiedRobotApiClient();

    void SignalSend_ShowWidgetMsg(bool showFlag);
    void SignalSend_GoHomeMsg(bool popupConfirmBoxFlag);
    void SignalSend_StopSingleAxisMsg(const std::vector<int> &stopAxes);
    void SignalSend_MoveSingleAxisMsg(const std::vector<int> &moveAxes, const std::vector<double> &moveSpeed);
    void SignalSend_SwitchToActionMsg(const std::string &actionFileContent);
    void SignalSend_SetMonitorActionThetaMsg(const std::vector<int> &actionMethod, const std::vector<int> &actionAxes, const std::vector<double> &actionTheta);
    void SignalSend_SwitchToIdleStateMsg();

    void SignalSend_PingMsg(int32_t timestamp, const std::string &content, const std::vector<double> &array);
    void SignalSend_GetGoHomeResultMsg();
    void SignalSend_GetRobotThetaMsg();
    void SignalSend_GetRobotMatrixMsg();
    void SignalSend_GetStatusStringMsg(bool isRequestString, bool isRequestStringIndex);
    void SignalSend_GetPositionLimitConfMsg();

    void SignalSend_SetPedalRobotDeviceDataMsg(const std::vector<double> &canDataValues);
    void SignalSend_SetPedalRobotEmergencyStopThetaMsg(int emergencyStopType, const std::vector<double> &emergencyStopTheta);

    void SignalSend_GetPedalRobotDeviceDataMsg();

private:
    void InitApiClientSendSignalAndSlots();

    void AcquireThreadSyncSemaphore(int num=1);

private:
    QObject *m_robotApiClientParent;
    PatacPedalRobotApiClient *m_robotApiClient;

    QThread *m_workingThread;

    bool m_threadSyncFlag;
    QSemaphore m_threadSyncSemaphore;
};

#endif // ROBOTAPICLIENTTHREADSYNCWRAPPER_H
