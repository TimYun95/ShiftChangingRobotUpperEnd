#ifndef UNIFIEDROBOTAPICLIENT_H
#define UNIFIEDROBOTAPICLIENT_H

#include <QSemaphore>

#include "unifiedrobotapiclientbase.h"
#include "unifiedrobotapiclientdefines.h"

class UnifiedRobotApiClient : public UnifiedRobotApiClientBase
{
    Q_OBJECT
public:
    UnifiedRobotApiClient(QObject *parent);
    ~UnifiedRobotApiClient();

    void SetThreadSyncSemaphore(QSemaphore *threadSyncSemaphore);

    void SetEnableRpcMsgDebugFlag(bool enableFlag);
    bool GetEnableRpcMsgDebugFlag();

    //和URI代码中MsgStructure的MonitorCommand中相同
    enum eActionMethod{
        DeltaControlMethod = 0,
        AbsControlMethod = 3,

        MaxActionMethod
    };

public slots:
    bool StartUnifiedRobotApiClient(const QString &serverUrl, int sendTimerIntervalMs);
    void StopUnifiedRobotApiClient();

protected:
    virtual void InitClientMsgHandler();

    //Send
public slots:
    //Unified Ptc msg
    /* Send_SetMonitorActionThetaMsg()中的actionAxes目前只支持所有轴(不可以只填写某几个轴) */
    bool Send_ShowWidgetMsg(bool showFlag);
    bool Send_GoHomeMsg(bool popupConfirmBoxFlag);
    bool Send_StopSingleAxisMsg(const std::vector<int> &stopAxes);
    bool Send_MoveSingleAxisMsg(const std::vector<int> &moveAxes, const std::vector<double> &moveSpeed);
    bool Send_SwitchToActionMsg(const std::string &actionFileContent);
    bool Send_SetMonitorActionThetaMsg(const std::vector<int> &actionMethod, const std::vector<int> &actionAxes, const std::vector<double> &actionTheta);
    bool Send_SwitchToIdleStateMsg();

    //Unified Rpc msg
    bool Send_PingMsg(int32_t timestamp, const std::string &content, const std::vector<double> &array);
    bool Send_GetGoHomeResultMsg();
    bool Send_GetRobotThetaMsg();
    bool Send_GetRobotMatrixMsg();
    bool Send_GetStatusStringMsg(bool isRequestString, bool isRequestStringIndex);
    bool Send_GetPositionLimitConfMsg();

    //Receive
protected:
    //Unified Rpc msg
    virtual void Process_PingMsg(const URMSG::Rpc_PingMsg_C2S &pingMsg);
    virtual void Process_GetGoHomeResultMsg(const URMSG::Rpc_GetGoHomeResultMsg_C2S &gghrMsg);
    virtual void Process_GetRobotThetaMsg(const URMSG::Rpc_GetRobotThetaMsg_C2S &grtMsg);
    virtual void Process_GetRobotMatrixMsg(const URMSG::Rpc_GetRobotMatrixMsg_C2S &grmMsg);
    virtual void Process_GetStatusStringMsg(const URMSG::Rpc_GetStatusStringMsg_C2S &gssMsg);
    virtual void Process_GetPositionLimitConfMsg(const URMSG::Rpc_GetPositionLimitConfMsg_C2S &gplcMsg);

    //Unified Pptc msg
    virtual void Process_ReceiveEmergencyStopSignalMsg(const URMSG::Pptc_ReceiveEmergencyStopSignalMsg_S2C &ressMsg);

signals:
    void SignalProcess_PingMsg(const URMSG::Rpc_PingMsg_C2S &pingMsg);
    void SignalProcess_GetGoHomeResultMsg(const URMSG::Rpc_GetGoHomeResultMsg_C2S &gghrMsg);
    void SignalProcess_GetRobotThetaMsg(const URMSG::Rpc_GetRobotThetaMsg_C2S &grtMsg);
    void SignalProcess_GetRobotMatrixMsg(const URMSG::Rpc_GetRobotMatrixMsg_C2S &grmMsg);
    void SignalProcess_GetStatusStringMsg(const URMSG::Rpc_GetStatusStringMsg_C2S &gssMsg);
    void SignalProcess_GetPositionLimitConfMsg(const URMSG::Rpc_GetPositionLimitConfMsg_C2S &gplcMsg);

    void SignalProcess_ReceiveEmergencyStopSignalMsg(const URMSG::Pptc_ReceiveEmergencyStopSignalMsg_S2C &ressMsg);

protected:
    //Unified Rpc msg
    void On_PingMsg(const std::string &requestMsg);
    void On_GetGoHomeResultMsg(const std::string &requestMsg);
    void On_GetRobotThetaMsg(const std::string &requestMsg);
    void On_GetRobotMatrixMsg(const std::string &requestMsg);
    void On_GetStatusStringMsg(const std::string &requestMsg);
    void On_GetPositionLimitConfMsg(const std::string &requestMsg);

    //Unified Pptc msg
    void On_ReceiveEmergencyStopSignalMsg(const std::string &requestMsg);

protected:
    void ReleaseThreadSyncSemaphore();

private:
    bool m_enableRpcMsgDebugFlag;

    QSemaphore *m_threadSyncSemaphore;
};

#endif // UNIFIEDROBOTAPICLIENT_H
