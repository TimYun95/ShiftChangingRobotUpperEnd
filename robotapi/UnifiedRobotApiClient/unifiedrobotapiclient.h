#ifndef UNIFIEDROBOTAPICLIENT_H
#define UNIFIEDROBOTAPICLIENT_H

#include "unifiedrobotapiclientbase.h"
#include "unifiedrobotapiclientdefines.h"

class UnifiedRobotApiClient : public UnifiedRobotApiClientBase
{
    Q_OBJECT
public:
    UnifiedRobotApiClient(QObject *parent);
    ~UnifiedRobotApiClient();

    //和URI代码中MsgStructure的MonitorCommand中相同
    enum eActionMethod{
        DeltaControlMethod = 0,
        AbsControlMethod = 3,

        MaxActionMethod
    };

    bool StartUnifiedRobotApiClient(const QString &serverUrl, int sendTimerIntervalMs);
    void StopUnifiedRobotApiClient();

protected:
    virtual void InitClientMsgHandler();

    //Send
public:
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
    virtual void Process_PingMsg(URMSG::Rpc_PingMsg_C2S &pingMsg);
    virtual void Process_GetGoHomeResultMsg(URMSG::Rpc_GetGoHomeResultMsg_C2S &gghrMsg);
    virtual void Process_GetRobotThetaMsg(URMSG::Rpc_GetRobotThetaMsg_C2S &grtMsg);
    virtual void Process_GetRobotMatrixMsg(URMSG::Rpc_GetRobotMatrixMsg_C2S &grmMsg);
    virtual void Process_GetStatusStringMsg(URMSG::Rpc_GetStatusStringMsg_C2S &gssMsg);

    //Unified Pptc msg
    virtual void Process_ReceiveEmergencyStopSignalMsg(URMSG::Pptc_ReceiveEmergencyStopSignalMsg_S2C &ressMsg);

protected:
    //Unified Rpc msg
    void On_PingMsg(const std::string &requestMsg);
    void On_GetGoHomeResultMsg(const std::string &requestMsg);
    void On_GetRobotThetaMsg(const std::string &requestMsg);
    void On_GetRobotMatrixMsg(const std::string &requestMsg);
    void On_GetStatusStringMsg(const std::string &requestMsg);

    //Unified Pptc msg
    void On_ReceiveEmergencyStopSignalMsg(const std::string &requestMsg);
};

#endif // UNIFIEDROBOTAPICLIENT_H
