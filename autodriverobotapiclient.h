#ifndef AUTODRIVEROBOTAPICLIENT_H
#define AUTODRIVEROBOTAPICLIENT_H

#include "RobotApiClientSyncThreadWrapper/robotapiclientthreadsyncwrapper.h"
#include "RobotApiClientType/PatacPedalRobotApiClient/patacpedalrobotapiclient.h"

class AutoDriveRobotApiClient : public QObject
{
    Q_OBJECT
public:
    static AutoDriveRobotApiClient *GetInstance();

private:
    explicit AutoDriveRobotApiClient(QObject *parent = 0);

public:
    ~AutoDriveRobotApiClient();

    //和URI代码中MsgStructure的MonitorCommand中相同
    enum eActionMethod{
        DeltaControlMethod = 0,
        AbsControlMethod = 3,

        MaxActionMethod
    };

    void StartClient(const QString &serverUrl, int sendTimerIntervalMs);
    void StopClient();

    //Ptc msg
    void Send_GoHomeMsg(bool popupConfirmBoxFlag);
    void Send_StopSingleAxisMsg(const std::vector<int> &stopAxes);
    void Send_MoveSingleAxisMsg(const std::vector<int> &moveAxes, const std::vector<double> &moveSpeed);
    void Send_SwitchToActionMsg(const std::string &actionFileContent);
    void Send_SetMonitorActionThetaMsg(const std::vector<int> &actionMethod, const std::vector<int> &actionAxes, const std::vector<double> &actionTheta);
    void Send_SwitchToIdleStateMsg();

    void Send_SetPedalRobotDeviceDataMsg(const std::vector<double> &canDataValues);
    void Send_SetPedalRobotEmergencyStopThetaMsg(int emergencyStopType, const std::vector<double> &emergencyStopTheta);

    //Rpc msg
    void Send_GetGoHomeResultMsg();
    void Send_GetRobotThetaMsg();
    void Send_GetStatusStringMsg(bool isRequestString, bool isRequestStringIndex);
    void Send_GetPedalRobotDeviceDataMsg();

private:
    RobotApiClientThreadSyncWrapper *m_apiClientWrapper;
    PatacPedalRobotApiClient *patacApiClient;

private slots:
    void SlotProcess_PingMsg(const URMSG::Rpc_PingMsg_C2S &pingMsg);
    void SlotProcess_GetGoHomeResultMsg(const URMSG::Rpc_GetGoHomeResultMsg_C2S &gghrMsg);
    void SlotProcess_GetRobotThetaMsg(const URMSG::Rpc_GetRobotThetaMsg_C2S &grtMsg);
    void SlotProcess_GetRobotMatrixMsg(const URMSG::Rpc_GetRobotMatrixMsg_C2S &grmMsg);
    void SlotProcess_GetStatusStringMsg(const URMSG::Rpc_GetStatusStringMsg_C2S &gssMsg);
    void SlotProcess_GetPositionLimitConfMsg(const URMSG::Rpc_GetPositionLimitConfMsg_C2S &gplcMsg);
    void SlotProcess_ReceiveEmergencyStopSignalMsg(const URMSG::Pptc_ReceiveEmergencyStopSignalMsg_S2C &ressMsg);
    void SlotProcess_GetPedalRobotDeviceDataMsg(const URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S &gprddMsg);

};

#endif // AUTODRIVEROBOTAPICLIENT_H
