#ifndef AUTODRIVEROBOTAPICLIENT_H
#define AUTODRIVEROBOTAPICLIENT_H

#include "robotapi/RobotApiClientType/PatacPedalRobotApiClient/patacpedalrobotapiclient.h"

class AutoDriveRobotApiClient : public PatacPedalRobotApiClient
{
    Q_OBJECT
public:
    static AutoDriveRobotApiClient *GetInstance();

private:
    explicit AutoDriveRobotApiClient(QObject *parent = 0);

public:
    ~AutoDriveRobotApiClient();

protected:
    void Process_PingMsg(URMSG::Rpc_PingMsg_C2S &pingMsg) Q_DECL_OVERRIDE;
    void Process_GetGoHomeResultMsg(URMSG::Rpc_GetGoHomeResultMsg_C2S &gghrMsg) Q_DECL_OVERRIDE;
    void Process_GetRobotThetaMsg(URMSG::Rpc_GetRobotThetaMsg_C2S &grtMsg) Q_DECL_OVERRIDE;
    void Process_GetRobotMatrixMsg(URMSG::Rpc_GetRobotMatrixMsg_C2S &grmMsg) Q_DECL_OVERRIDE;
    void Process_GetStatusStringMsg(URMSG::Rpc_GetStatusStringMsg_C2S &gssMsg) Q_DECL_OVERRIDE;
    void Process_GetPedalRobotDeviceDataMsg(URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S &gprddMsg) Q_DECL_OVERRIDE;
};

#endif // AUTODRIVEROBOTAPICLIENT_H
