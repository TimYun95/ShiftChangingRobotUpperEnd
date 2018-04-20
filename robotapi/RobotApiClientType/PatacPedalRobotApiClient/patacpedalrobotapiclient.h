#ifndef PATACPEDALROBOTAPICLIENT_H
#define PATACPEDALROBOTAPICLIENT_H

#include "robotapi/UnifiedRobotApiClient/unifiedrobotapiclient.h"

class PatacPedalRobotApiClient : public UnifiedRobotApiClient
{
    Q_OBJECT
public:
    PatacPedalRobotApiClient(QObject *parent);
    ~PatacPedalRobotApiClient();

    enum eRobotAxisNum{
        axisNum = 6
    };

    enum eDeivceDataIndex{
        BrakeIndex,
        AccIndex,
        CarSpeedIndex,
        PowerModeIndex,

        MaxCanDataSize,
    };

protected:
    void InitClientMsgHandler() Q_DECL_OVERRIDE;

    //Send
public:
    //Specific Ptc msg
    bool Send_SetPedalRobotDeviceDataMsg(const std::vector<double> &canDataValues);

    //Specific Rpc msg
    bool Send_GetPedalRobotDeviceDataMsg();

    //Receive
protected:
    virtual void Process_GetPedalRobotDeviceDataMsg(URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S &gprddMsg);

protected:
    void On_GetPedalRobotDeviceDataMsg(const std::string &requestMsg);
};

#endif // PATACPEDALROBOTAPICLIENT_H
