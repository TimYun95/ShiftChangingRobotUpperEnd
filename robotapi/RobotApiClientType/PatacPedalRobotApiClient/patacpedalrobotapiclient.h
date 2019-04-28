#ifndef PATACPEDALROBOTAPICLIENT_H
#define PATACPEDALROBOTAPICLIENT_H

#include "UnifiedRobotApiClient/unifiedrobotapiclient.h"

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

        MTAppIndex,
        TransReqGrIndex,
        CmndMdIndex,
        VehStSpdIndex,
        AcclStPosIndex,
        AcclDlyTmrIndex,
        BrkPdlStPosIndex,
        BrkPdlDlyTmrIndex,

        MaxCanDataSize,
    };

    enum eEmergencyStopType{
        ManualShiftGearType,
        AutomaticShiftGearType,

        MaxEmergencyStopType
    };

protected:
    void InitClientMsgHandler() Q_DECL_OVERRIDE;

    //Send
public slots:
    //Specific Ptc msg
    bool Send_SetPedalRobotDeviceDataMsg(const std::vector<double> &canDataValues);
    bool Send_SetPedalRobotEmergencyStopThetaMsg(int emergencyStopType, const std::vector<double> &emergencyStopTheta);

    //Specific Rpc msg
    bool Send_GetPedalRobotDeviceDataMsg();

    //Receive
protected:
    virtual void Process_GetPedalRobotDeviceDataMsg(const URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S &gprddMsg);

signals:
    void SignalProcess_GetPedalRobotDeviceDataMsg(const URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S &gprddMsg);

protected:
    void On_GetPedalRobotDeviceDataMsg(const std::string &requestMsg);
};

#endif // PATACPEDALROBOTAPICLIENT_H
