#include "autodriverobotapiclient.h"
#include "robotparams.h"
#include "printf.h"
#include "sys/time.h"

AutoDriveRobotApiClient *AutoDriveRobotApiClient::GetInstance()
{
    static AutoDriveRobotApiClient* instance = NULL;
    if(instance == NULL){
        instance = new AutoDriveRobotApiClient();
    }

    return instance;
}

AutoDriveRobotApiClient::AutoDriveRobotApiClient(QObject *parent)
    : PatacPedalRobotApiClient(parent)
{

}

AutoDriveRobotApiClient::~AutoDriveRobotApiClient()
{

}

void AutoDriveRobotApiClient::Process_PingMsg(URMSG::Rpc_PingMsg_C2S &pingMsg)
{

}

void AutoDriveRobotApiClient::Process_GetGoHomeResultMsg(URMSG::Rpc_GetGoHomeResultMsg_C2S &gghrMsg)
{
    RobotParams::ifGoHome = gghrMsg.isgohomed();
}

void AutoDriveRobotApiClient::Process_GetRobotThetaMsg(URMSG::Rpc_GetRobotThetaMsg_C2S &grtMsg)
{
    for(int i=0; i<grtMsg.robottheta_size(); ++i){
        RobotParams::angleRealTime[i] = grtMsg.robottheta(i);
    }
}

void AutoDriveRobotApiClient::Process_GetRobotMatrixMsg(URMSG::Rpc_GetRobotMatrixMsg_C2S &grmMsg)
{

}

void AutoDriveRobotApiClient::Process_GetStatusStringMsg(URMSG::Rpc_GetStatusStringMsg_C2S &gssMsg)
{
    RobotParams::statusStr = gssMsg.statusstring();
    RobotParams::statusStrIndex = gssMsg.statusstringindex();
}

void AutoDriveRobotApiClient::Process_GetPedalRobotDeviceDataMsg(URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S &gprddMsg)
{
    RobotParams::brakeOpenValue = gprddMsg.candatavalues(0);
    RobotParams::accOpenValue = gprddMsg.candatavalues(1);
    RobotParams::canCarSpeed = gprddMsg.candatavalues(2);
    RobotParams::powerMode = gprddMsg.candatavalues(3);
    RobotParams::pulseCarSpeed = gprddMsg.pulsedatavalue();
}
