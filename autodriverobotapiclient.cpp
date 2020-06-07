#include "autodriverobotapiclient.h"
#include "robotparams.h"
#include "shiftclutchparams.h"
#include "printf.h"
#include "sys/time.h"

#define CONNECT_PROCESSMSG(ProcessMsgFunc)\
    connect(patacApiClient, SIGNAL(Signal##ProcessMsgFunc), this, SLOT(Slot##ProcessMsgFunc))

AutoDriveRobotApiClient *AutoDriveRobotApiClient::GetInstance()
{
    static AutoDriveRobotApiClient* instance = NULL;
    if(instance == NULL){
        instance = new AutoDriveRobotApiClient();
    }

    return instance;
}

AutoDriveRobotApiClient::AutoDriveRobotApiClient(QObject *parent) :
    QObject(parent)
{
    m_apiClientWrapper = new RobotApiClientThreadSyncWrapper(this, false);
    m_apiClientWrapper->SetEnableMsgDebugFlag(false);

    patacApiClient = m_apiClientWrapper->GetPatacPedalRobotApiClient();

    CONNECT_PROCESSMSG( Process_PingMsg(URMSG::Rpc_PingMsg_C2S) );
    CONNECT_PROCESSMSG( Process_GetGoHomeResultMsg(URMSG::Rpc_GetGoHomeResultMsg_C2S) );
    CONNECT_PROCESSMSG( Process_GetRobotThetaMsg(URMSG::Rpc_GetRobotThetaMsg_C2S) );
    CONNECT_PROCESSMSG( Process_GetRobotMatrixMsg(URMSG::Rpc_GetRobotMatrixMsg_C2S) );
    CONNECT_PROCESSMSG( Process_GetStatusStringMsg(URMSG::Rpc_GetStatusStringMsg_C2S) );
    CONNECT_PROCESSMSG( Process_GetPositionLimitConfMsg(URMSG::Rpc_GetPositionLimitConfMsg_C2S) );
    CONNECT_PROCESSMSG( Process_ReceiveEmergencyStopSignalMsg(URMSG::Pptc_ReceiveEmergencyStopSignalMsg_S2C) );
    CONNECT_PROCESSMSG( Process_GetPedalRobotDeviceDataMsg(URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S) );
    CONNECT_PROCESSMSG( Process_ReceiveUnifiedInformSignalMsg(URMSG::Pptc_ReceiveUnifiedInformSignalMsg_S2C) );
}

AutoDriveRobotApiClient::~AutoDriveRobotApiClient()
{
    m_apiClientWrapper->StopSyncThreadWrapper();
    m_apiClientWrapper->deleteLater();
}

void AutoDriveRobotApiClient::StartClient(const QString &serverUrl, int sendTimerIntervalMs)
{
    m_apiClientWrapper->StartSyncThreadWrapper(serverUrl, sendTimerIntervalMs);
}

void AutoDriveRobotApiClient::StopClient()
{
    m_apiClientWrapper->StopSyncThreadWrapper();
}

void AutoDriveRobotApiClient::Send_GoHomeMsg(bool popupConfirmBoxFlag)
{
    m_apiClientWrapper->ThreadSend_GoHomeMsg(popupConfirmBoxFlag);
}

void AutoDriveRobotApiClient::Send_StopSingleAxisMsg(const std::vector<int> &stopAxes)
{
    m_apiClientWrapper->ThreadSend_StopSingleAxisMsg(stopAxes);
}

void AutoDriveRobotApiClient::Send_MoveSingleAxisMsg(const std::vector<int> &moveAxes, const std::vector<double> &moveSpeed)
{
    m_apiClientWrapper->ThreadSend_MoveSingleAxisMsg(moveAxes, moveSpeed);

}

void AutoDriveRobotApiClient::Send_SwitchToActionMsg(const std::string &actionFileContent)
{
    m_apiClientWrapper->ThreadSend_SwitchToActionMsg(actionFileContent);
}

void AutoDriveRobotApiClient::Send_SetMonitorActionThetaMsg(const std::vector<int> &actionMethod, const std::vector<int> &actionAxes, const std::vector<double> &actionTheta, const int customVariable)
{
    m_apiClientWrapper->ThreadSend_SetMonitorActionThetaMsg(actionMethod, actionAxes, actionTheta, customVariable);
}

void AutoDriveRobotApiClient::Send_SwitchToIdleStateMsg()
{
    m_apiClientWrapper->ThreadSend_SwitchToIdleStateMsg();
}


void AutoDriveRobotApiClient::Send_SetPositionLimitConf(const std::vector<double> &positiveLimit, const std::vector<double> &negativeLimit)
{
    m_apiClientWrapper->ThreadSend_SetPositionLimitConf(positiveLimit, negativeLimit);
}

void AutoDriveRobotApiClient::Send_SetReservedParamConf(const std::vector<double> &reservedParam)
{
    m_apiClientWrapper->ThreadSend_SetReservedParamConf(reservedParam);
}

void AutoDriveRobotApiClient::Send_SaveAndSendConfMsg(bool saveFlag, bool sendFlag)
{
    m_apiClientWrapper->ThreadSend_SaveAndSendConfMsg(saveFlag, sendFlag);
}

void AutoDriveRobotApiClient::Send_MessageInformMsg(int informType, double informValue)
{
    m_apiClientWrapper->ThreadSend_MessageInformMsg(informType, informValue);
}

void AutoDriveRobotApiClient::Send_SetPedalRobotDeviceDataMsg(const std::vector<double> &canDataValues)
{
    Q_UNUSED(canDataValues);
}

void AutoDriveRobotApiClient::Send_SetPedalRobotEmergencyStopThetaMsg(int emergencyStopType, const std::vector<double> &emergencyStopTheta)
{
    m_apiClientWrapper->ThreadSend_SetPedalRobotEmergencyStopThetaMsg(emergencyStopType, emergencyStopTheta);
}

void AutoDriveRobotApiClient::Send_GetGoHomeResultMsg()
{
    m_apiClientWrapper->ThreadSend_GetGoHomeResultMsg();
}

void AutoDriveRobotApiClient::Send_GetRobotThetaMsg()
{
    m_apiClientWrapper->ThreadSend_GetRobotThetaMsg();
}

void AutoDriveRobotApiClient::Send_GetStatusStringMsg(bool isRequestString, bool isRequestStringIndex)
{
    m_apiClientWrapper->ThreadSend_GetStatusStringMsg(isRequestString, isRequestStringIndex);
}

void AutoDriveRobotApiClient::Send_GetPedalRobotDeviceDataMsg()
{
    m_apiClientWrapper->ThreadSend_GetPedalRobotDeviceDataMsg();
}

void AutoDriveRobotApiClient::SlotProcess_PingMsg(const URMSG::Rpc_PingMsg_C2S &pingMsg)
{
    Q_UNUSED(pingMsg);
}

void AutoDriveRobotApiClient::SlotProcess_GetGoHomeResultMsg(const URMSG::Rpc_GetGoHomeResultMsg_C2S &gghrMsg)
{
    RobotParams::ifGoHome = gghrMsg.isgohomed();

    // 开始询问回原的结果为0 说明之前没问过 这是第一次问 因此回答一下回原结果给询问结果
    if (RobotParams::askGoHomeatstartresult == 0)
    {
        if (RobotParams::ifGoHome)
        {
            RobotParams::askGoHomeatstartresult = 1;
        }
        else
        {
            RobotParams::askGoHomeatstartresult = 100;
        }
    }
}

void AutoDriveRobotApiClient::SlotProcess_GetRobotThetaMsg(const URMSG::Rpc_GetRobotThetaMsg_C2S &grtMsg)
{
    for(int i=0; i<grtMsg.robottheta_size(); ++i){
        RobotParams::angleRealTime[i] = grtMsg.robottheta(i);
    }

//#ifdef ENABLE_TIME_CHECK
//    int usingnum = 0;
//    int timeduring = RobotParams::testingTimes[usingnum].elapsed();

//    if (timeduring > 60 || timeduring < 50)
//    {
//        int errt;
//        if (RobotParams::testingtimenum[usingnum] == 0)
//        {
//            errt = 0;
//        }
//        else
//        {
//            errt = 54 * RobotParams::testingtimenum[usingnum];
//        }
//        PRINTF(LOG_DEBUG, "%s: get ThetaMsg abnormal, using time %d ms, error time %d.\n", __func__, timeduring, errt);
//        RobotParams::testingtimenum[usingnum] = 0;
//    }
//    else
//    {
//        RobotParams::testingtimenum[usingnum]++;
//    }

//    RobotParams::testingTimes[usingnum].restart();
//#else

//#endif
}

void AutoDriveRobotApiClient::SlotProcess_GetRobotMatrixMsg(const URMSG::Rpc_GetRobotMatrixMsg_C2S &grmMsg)
{
    Q_UNUSED(grmMsg);
}

void AutoDriveRobotApiClient::SlotProcess_GetStatusStringMsg(const URMSG::Rpc_GetStatusStringMsg_C2S &gssMsg)
{
    RobotParams::statusStr = gssMsg.statusstring();
    RobotParams::statusStrIndex = gssMsg.statusstringindex();
}

void AutoDriveRobotApiClient::SlotProcess_GetPositionLimitConfMsg(const URMSG::Rpc_GetPositionLimitConfMsg_C2S &gplcMsg)
{
    Q_UNUSED(gplcMsg);
}

void AutoDriveRobotApiClient::SlotProcess_ReceiveEmergencyStopSignalMsg(const URMSG::Pptc_ReceiveEmergencyStopSignalMsg_S2C &ressMsg)
{
    RobotParams::isEmergencyStopNow = true;
    if(ressMsg.has_isstopsuccess()){
        RobotParams::isEmergencySuccess = true;
    }else{
        RobotParams::isEmergencySuccess = false;
    }
}

void AutoDriveRobotApiClient::SlotProcess_GetPedalRobotDeviceDataMsg(const URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S &gprddMsg)
{
//        RobotParams::brakeOpenValue = gprddMsg.candatavalues(0);
//        RobotParams::accOpenValue = gprddMsg.candatavalues(1);
//        RobotParams::canCarSpeed = gprddMsg.candatavalues(2);
//        RobotParams::powerMode = gprddMsg.candatavalues(3);
//        RobotParams::pulseCarSpeed = gprddMsg.pulsedatavalue();

    Q_UNUSED(gprddMsg);
    RobotParams::accOpenValue = RobotParams::angleRealTime[1] * 0.9;
    RobotParams::brakeOpenValue = RobotParams::angleRealTime[0] * 0.9;

    if ((RobotParams::accOpenValue = RobotParams::angleRealTime[1] - 1)<0) RobotParams::accOpenValue=0;
    if ((RobotParams::brakeOpenValue = RobotParams::angleRealTime[0] - 1)<0) RobotParams::brakeOpenValue=0;

    double pos, neg;

    if ((pos = RobotParams::angleRealTime[1] - 1)<0) pos=0;
    if ((neg = RobotParams::angleRealTime[0] - 1)<0) neg=0;
    pos = pos*0.005;
    neg = neg*0.01;

    if ((RobotParams::canCarSpeed += ((pos-neg) - 0.0002*RobotParams::canCarSpeed)) < 0) RobotParams::canCarSpeed = 0;
    if (RobotParams::canCarSpeed > 140.0) RobotParams::canCarSpeed = 140.0;

    RobotParams::pulseCarSpeed = RobotParams::canCarSpeed;

    RobotParams::powerMode = 2;
}

void AutoDriveRobotApiClient::SlotProcess_ReceiveUnifiedInformSignalMsg(const URMSG::Pptc_ReceiveUnifiedInformSignalMsg_S2C &ruisMsg)
{
    ShiftClutchParams::shiftclutchState = ruisMsg.intdataarray(0);
    ShiftClutchParams::shiftState = ruisMsg.intdataarray(1);
    ShiftClutchParams::clutchState = ruisMsg.intdataarray(2);

    RobotParams::shiftclutchIndex = ruisMsg.intdataarray(0);
    RobotParams::shiftIndex = ruisMsg.intdataarray(1);
    RobotParams::clutchIndex = ruisMsg.intdataarray(2);

    ShiftClutchParams::totalTime = ruisMsg.doubledataarray(0);
    ShiftClutchParams::partialTime[0] = ruisMsg.doubledataarray(1);
    ShiftClutchParams::partialTime[1] = ruisMsg.doubledataarray(2);
    ShiftClutchParams::partialTime[2] = ruisMsg.doubledataarray(3);
    ShiftClutchParams::partialTime[3] = ruisMsg.doubledataarray(4);
    ShiftClutchParams::partialTime[4] = ruisMsg.doubledataarray(5);
}
