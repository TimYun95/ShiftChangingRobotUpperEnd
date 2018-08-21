#include "unifiedrobotapiclient.h"

UnifiedRobotApiClient::UnifiedRobotApiClient(QObject *parent)
    : UnifiedRobotApiClientBase(parent),
      m_enableRpcMsgDebugFlag(false),
      m_threadSyncSemaphore(Q_NULLPTR)
{
    qRegisterMetaType<URMSG::Rpc_PingMsg_C2S>("URMSG::Rpc_PingMsg_C2S");
    qRegisterMetaType<URMSG::Rpc_GetGoHomeResultMsg_C2S>("URMSG::Rpc_GetGoHomeResultMsg_C2S");
    qRegisterMetaType<URMSG::Rpc_GetRobotThetaMsg_C2S>("URMSG::Rpc_GetRobotThetaMsg_C2S");
    qRegisterMetaType<URMSG::Rpc_GetRobotMatrixMsg_C2S>("URMSG::Rpc_GetRobotMatrixMsg_C2S");
    qRegisterMetaType<URMSG::Rpc_GetStatusStringMsg_C2S>("URMSG::Rpc_GetStatusStringMsg_C2S");
    qRegisterMetaType<URMSG::Rpc_GetPositionLimitConfMsg_C2S>("URMSG::Rpc_GetPositionLimitConfMsg_C2S");
    qRegisterMetaType<URMSG::Pptc_ReceiveEmergencyStopSignalMsg_S2C>("URMSG::Pptc_ReceiveEmergencyStopSignalMsg_S2C");
}

UnifiedRobotApiClient::~UnifiedRobotApiClient()
{

}

void UnifiedRobotApiClient::SetThreadSyncSemaphore(QSemaphore *threadSyncSemaphore)
{
    m_threadSyncSemaphore = threadSyncSemaphore;
}

void UnifiedRobotApiClient::SetEnableRpcMsgDebugFlag(bool enableFlag)
{
    m_enableRpcMsgDebugFlag = enableFlag;
}

bool UnifiedRobotApiClient::GetEnableRpcMsgDebugFlag()
{
    return m_enableRpcMsgDebugFlag;
}

bool UnifiedRobotApiClient::StartUnifiedRobotApiClient(const QString &serverUrl, int sendTimerIntervalMs)
{
    InitClientMsgHandler();
    bool connectSuccess = ConnectToServer(serverUrl, 25000);
    if(connectSuccess){
        StartSendTimer(sendTimerIntervalMs);
    }
    return connectSuccess;
}

void UnifiedRobotApiClient::StopUnifiedRobotApiClient()
{
    StopSendTimer();
    DisconnectFromServer();
}

void UnifiedRobotApiClient::InitClientMsgHandler()
{
    REGISTER_MSG_HANDLER(URMSG::Id_Rpc_PingMsg_C2S, UnifiedRobotApiClient::On_PingMsg);
    REGISTER_MSG_HANDLER(URMSG::Id_Rpc_GetGoHomeResultMsg_C2S, UnifiedRobotApiClient::On_GetGoHomeResultMsg);
    REGISTER_MSG_HANDLER(URMSG::Id_Rpc_GetRobotThetaMsg_C2S, UnifiedRobotApiClient::On_GetRobotThetaMsg);
    REGISTER_MSG_HANDLER(URMSG::Id_Rpc_GetRobotMatrixMsg_C2S, UnifiedRobotApiClient::On_GetRobotMatrixMsg);
    REGISTER_MSG_HANDLER(URMSG::Id_Rpc_GetStatusStringMsg_C2S, UnifiedRobotApiClient::On_GetStatusStringMsg);
    REGISTER_MSG_HANDLER(URMSG::Id_Rpc_GetPositionLimitConfMsg_C2S, UnifiedRobotApiClient::On_GetPositionLimitConfMsg);

    REGISTER_MSG_HANDLER(URMSG::Id_Pptc_ReceiveEmergencyStopSignalMsg_S2C, UnifiedRobotApiClient::On_ReceiveEmergencyStopSignalMsg);
}

bool UnifiedRobotApiClient::Send_ShowWidgetMsg(bool showFlag)
{
    URMSG::Ptc_ShowWidgetMsg_C2S swMsg;
    swMsg.set_showflag(showFlag);

    return Send_ProtobufMsg(URMSG::Id_Ptc_ShowWidgetMsg_C2S, swMsg);
}

bool UnifiedRobotApiClient::Send_GoHomeMsg(bool popupConfirmBoxFlag)
{
    URMSG::Ptc_GoHomeMsg_C2S ghMsg;
    ghMsg.set_popupconfirmboxflag(popupConfirmBoxFlag);

    return Send_ProtobufMsg(URMSG::Id_Ptc_GoHomeMsg_C2S, ghMsg);
}

bool UnifiedRobotApiClient::Send_StopSingleAxisMsg(const std::vector<int> &stopAxes)
{
    URMSG::Ptc_StopSingleAxisMsg_C2S ssaMsg;
    for(const int &axis : stopAxes){
        ssaMsg.add_stopaxes(axis);
    }

    return Send_ProtobufMsg(URMSG::Id_Ptc_StopSingleAxisMsg_C2S, ssaMsg);
}

bool UnifiedRobotApiClient::Send_MoveSingleAxisMsg(const std::vector<int> &moveAxes, const std::vector<double> &moveSpeed)
{
    if(moveAxes.size() != moveSpeed.size()){
        return false;
    }

    URMSG::Ptc_MoveSingleAxisMsg_C2S msaMsg;
    for(size_t i=0; i<moveAxes.size(); ++i){
        msaMsg.add_moveaxes( moveAxes[i] );
        msaMsg.add_movespeed( moveSpeed[i] );
    }

    return Send_ProtobufMsg(URMSG::Id_Ptc_MoveSingleAxisMsg_C2S, msaMsg);
}

bool UnifiedRobotApiClient::Send_SwitchToActionMsg(const std::__cxx11::string &actionFileContent)
{
    URMSG::Ptc_SwitchToActionMsg_C2S staMsg;
    staMsg.set_actionfilecontent(actionFileContent);

    return Send_ProtobufMsg(URMSG::Id_Ptc_SwitchToActionMsg_C2S, staMsg);
}

bool UnifiedRobotApiClient::Send_SetMonitorActionThetaMsg(const std::vector<int> &actionMethod, const std::vector<int> &actionAxes, const std::vector<double> &actionTheta)
{
    const size_t methodLength = actionMethod.size();
    const size_t axesLength = actionAxes.size();
    const size_t thetaLength = actionTheta.size();
    bool sameLength = (methodLength==axesLength) && (axesLength==thetaLength);
    if(!sameLength){
        return false;
    }

    URMSG::Ptc_SetMonitorActionThetaMsg_C2S smatMsg;
    for(size_t i=0; i<actionAxes.size(); ++i){
        smatMsg.add_actionmethod( actionMethod[i] );
        smatMsg.add_actionaxes( actionAxes[i] );
        smatMsg.add_actiontheta( actionTheta[i] );
    }

    return Send_ProtobufMsg(URMSG::Id_Ptc_SetMonitorActionThetaMsg_C2S, smatMsg);
}

bool UnifiedRobotApiClient::Send_SwitchToIdleStateMsg()
{
    URMSG::Ptc_SwitchToIdleStateMsg_C2S stisMsg;
    stisMsg.set_placeholder(true);//占位填充

    return Send_ProtobufMsg(URMSG::Id_Ptc_SwitchToIdleStateMsg_C2S, stisMsg);
}

bool UnifiedRobotApiClient::Send_PingMsg(int32_t timestamp, const std::__cxx11::string &content, const std::vector<double> &array)
{
    URMSG::Rpc_PingMsg_C2S pingMsg;
    pingMsg.set_timestamp(timestamp);
    pingMsg.set_content(content);
    for(auto num : array){
        pingMsg.add_array(num);
    }

    return Send_ProtobufMsg(URMSG::Id_Rpc_PingMsg_C2S, pingMsg);
}

bool UnifiedRobotApiClient::Send_GetGoHomeResultMsg()
{
    URMSG::Rpc_GetGoHomeResultMsg_C2S ggrMsg;
    ggrMsg.set_isgohomed(false);//空白填充

    return Send_ProtobufMsg(URMSG::Id_Rpc_GetGoHomeResultMsg_C2S, ggrMsg);
}

bool UnifiedRobotApiClient::Send_GetRobotThetaMsg()
{
    URMSG::Rpc_GetRobotThetaMsg_C2S grtMsg;
    grtMsg.add_robottheta(0.0);//空白填充

    return Send_ProtobufMsg(URMSG::Id_Rpc_GetRobotThetaMsg_C2S, grtMsg);
}

bool UnifiedRobotApiClient::Send_GetRobotMatrixMsg()
{
    URMSG::Rpc_GetRobotMatrixMsg_C2S grmMsg;
    grmMsg.add_robotmatrix(0.0);//空白填充

    return Send_ProtobufMsg(URMSG::Id_Rpc_GetRobotMatrixMsg_C2S, grmMsg);
}

bool UnifiedRobotApiClient::Send_GetStatusStringMsg(bool isRequestString, bool isRequestStringIndex)
{
    URMSG::Rpc_GetStatusStringMsg_C2S gssMsg;
    gssMsg.set_isrequeststring(isRequestString);
    gssMsg.set_isrequeststringindex(isRequestStringIndex);

    return Send_ProtobufMsg(URMSG::Id_Rpc_GetStatusStringMsg_C2S, gssMsg);
}

bool UnifiedRobotApiClient::Send_GetPositionLimitConfMsg()
{
    URMSG::Rpc_GetPositionLimitConfMsg_C2S gplcMsg;
    gplcMsg.add_positionlimitnegative(0.0);//空白填充

    return Send_ProtobufMsg(URMSG::Id_Rpc_GetPositionLimitConfMsg_C2S, gplcMsg);
}

void UnifiedRobotApiClient::Process_PingMsg(const URMSG::Rpc_PingMsg_C2S &pingMsg)
{
    if(!GetEnableRpcMsgDebugFlag()){
        return;
    }

    qDebug()<< __func__ << " timestamp=" << pingMsg.timestamp();
    qDebug()<< __func__ << " content=" << pingMsg.content().c_str();
    for(int i=0; i<pingMsg.array_size(); ++i){
        qDebug()<< __func__ << " array=" << pingMsg.array(i);
    }
}

void UnifiedRobotApiClient::Process_GetGoHomeResultMsg(const URMSG::Rpc_GetGoHomeResultMsg_C2S &gghrMsg)
{
    if(!GetEnableRpcMsgDebugFlag()){
        return;
    }

    qDebug()<< __func__ << gghrMsg.isgohomed();
}

void UnifiedRobotApiClient::Process_GetRobotThetaMsg(const URMSG::Rpc_GetRobotThetaMsg_C2S &grtMsg)
{
    if(!GetEnableRpcMsgDebugFlag()){
        return;
    }

    for(int i=0; i<grtMsg.robottheta_size(); ++i){
        qDebug()<< __func__ << grtMsg.robottheta(i);
    }
}

void UnifiedRobotApiClient::Process_GetRobotMatrixMsg(const URMSG::Rpc_GetRobotMatrixMsg_C2S &grmMsg)
{
    if(!GetEnableRpcMsgDebugFlag()){
        return;
    }

    for(int i=0; i<grmMsg.robotmatrix_size(); ++i){
        qDebug()<< __func__ << grmMsg.robotmatrix(i);
    }
}

void UnifiedRobotApiClient::Process_GetStatusStringMsg(const URMSG::Rpc_GetStatusStringMsg_C2S &gssMsg)
{
    if(!GetEnableRpcMsgDebugFlag()){
        return;
    }

    if(gssMsg.has_isrequeststring() && gssMsg.isrequeststring()){
        qDebug()<< __func__ << gssMsg.statusstring().c_str();
    }
    if(gssMsg.has_isrequeststringindex() && gssMsg.isrequeststringindex()){
        qDebug()<< __func__ << gssMsg.statusstringindex();
    }
}

void UnifiedRobotApiClient::Process_GetPositionLimitConfMsg(const URMSG::Rpc_GetPositionLimitConfMsg_C2S &gplcMsg)
{
    if(!GetEnableRpcMsgDebugFlag()){
        return;
    }

    for(int i=0; i<gplcMsg.positionlimitpositive_size(); ++i){
        qDebug()<< __func__ << "positionLimitPositive=" << gplcMsg.positionlimitpositive(i);
    }
    for(int i=0; i<gplcMsg.positionlimitnegative_size(); ++i){
        qDebug()<< __func__ << "positionLimitNegative" << gplcMsg.positionlimitnegative(i);
    }
}

void UnifiedRobotApiClient::Process_ReceiveEmergencyStopSignalMsg(const URMSG::Pptc_ReceiveEmergencyStopSignalMsg_S2C &ressMsg)
{
    if(!GetEnableRpcMsgDebugFlag()){
        return;
    }

    if(ressMsg.has_isstopsuccess()){
        qDebug()<< __func__ << "isStopSuccess=" << ressMsg.isstopsuccess();
    }else{
        qDebug()<< __func__ << "no isStopSuccess";
    }
}

void UnifiedRobotApiClient::On_PingMsg(const std::__cxx11::string &requestMsg)
{
    URMSG::Rpc_PingMsg_C2S pingMsg;
    PARSE_PROTOBUF_MSG(requestMsg, pingMsg);

    Process_PingMsg(pingMsg);
    emit SignalProcess_PingMsg(pingMsg);
    ReleaseThreadSyncSemaphore();
}

void UnifiedRobotApiClient::On_GetGoHomeResultMsg(const std::__cxx11::string &requestMsg)
{
    URMSG::Rpc_GetGoHomeResultMsg_C2S gghrMsg;
    PARSE_PROTOBUF_MSG(requestMsg, gghrMsg);

    Process_GetGoHomeResultMsg(gghrMsg);
    emit SignalProcess_GetGoHomeResultMsg(gghrMsg);
    ReleaseThreadSyncSemaphore();
}

void UnifiedRobotApiClient::On_GetRobotThetaMsg(const std::__cxx11::string &requestMsg)
{
    URMSG::Rpc_GetRobotThetaMsg_C2S grtMsg;
    PARSE_PROTOBUF_MSG(requestMsg, grtMsg);

    Process_GetRobotThetaMsg(grtMsg);
    emit SignalProcess_GetRobotThetaMsg(grtMsg);
    ReleaseThreadSyncSemaphore();
}

void UnifiedRobotApiClient::On_GetRobotMatrixMsg(const std::__cxx11::string &requestMsg)
{
    URMSG::Rpc_GetRobotMatrixMsg_C2S grmMsg;
    PARSE_PROTOBUF_MSG(requestMsg, grmMsg);

    Process_GetRobotMatrixMsg(grmMsg);
    emit SignalProcess_GetRobotMatrixMsg(grmMsg);
    ReleaseThreadSyncSemaphore();
}

void UnifiedRobotApiClient::On_GetStatusStringMsg(const std::__cxx11::string &requestMsg)
{
    URMSG::Rpc_GetStatusStringMsg_C2S gssMsg;
    PARSE_PROTOBUF_MSG(requestMsg, gssMsg);

    Process_GetStatusStringMsg(gssMsg);
    emit SignalProcess_GetStatusStringMsg(gssMsg);
    ReleaseThreadSyncSemaphore();
}

void UnifiedRobotApiClient::On_GetPositionLimitConfMsg(const std::__cxx11::string &requestMsg)
{
    URMSG::Rpc_GetPositionLimitConfMsg_C2S gplcMsg;
    PARSE_PROTOBUF_MSG(requestMsg, gplcMsg);

    Process_GetPositionLimitConfMsg(gplcMsg);
    emit SignalProcess_GetPositionLimitConfMsg(gplcMsg);
    ReleaseThreadSyncSemaphore();
}

void UnifiedRobotApiClient::On_ReceiveEmergencyStopSignalMsg(const std::__cxx11::string &requestMsg)
{
    URMSG::Pptc_ReceiveEmergencyStopSignalMsg_S2C ressMsg;
    PARSE_PROTOBUF_MSG(requestMsg, ressMsg);

    Process_ReceiveEmergencyStopSignalMsg(ressMsg);
    emit SignalProcess_ReceiveEmergencyStopSignalMsg(ressMsg);
    ReleaseThreadSyncSemaphore();
}

void UnifiedRobotApiClient::ReleaseThreadSyncSemaphore()
{
    if(m_threadSyncSemaphore != Q_NULLPTR){
        m_threadSyncSemaphore->release();
    }
}
