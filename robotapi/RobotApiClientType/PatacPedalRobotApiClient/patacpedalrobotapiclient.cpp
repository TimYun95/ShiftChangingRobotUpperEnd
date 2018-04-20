#include "patacpedalrobotapiclient.h"

PatacPedalRobotApiClient::PatacPedalRobotApiClient(QObject *parent)
    : UnifiedRobotApiClient(parent)
{

}

PatacPedalRobotApiClient::~PatacPedalRobotApiClient()
{

}

void PatacPedalRobotApiClient::InitClientMsgHandler()
{
    UnifiedRobotApiClient::InitClientMsgHandler();
    REGISTER_MSG_HANDLER(URMSG::Id_Rpc_GetPedalRobotDeviceDataMsg_C2S, PatacPedalRobotApiClient::On_GetPedalRobotDeviceDataMsg);
}

bool PatacPedalRobotApiClient::Send_SetPedalRobotDeviceDataMsg(const std::vector<double> &canDataValues)
{
    if(canDataValues.size() != MaxCanDataSize){
        qDebug()<< __func__ << "error size";
        return false;
    }

    URMSG::Ptc_SetPedalRobotDeviceDataMsg_C2S sprddMsg;
    for(const double &value : canDataValues){
        sprddMsg.add_candatavalues(value);
    }

    //sprddMsg.set_pulsedatavalue(0.0);//no effect yet

    return Send_ProtobufMsg(URMSG::Id_Ptc_SetPedalRobotDeviceDataMsg_C2S, sprddMsg);
}

bool PatacPedalRobotApiClient::Send_GetPedalRobotDeviceDataMsg()
{
    URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S gprddMsg;
    gprddMsg.set_pulsedatavalue(0.0);//占位填充

    return Send_ProtobufMsg(URMSG::Id_Rpc_GetPedalRobotDeviceDataMsg_C2S, gprddMsg);
}

void PatacPedalRobotApiClient::Process_GetPedalRobotDeviceDataMsg(URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S &gprddMsg)
{
    for(int i=0; i<gprddMsg.candatavalues_size(); ++i){
        qDebug()<< __func__ << "value=" << gprddMsg.candatavalues(i);
    }
    if(gprddMsg.has_pulsedatavalue()){
        qDebug()<< __func__ << "pulse=" << gprddMsg.pulsedatavalue();
    }
}

void PatacPedalRobotApiClient::On_GetPedalRobotDeviceDataMsg(const std::__cxx11::string &requestMsg)
{
    URMSG::Rpc_GetPedalRobotDeviceDataMsg_C2S gprddmsg;
    PARSE_PROTOBUF_MSG(requestMsg, gprddmsg);

    Process_GetPedalRobotDeviceDataMsg(gprddmsg);
}
