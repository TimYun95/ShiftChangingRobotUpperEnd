#include "unifiedrobotapiclientbase.h"

#include "TcpClient/common/tcppacketer.h"

UnifiedRobotApiClientBase::UnifiedRobotApiClientBase(QObject *parent)
    : TcpClientDispatcher(parent),
      m_sendTimer(parent),
      m_sendQueueMaxSize(10),
      m_sendQueue()
{
    connect(&m_sendTimer, SIGNAL(timeout()), this, SLOT(OnSendTimerOut()));
}

UnifiedRobotApiClientBase::~UnifiedRobotApiClientBase()
{

}

void UnifiedRobotApiClientBase::StartSendTimer(int sendTimerIntervalMs)
{
    m_sendTimer.start(sendTimerIntervalMs);
}

void UnifiedRobotApiClientBase::StopSendTimer()
{
    m_sendTimer.stop();
}

void UnifiedRobotApiClientBase::SetSendQueueMaxSize(int maxSize)
{
    m_sendQueueMaxSize = maxSize;
}

int UnifiedRobotApiClientBase::GetSendQueueMaxSize()
{
    return m_sendQueueMaxSize;
}

bool UnifiedRobotApiClientBase::Send_ProtobufMsg(const TcpTypes::TcpPacketMsgId_t msgId, const google::protobuf::Message &protobufMsg)
{
    QSharedPointer<std::string> msgString =  QSharedPointer<std::string>(new std::string());
    if( !protobufMsg.SerializePartialToString(msgString.data()) ){
        qDebug()<< __func__ << "Serialize Error" << msgId;
        return false;
    }

    if(m_sendQueue.size() > GetSendQueueMaxSize()){
        qDebug()<< __func__ << "sendQueue is too long!";
        return false;
    }

    m_sendQueue.push_back( SendMsg_t(msgId, msgString) );
    return true;
}

void UnifiedRobotApiClientBase::DoSendProtobufMsg()
{
    if(m_sendQueue.isEmpty()){
        return;
    }

    SendMsg_t sendingMsg = m_sendQueue.front();
    m_sendQueue.pop_front();

    WriteMsg(sendingMsg.m_msgId, *sendingMsg.m_msgString);
}

void UnifiedRobotApiClientBase::OnSendTimerOut()
{
    DoSendProtobufMsg();
}
