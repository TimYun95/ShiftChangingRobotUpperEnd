#include "tcpclientdispatcher.h"

TcpClientDispatcher::TcpClientDispatcher(QObject *parent)
    : BaseTcpClient(parent),
      m_msgHandlerHashMap()
{

}

TcpClientDispatcher::~TcpClientDispatcher()
{

}

qint64 TcpClientDispatcher::WriteMsg(const TcpTypes::TcpPacketMsgId_t msgId, const std::__cxx11::string &msg)
{
    //1. msg格式转换
    QByteArray msgByteArray;
    TcpMsgString2ByteArray(msg, &msgByteArray);

    //2. 发送消息
    return DoWriteMsg(msgId, msgByteArray);
}

void TcpClientDispatcher::RegisterMsgHandler(const TcpTypes::TcpPacketMsgId_t msgId, TcpClientDispatcher::MsgHandler_t msgHandler)
{
    m_msgHandlerHashMap.insert(msgId, msgHandler);
}

void TcpClientDispatcher::OnClientRecvMsg(const TcpTypes::TcpPacketMsgId_t msgId, const QByteArray &msg)
{
    //1. msg格式转换
    std::string msgString;
    TcpMsgByteArray2String(msg, &msgString);

    //2. 实际处理消息的操作
    DoProcessTcpMsgString(msgId, msgString);
}

void TcpClientDispatcher::TcpMsgByteArray2String(const QByteArray &msgByteArray, std::__cxx11::string *msgString)
{
    //可以插入解密操作
    *msgString = std::string(msgByteArray.data(), msgByteArray.size());
}

void TcpClientDispatcher::TcpMsgString2ByteArray(const std::__cxx11::string &msgString, QByteArray *msgByteArray)
{
    //可以插入加密操作
    *msgByteArray = QByteArray::fromStdString(msgString);
}

void TcpClientDispatcher::DoProcessTcpMsgString(const TcpTypes::TcpPacketMsgId_t msgId, const std::__cxx11::string &msgString)
{
    //1. 根据requestMsgId 确定相应的消息处理函数msgHandler
    MsgHandlerHashMap_t::const_iterator itr = m_msgHandlerHashMap.find( msgId );
    if(itr == m_msgHandlerHashMap.end()){
        qDebug()<< "MsgIdNotRegistered" << msgId;
        return;
    }
    const MsgHandler_t &msgHandler = itr.value();

    //2. msgHandler处理消息
    msgHandler(msgString);
}
