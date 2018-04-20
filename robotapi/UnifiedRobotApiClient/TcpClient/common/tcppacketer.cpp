#include "tcppacketer.h"

#include <QDataStream>

/* TCP数据包的定义如下
 *    buff    =    msgLen    +    msgId    +    msg    +    下一条消息的MsgLen...
 * msgLen/msgId=大端定长(两者组成消息头)
 * msgLen标识了从msgLen头到msg尾的数据长度(字节数)
 *                |--------------------------------|
 * */

static constexpr TcpTypes::TcpPacketMsgLen_t MsgHeaderLen = sizeof(TcpTypes::TcpPacketMsgLen_t)+sizeof(TcpTypes::TcpPacketMsgId_t);

void TcpPacketer::UnpacketTcpData(QByteArray *buff, TcpTypes::TcpPacketMsgId_t *msgId, QByteArray *msg)
{
    msg->clear();

    //1. 读取msgLen和msgId组成的消息头
    const TcpTypes::TcpPacketMsgLen_t buffSize = buff->size();
    if(buffSize < MsgHeaderLen){//不足消息头的固定长度
        return;
    }
    QDataStream inStream(buff, QIODevice::ReadOnly);
    inStream.setByteOrder(QDataStream::BigEndian);
    TcpTypes::TcpPacketMsgLen_t msgLen;
    inStream >> msgLen >> *msgId;

    //2. 读取msg头开始的数据
    if(buffSize < msgLen){//数据没有全部接收完成
        return;
    }
    QByteArray leftByteArray;
    inStream >> leftByteArray;

    //3. 截取msg
    *msg = leftByteArray.left(msgLen - MsgHeaderLen);

    //4. 下一条消息的MsgLen...部分继续保存
    *buff = leftByteArray.mid( msg->size() );
}

void TcpPacketer::PacketTcpPayload(const TcpTypes::TcpPacketMsgId_t msgId, const QByteArray &msg, QByteArray *buff)
{
    buff->clear();

    //写入msgLen和msgId组成的消息头
    QDataStream outStream(buff, QIODevice::WriteOnly);
    outStream.setByteOrder(QDataStream::BigEndian);
    TcpTypes::TcpPacketMsgLen_t msgLen = MsgHeaderLen + msg.size();
    outStream << msgLen << msgId;

    //写入msg
    outStream << msg;
}
