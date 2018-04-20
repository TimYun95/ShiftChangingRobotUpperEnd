#include "basetcpclient.h"

#include <QMetaEnum>

#include "common/tcppacketer.h"

BaseTcpClient::BaseTcpClient(QObject *parent)
    : QObject(parent),
      m_tcpSocket(parent),
      m_recvBuff()
{
    connect(&m_tcpSocket, SIGNAL(connected()), this, SLOT(SocketConnected()));
    connect(&m_tcpSocket, SIGNAL(disconnected()), this, SLOT(SocketDisconnected()));
    connect(&m_tcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(SocketError(QAbstractSocket::SocketError)));
    connect(&m_tcpSocket, SIGNAL(readyRead()), this, SLOT(SocketReadReady()));
}

BaseTcpClient::~BaseTcpClient()
{
    DisconnectFromServer();
}

QAbstractSocket::SocketState BaseTcpClient::GetSocketState()
{
    return m_tcpSocket.state();
}

bool BaseTcpClient::ConnectToServer(const QString &serverUrl, quint16 port)
{
    m_tcpSocket.connectToHost(serverUrl, port);
    return m_tcpSocket.waitForConnected(1*1000);
}

void BaseTcpClient::DisconnectFromServer()
{
    if(m_tcpSocket.isOpen()){
        m_tcpSocket.disconnectFromHost();
    }
}

void BaseTcpClient::SocketConnected()
{
    qDebug()<< __PRETTY_FUNCTION__;

    emit ClientConnected();
}

void BaseTcpClient::SocketDisconnected()
{
    qDebug()<< __PRETTY_FUNCTION__;

    emit ClientDisconnected();
}

void BaseTcpClient::SocketError(QAbstractSocket::SocketError error)
{
    QMetaEnum metaEnum = QMetaEnum::fromType<QAbstractSocket::SocketError>();
    const char* string = metaEnum.valueToKey(error);
    qDebug()<< __PRETTY_FUNCTION__<< string;

    emit ClientError(error);
}

void BaseTcpClient::SocketReadReady()
{
    TcpTypes::TcpPacketMsgId_t msgId;
    QByteArray msg;
    bool recv = DoReadMsg(&msgId, &msg);
    if(!recv){//未接收到一条完整的消息
        return;
    }

    OnClientRecvMsg(msgId, msg);
}

qint64 BaseTcpClient::DoWriteMsg(const TcpTypes::TcpPacketMsgId_t msgId, const QByteArray &msg)
{
    //1. TCP数据打包
    QByteArray sendBuff;
    TcpPacketer::PacketTcpPayload(msgId, msg, &sendBuff);
    if(sendBuff.isEmpty()){
        qDebug()<< __PRETTY_FUNCTION__ << "packet error!";
        return 0;
    }

    //2. Tcp发送ByteArray的原始数据
    return m_tcpSocket.write(sendBuff);
}

bool BaseTcpClient::DoReadMsg(TcpTypes::TcpPacketMsgId_t *msgId, QByteArray *msg)
{
    msg->clear();

    //1. Tcp接收ByteArray的原始数据 并 加入已接收数据末尾
    m_recvBuff.append( m_tcpSocket.readAll() );

    //2. TCP数据解包(确定是否接收到一条完整的请求消息)
    TcpPacketer::UnpacketTcpData(&m_recvBuff, msgId, msg);
    if(msg->isEmpty()){
        return false;
    }
    return true;
}
