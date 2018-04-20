#ifndef BASETCPCLIENT_H
#define BASETCPCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QString>
#include <QByteArray>

#include "common/tcptypes.h"

/* 基础TCP客户端
 * 1. 与TCP服务器的连接建立
 * 2. 与TCP服务器的数据读写
 * 3. TCP数据的打解包(使用TcpPacketer)
 */

class BaseTcpClient : public QObject
{
    Q_OBJECT
public:
    explicit BaseTcpClient(QObject *parent);
    ~BaseTcpClient();

    QAbstractSocket::SocketState GetSocketState();

    bool ConnectToServer(const QString &serverUrl, quint16 port);
    void DisconnectFromServer();

signals:
    void ClientConnected();
    void ClientDisconnected();
    void ClientError(QAbstractSocket::SocketError error);

protected slots:
    void SocketConnected();
    void SocketDisconnected();
    void SocketError(QAbstractSocket::SocketError error);
    void SocketReadReady();

protected:
    virtual void OnClientRecvMsg(const TcpTypes::TcpPacketMsgId_t msgId, const QByteArray &msg) = 0;

    qint64 DoWriteMsg(const TcpTypes::TcpPacketMsgId_t msgId, const QByteArray &msg);
    bool DoReadMsg(TcpTypes::TcpPacketMsgId_t *msgId, QByteArray *msg);

private:
    QTcpSocket m_tcpSocket;
    QByteArray m_recvBuff;
};

#endif // BASETCPCLIENT_H
