#ifndef UNIFIEDROBOTAPICLIENTBASE_H
#define UNIFIEDROBOTAPICLIENTBASE_H

#include <QTimer>
#include <QQueue>

#include "TcpClient/tcpclientdispatcher.h"
#include "MessageProtocol/message.pb.h"

//定时器 定时发送存储在发送队列中的数据 (不使用后台线程 避免线程间同步问题)
class UnifiedRobotApiClientBase : public TcpClientDispatcher
{
    Q_OBJECT

public:
    UnifiedRobotApiClientBase(QObject *parent);
    ~UnifiedRobotApiClientBase();

    void StartSendTimer(int sendTimerIntervalMs);
    void StopSendTimer();

    void SetSendQueueMaxSize(int maxSize);
    int GetSendQueueMaxSize();

protected:
    bool Send_ProtobufMsg(const TcpTypes::TcpPacketMsgId_t msgId, const google::protobuf::Message &protobufMsg);

private:
    struct SendMsg_t{
        SendMsg_t(const TcpTypes::TcpPacketMsgId_t &msgId, const QSharedPointer<std::string> &msgString) : m_msgId(msgId), m_msgString(msgString) {}

        TcpTypes::TcpPacketMsgId_t m_msgId;
        QSharedPointer<std::string> m_msgString;
    };

    void DoSendProtobufMsg();

private slots:
    void OnSendTimerOut();

private:
    QTimer m_sendTimer;

    int m_sendQueueMaxSize;
    QQueue<SendMsg_t> m_sendQueue;
};

#endif // UNIFIEDROBOTAPICLIENTBASE_H
