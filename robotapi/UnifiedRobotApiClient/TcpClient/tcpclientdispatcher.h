#ifndef TCPCLIENTDISPATCHER_H
#define TCPCLIENTDISPATCHER_H

#include <QHash>

#include <string>
#include <tr1/functional>

#include "basetcpclient.h"

/* TCP客户端装饰器
 * 1. 装饰BaseTcpClient的读写接口
 */

class TcpClientDispatcher : public BaseTcpClient
{
    Q_OBJECT
public:
    explicit TcpClientDispatcher(QObject *parent);
    ~TcpClientDispatcher();

    qint64 WriteMsg(const TcpTypes::TcpPacketMsgId_t msgId, const std::string &msg);

    typedef std::tr1::function<void (const std::string&)> MsgHandler_t;
    void RegisterMsgHandler(const TcpTypes::TcpPacketMsgId_t msgId, MsgHandler_t msgHandler);

protected:
    void OnClientRecvMsg(const TcpTypes::TcpPacketMsgId_t msgId, const QByteArray &msg) Q_DECL_OVERRIDE;

protected:
    virtual void TcpMsgByteArray2String(const QByteArray &msgByteArray, std::string *msgString);
    virtual void TcpMsgString2ByteArray(const std::string &msgString, QByteArray *msgByteArray);

    void DoProcessTcpMsgString(const TcpTypes::TcpPacketMsgId_t msgId, const std::string &msgString);

private:
    typedef QHash<qint32, MsgHandler_t> MsgHandlerHashMap_t;
    MsgHandlerHashMap_t m_msgHandlerHashMap;
};

#endif // TCPCLIENTDISPATCHER_H
