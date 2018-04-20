#ifndef TCPPACKETER_H
#define TCPPACKETER_H

#include "tcptypes.h"
#include <QByteArray>

/* TCP打包器
 * 1. 对原始TCP数据的打解包
 */

class TcpPacketer
{
public:
    TcpPacketer() = delete;

    //根据buff中的msgLen和msgId 对数据buff进行解包
    //获得msg和msgId
    static void UnpacketTcpData(QByteArray *buff, TcpTypes::TcpPacketMsgId_t *msgId, QByteArray *msg);

    //根据msg和msgId 对数据进行打包
    //获得打包后的buff
    static void PacketTcpPayload(const TcpTypes::TcpPacketMsgId_t msgId, const QByteArray &msg, QByteArray *buff);
};

#endif // TCPPACKETER_H
