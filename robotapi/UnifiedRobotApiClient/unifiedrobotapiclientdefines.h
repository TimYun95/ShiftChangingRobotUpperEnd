#ifndef UNIFIEDROBOTAPICLIENTDEFINES_H
#define UNIFIEDROBOTAPICLIENTDEFINES_H

#define REGISTER_MSG_HANDLER(msgId, msgHandler) \
    RegisterMsgHandler(msgId, std::tr1::bind(msgHandler, this, std::tr1::placeholders::_1))

#define PARSE_PROTOBUF_MSG(requestMsg, protobufMsg) \
    if( !protobufMsg.ParsePartialFromString(requestMsg) ){ \
        qDebug()<< __func__ << "Parse Error" << #protobufMsg; \
        return; \
    }

#endif // UNIFIEDROBOTAPICLIENTDEFINES_H
