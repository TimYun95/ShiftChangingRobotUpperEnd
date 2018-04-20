HEADERS += \
    $$PWD/unifiedrobotapiclient.h \
    $$PWD/unifiedrobotapiclientdefines.h

SOURCES += \
    $$PWD/unifiedrobotapiclient.cpp

include(TcpClient/TcpClient.pri)
include(MessageProtocol/MessageProtocol.pri)
