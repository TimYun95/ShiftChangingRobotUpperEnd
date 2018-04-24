HEADERS += \
    $$PWD/unifiedrobotapiclient.h \
    $$PWD/unifiedrobotapiclientdefines.h \
    $$PWD/unifiedrobotapiclientbase.h

SOURCES += \
    $$PWD/unifiedrobotapiclient.cpp \
    $$PWD/unifiedrobotapiclientbase.cpp

include(TcpClient/TcpClient.pri)
include(MessageProtocol/MessageProtocol.pri)
