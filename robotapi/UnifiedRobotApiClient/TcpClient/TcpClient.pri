QT += network

HEADERS += \
    $$PWD/common/tcptypes.h \
    $$PWD/common/tcppacketer.h \
    $$PWD/basetcpclient.h \
    $$PWD/tcpclientdispatcher.h

SOURCES += \
    $$PWD/common/tcppacketer.cpp \
    $$PWD/basetcpclient.cpp \
    $$PWD/tcpclientdispatcher.cpp
