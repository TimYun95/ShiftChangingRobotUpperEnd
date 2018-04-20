#This file is included in .pro

HEADERS += \
    $$PWD/log4qtwrapper.h

SOURCES += \
    $$PWD/log4qtwrapper.cpp

#libs
LIBS += -L$$PWD/log4qt/lib -llog4qt

DEPENDPATH += \
    $$PWD/log4qt \
    $$PWD/log4qt/helpers \
    $$PWD/log4qt/spi \
    $$PWD/log4qt/varia

INCLUDEPATH += \
    $$PWD/log4qt \
    $$PWD/log4qt/helpers \
    $$PWD/log4qt/spi \
    $$PWD/log4qt/varia

CONFIG(debug, debug|release){
    DEFINES += \
        DEBUG_OUTPUT_2_STD_COUT # in debug mode, print to std::cout
}else{
    DEFINES += \
        DEBUG_OUTPUT_2_LOCAL_FILE # in release mode, print to local file
}

DEFINES += \
    ENABLE_LOG_VERBOSE #used in Log4QTWrapper::Printf()
