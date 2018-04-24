#This file is included in .pro

QT += serialbus serialport

HEADERS += \
    $$PWD/settingbase.h \
    $$PWD/settingwidgetpedalrobotgetspeed.h \
    $$PWD/settingwidgetpedalrobotdeathzone.h \
    $$PWD/settingwidgetpedalrobotstudy.h \
    $$PWD/settingwidgetpedalrobotstudywltc.h \
    $$PWD/settingwidgetscrobot.h

SOURCES += \
    $$PWD/settingbase.cpp \
    $$PWD/settingwidgetpedalrobotgetspeed.cpp \
    $$PWD/settingwidgetpedalrobotdeathzone.cpp \
    $$PWD/settingwidgetpedalrobotstudy.cpp \
    $$PWD/settingwidgetpedalrobotstudywltc.cpp \
    $$PWD/settingwidgetscrobot.cpp

FORMS += \
    $$PWD/settingwidgetpedalrobotgetspeed.ui \
    $$PWD/settingwidgetpedalrobotdeathzone.ui \
    $$PWD/settingwidgetpedalrobotstudy.ui \
    $$PWD/settingwidgetpedalrobotstudywltc.ui \
    $$PWD/settingwidgetscrobot.ui


