#-------------------------------------------------
#
# Project created by QtCreator 2018-04-11T15:20:26
#
#-------------------------------------------------

QT       += core gui xml axcontainer

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AutoDrivingRobot
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    pedal/pedalrobot.cpp \
    pedal/pedalrobotui.cpp \
    pedal/syscontrol.cpp \
    shiftclutch/shiftclutchui.cpp \
    autodriverobotapiclient.cpp \
    configuration.cpp \
    fileoperation/normalfile.cpp \
    fileoperation/excel/ExcelBase.cpp \
    fileoperation/excel/QVariantListListModel.cpp \
    mylogger/logger.cpp \
    workerthread.cpp \
    settingui.cpp \
    robotparams.cpp

HEADERS  += mainwindow.h \
    pedal/pedalrobot.h \
    pedal/pedalrobotui.h \
    pedal/syscontrol.h \
    shiftclutch/shiftclutchui.h \
    robotparams.h \
    autodriverobotapiclient.h \
    printf.h \
    configuration.h \
    fileoperation/normalfile.h \
    fileoperation/excel/ExcelBase.h \
    fileoperation/excel/QVariantListListModel.h \
    mylogger/logger.h \
    workerthread.h \
    settingui.h

FORMS    += mainwindow.ui \
    pedal/pedalrobotui.ui \
    shiftclutch/shiftclutchui.ui \
    settingui.ui

DEFINES += \
    DISABLE_STUDY_LIMIT

#mywidget
include(mywidget/MyWidget.pri)

#SettingWidget
include(settingwidget/SettingWidget.pri)

#log4qt
include(mylogger/Log4QTWrapper.pri)

#API Func#
#AssistantFunc
include(robotapi/AssistantFunc/AssistantFunc.pri)

#UnifiedRobotApiClient
include(robotapi/UnifiedRobotApiClient/UnifiedRobotApiClient.pri)

#Specific Robot
include(robotapi/RobotApiClientType/RobotApiClientTypeDefine.pri)
include(robotapi/RobotApiClientType/RobotApiClientType.pri)
include(robotapi/RobotApiClientSyncThreadWrapper/RobotApiClientSyncThreadWrapper.pri)

INCLUDEPATH += $$PWD/robotapi/

#DEFINES += ENABLE_TIME_CHECK
