#include "log4qtwrapper.h"

#include <stdarg.h>

#include <iostream>

#include "configuration.h"

#include "logger.h"
#include "ttcclayout.h"
#include "logmanager.h"
#include "loggerrepository.h"
#include "fileappender.h"
#include "consoleappender.h"
#include "propertyconfigurator.h"

#include <QFile>
#include <QCoreApplication>
#include <QScopedPointer>
#include <QStringBuilder>

int Log4QTWrapper::refNum = 0;
constexpr struct Log4QTWrapper::tagLogLevel Log4QTWrapper::logLevel[8];

Log4QTWrapper::Log4QTWrapper()
{

}

Log4QTWrapper::~Log4QTWrapper()
{

}

void Log4QTWrapper::Start()
{
    if(refNum == 0){
        refNum=1;
        InitRootLogger();
        LogStartup();
        std::cout<<"log is started!"<<std::endl;
    }else{
//        std::cout<<"log has alread been started!"<<std::endl;
    }
}

void Log4QTWrapper::Stop()
{
    if(refNum != 0){
        refNum=0;
        LogShutdown();
        ShutdownRootLogger();
        std::cout<<"log is stopped!"<<std::endl;
    }
}

void Log4QTWrapper::Printf(const int level, const char *format, ...)
{
    Start();//确保log启动

    if(refNum == 0){
        Log4Qt::Logger* logger = Log4Qt::Logger::logger( logLevel[LOG_ERR].name );
        logger->error("error! Log4QT has not been started.");
        return;
    }

    /* 变参转换为字符串 */
    char buf[maxLength];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, maxLength, format, args);
    va_end(args);
    if(len < 0){
        std::cout<<"error! buf size is small!"<<std::endl;
        return;
    }

    Log4Qt::Logger* logger = Log4Qt::Logger::logger( logLevel[level].name );
    switch (logLevel[level].level){
    case LOG_EMERG:
    case LOG_ALERT:
    case LOG_CRIT:
        logger->fatal(buf);
        break;
    case LOG_ERR:
        logger->error(buf);
        break;
    case LOG_WARNING:
    case LOG_NOTICE:
        logger->warn(buf);
        break;
    case LOG_INFO:
        logger->info(buf);
        break;
    case LOG_DEBUG:
        logger->debug(buf);
        break;
    default:
        logger->fatal(QString("undefined level=") + QString::number(level));
        break;
    }
}

void Log4QTWrapper::InitRootLogger(const QString &content)
{
    /* 配置文件备份在mylogger\log4qt\lib中 */
    QString configFile = Configuration::mainFolder.c_str() + QString("/system_files/log4qt.properties");
    std::cout<<"try to use configFile of log: "<<configFile.toStdString()<<std::endl;

    if (QFile::exists(configFile)){
        Log4Qt::PropertyConfigurator::configureAndWatch(configFile);
    }else{
        std::cout<<"error! configFile does not exit!"<<std::endl;
    }

    if (!content.isEmpty()){
        Log4Qt::Logger::rootLogger()->debug(content);
    }
}

void Log4QTWrapper::LogStartup()
{
    Log4Qt::Logger* logger = Log4Qt::Logger::rootLogger();
    logger->debug("################################################################\n");
    logger->debug("#                          START                               #\n");
    logger->debug("################################################################\n");
}

void Log4QTWrapper::LogShutdown()
{
    Log4Qt::Logger* logger = Log4Qt::Logger::rootLogger();
    logger->debug("################################################################\n");
    logger->debug("#                          STOP                                #\n");
    logger->debug("################################################################\n");
}

void Log4QTWrapper::ShutdownRootLogger(const QString &content)
{
    Log4Qt::Logger* logger = Log4Qt::Logger::rootLogger();
    if (!content.isEmpty()){
        logger->debug(content);
    }

    logger->removeAllAppenders();
    logger->loggerRepository()->shutdown();
}
