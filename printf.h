#ifndef PRINTF_H
#define PRINTF_H

#include <qglobal.h>
#include <stdio.h>

#include <iostream>

#include <QDebug>

/* 切换PRINTF
 *   1. 输出到到控制台
 *   2. 刷出本地日志文件(利用log4qt) */

#ifdef DEBUG_OUTPUT_2_LOCAL_FILE//输出日志到本地文件
    #include "mylogger/log4qtwrapper.h"
#elif defined DEBUG_OUTPUT_2_STD_COUT//输出日志到std::out

    #define LOG_EMERG   0 //紧急情况，需要立即通知技术人员
    #define LOG_ALERT   1 //应该被立即改正的问题，如系统数据库被破坏，ISP连接丢失
    #define LOG_CRIT    2 //重要情况，如硬盘错误，备用连接丢失
    #define LOG_ERR     3 //错误，不是非常紧急，在一定时间内修复即可
    #define LOG_WARNING 4 //警告信息，不是错误，比如系统磁盘使用了85%等
    #define LOG_NOTICE  5 //不是错误情况，也不需要立即处理
    #define LOG_INFO    6 //情报信息，正常的系统消息，比如骚扰报告，带宽数据等，不需要处理
    #define LOG_DEBUG   7 //包含详细的开发情报的信息，通常只在调试一个程序时使用
    #define LOG_VERBOSE 8 //最详细的信息，底层调试时使用

 #ifdef ENABLE_LOG_VERBOSE
     #define PRINTF(level,fmt,args...) do{Q_UNUSED(level); printf(fmt, ##args); }while(0)
 #else
     #define PRINTF(level,fmt,args...) do{if(level!=LOG_VERBOSE){ printf(fmt, ##args); }}while(0)
 #endif

#else//无日志输出
    #define PRINTF(level,fmt,args...) do{Q_UNUSED(level); Q_UNUSED(fmt); Q_UNUSED(args);}while(0)
#endif

#endif
