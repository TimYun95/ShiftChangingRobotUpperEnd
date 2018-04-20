#ifndef LOG4QTWRAPPER_H
#define LOG4QTWRAPPER_H

#define LOG_EMERG   0 //紧急情况，需要立即通知技术人员
#define LOG_ALERT   1 //应该被立即改正的问题，如系统数据库被破坏，ISP连接丢失
#define LOG_CRIT    2 //重要情况，如硬盘错误，备用连接丢失
#define LOG_ERR     3 //错误，不是非常紧急，在一定时间内修复即可
#define LOG_WARNING 4 //警告信息，不是错误，比如系统磁盘使用了85%等
#define LOG_NOTICE  5 //不是错误情况，也不需要立即处理
#define LOG_INFO    6 //情报信息，正常的系统消息，比如骚扰报告，带宽数据等，不需要处理
#define LOG_DEBUG   7 //包含详细的开发情报的信息，通常只在调试一个程序时使用
#define LOG_VERBOSE 7

#define PRINTF(level, format, args...) Log4QTWrapper::Printf(level, format, ##args)

#include <QString>

class Log4QTWrapper
{
public:
    static void Start();//确保在第一次Printf之前调用
    static void Stop();//确保在最后一次Printf之后调用
    static void Printf(const int level, const char* format, ...);//use it just like printf in stdio.h

private:
    /* start */
    static void InitRootLogger(const QString &content="Root logger is setup.\n"); //startup
    static void LogStartup();

    /* stop */
    static void LogShutdown();
    static void ShutdownRootLogger(const QString &content="Root logger was shutdown.\n");

    /* 该类只有只有静态成员函数 拒绝构造/析构对象 */
private:
    Log4QTWrapper();
     ~Log4QTWrapper();

private:
    static int refNum;//单例模式

    static const int maxLength=128;
    struct tagLogLevel{
        int level;
        const char* name;
    };
    static constexpr struct tagLogLevel logLevel[8]=
    {
        /* 和define的顺序相同 */
        {LOG_EMERG,   "LOG_EMERG"},
        {LOG_ALERT,   "LOG_ALERT"},
        {LOG_CRIT,    "LOG_CRIT"},
        {LOG_ERR,     "LOG_ERR"},
        {LOG_WARNING, "LOG_WARNING"},
        {LOG_NOTICE,  "LOG_NOTICE"},
        {LOG_INFO,    "LOG_INFO"},
        {LOG_DEBUG,   "LOG_DEBUG"}
    };
};

#endif // LOG4QTWRAPPER_H
