#include "logger.h"

#include <string.h>
#include <stdio.h>

#include "printf.h"

Logger::Logger() : _size(0)
{
}

void Logger::log(const char *buf)
{
    int len=strlen(buf);
    if( _size+len >= _capacity){/*_size+len--->maxima=capacity-1*/
        PRINTF(LOG_INFO, "%s: logger is looped.\n",__func__);
        _size=0;
    }

    memcpy(&_buf[_size],buf,len);/*_size开始 恰好覆盖_buf[_size]=0的结束符*/
    _size += len;
    _buf[_size]=0;//PRINTF()遇到0停止输出
}

char *Logger::data()
{
    return _buf;
}

void Logger::clear()
{
    _size=0;
}

void Logger::EnableAutoLogger(bool flag)
{
    autoLogger=flag;
}

void Logger::EnableCustomizedLogger(bool flag)
{
    customizedLogger=flag;
}

Logger &Logger::operator <<(const char *buf)
{
    if(autoLogger){
        log(buf);
    }
    return *this;
}

Logger &Logger::operator <<(int i)
{
    if(autoLogger){
        char buf[32];
        sprintf(buf,"%d",i);
        log(buf);
    }
    return *this;
}

Logger &Logger::operator <<(double i)
{
    if(autoLogger){
        char buf[32];
        sprintf(buf,"%f",i);
        log(buf);
    }
    return *this;
}

Logger &Logger::Customize(int i)
{
    if(customizedLogger){
        char buf[32];
        sprintf(buf,"%d",i);
        log(buf);
    }
    return *this;
}

Logger &Logger::Customize(double i, int precision)
{
    if(customizedLogger){
        char buf[32];
        sprintf(buf,"%.*lf",precision,i);//precision位小数
        log(buf);
    }
    return *this;
}

Logger &Logger::Customize(const char *buf)
{
    if(customizedLogger){
        log(buf);
    }
    return *this;
}

int Logger::GetLength()
{
    return _size;
}
