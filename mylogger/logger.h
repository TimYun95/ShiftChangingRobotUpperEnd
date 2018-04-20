#ifndef LOGGER_H
#define LOGGER_H

class Logger
{
public:
    Logger();

    char* data();
    void clear();

    void EnableAutoLogger(bool flag);
    void EnableCustomizedLogger(bool flag);
    Logger& operator<<(const char*);
    Logger& operator<<(int);
    Logger& operator<<(double);

    Logger& Customize(int i);
    Logger& Customize(double i, int precision=3);
    Logger& Customize(const char* buf);
    int GetLength();

private:
    void log(const char* buf);

private:
    static const int _capacity=1024*1024*5;
    char _buf[_capacity];
    int _size;
    bool autoLogger;
    bool customizedLogger;

};

#endif // LOGGER_H
