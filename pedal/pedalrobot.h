#ifndef PEDAL_H
#define PEDAL_H
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
/* 针对2DOF油门刹车踏板机器人 */

#include <time.h>

#include <string>
#include <vector>
#include <utility>

#include <QString>
#include <QObject>

#include "mylogger/logger.h"
#include "configuration.h"
#include "workerthread.h"
#include "mywidget/qcustomplot.h"
#include "syscontrol.h"

class PedalRobot: public QObject
{
    Q_OBJECT
public:
    explicit PedalRobot(QCustomPlot *_widget, Configuration *_conf);
    ~PedalRobot();

    void SoftStop(); // 退出曲线跟踪 保存日志

    void SelectSpeedCurve(const bool selectFile); // 选择跟踪的曲线
    void StartQCustomPlot(const std::string& fileNameARM); // 开始绘制曲线
    void UpdateQCustomPlot(); // 更新绘制曲线
    void FinishQCustomPlot(bool showMessageBox=true); // 完成更新曲线

    void UpdatePart1(); // 定时更新 只校验时间差
    void UpdatePart2(); // 定时更新 执行控制功能
    double GetBrakePosition(); // 获取刹车踏板开度
    double GetAcceleratorPosition(); // 获取油门踏板开度
    double GetCanCarSpeed(); // 获取CAN口的车速
    int GetPowerMode(); // 获取上电状态
    enum _powerMode{
        Off=0,
        Accessory,
        Run,
        CrankRequest
    }; // 上电状态
    double GetMP412CarSpeed(); // 获取PULSE口的车速

    int GetSysControlMethod(); // 获取系统控制方法
    double GetError(); // 获得当前车速误差
    void GetPIDParams(double *params);
    void SaveLoggerFile(const char* filePath); // 保存日志
    void CheckIfSaveLogger(); // 检查是否要保存日志

private:
    void InitParameters(); // 参数初始化
    int ReadListeningTeachFile(const std::string &fileName); // 解析XXX文件 获得目标曲线
    void CheckTimerAccuracy(); // 测试定时器准确度

    double CalculateUpperLowerBound(QVector<double>& time, QVector<double>& speed,
                                    QVector<double>& upperTime, QVector<double>& upperSpeed,
                                    QVector<double>& lowerTime, QVector<double>& lowerSpeed); // 计算曲线容差边界
    void CalculateSgnOfAcc(size_t index, int& accBefore, int&accAfter); // 计算目标曲线加速度方向
    void CalibrateBoundSpeed(const QVector<double> &boundTime, QVector<double>& boundSpeed, const int lowerUpperBound); // 容差边界速度第二次整定
    void ProcessSpeedWithSameTime(const QVector<double>& boundTime, QVector<double>& boundSpeed, const int lowerUpperBound); // 容差边界速度第三次整定
    void InitQCustomPlot(double maxY); // 初始化曲线绘布

    double GetElapsedMillisecond(struct timeval tvStart, struct timeval tvEnd); // 获取毫秒级时间差
    double GetDisplayLowerBound(double value); // 获取时间轴下限
    double GetDisplayUpperBound(double value); // 获取时间轴上限

    double GetCarSpeed(); // 获取车速

    void SendMoveCommand(double deltaBrake, double deltaAcc); // 发送踏板控制指令
    void LoggerStudySamples(); // 添加日志
    double GetCurrentTargetSpeed(); //获取当前目标速度

private:
    QCustomPlot *pQCustomPlot; // 绘布
    Configuration *conf; // 配置文件

    Logger *myLogger; // 日志
    bool autoSaveLogger; // 自动保存日志标志位

    SysControl* mySysControl; // 控制逻辑

    std::string curveFilePath; // 曲线文件保存路径

    typedef std::vector<std::pair<double,double> > PairData; // pair=<time,speed>
    PairData vp_data; // vector_pair_data 曲线数据
    bool isControlling; // 是否正在跟踪曲线
    struct timeval actionStartTime; // 整个工作的起始时间
    double elapsedSeconds; // 当前的工作时间 秒
    double actionDurationSecond; // 整个工作的持续时间 秒

    int timeStamp; // 时间戳

private:
    //控制运动
    void InitControlMethod(); // 初始化系统控制方法
    void PedalControl(); // 踏板控制逻辑
};

#endif // PEDAL_H
