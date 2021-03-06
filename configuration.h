#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <string>
#include <QDomElement>
#include "robotparams.h"

class Configuration
{
public:
    static Configuration* GetInstance();
private:
    explicit Configuration();

public:
    ~Configuration();

public:
    int  ReadFromFile();
    int  SaveToFile();
    void LoadDefaultConfiguration();

public:
    std::string carTypeName; // 车型名称
    std::string defaultFile; // 默认曲线文件 XXX_ARM
    std::string normalPassword; // 设置登陆密码
    std::string rootPassword; // 设置高级登陆密码
    unsigned int translateSpeed; // 平移速度 执行非监视模式的踏板运动速度

    double deathPos[RobotParams::axisNum]; // 死区位置 共6个轴
    double limPos[RobotParams::axisNum]; // 极限位置 共6个轴
    double brakeThetaAfterGoHome; // 回原后的刹车踏板位置
    unsigned int getSpeedMethod; // 获得车速方式
    unsigned int calcSpeedErrorMethod; // 车速误差计算方式
    unsigned int pedalRobotUsage; // 程序控制的用途切换
    double sysControlParams[7]; // NEDC学习的参数
    double sysControlParamsWltc[7]; // WLTC学习的参数
    double pedalStartTimeS; // 调试起始时间

    bool ifManualShift; // 是否是手动挡
    double angleErr_M[3]; // 手动调整电机轴角度容差 相差该角度下认为到达该相应电机轴位置
    double angleErr_A[3]; // 自动调整电机轴角度容差 相差该角度下认为到达该相应电机轴位置
    double angleErr_P; // 过程调整电机轴角度容差 相差该角度下认为到达该相应电机轴位置
    double shiftAxisAngles1[11]; // 换挡电机轴1的各挡位角度
    double shiftAxisAngles2[11]; // 换挡电机轴2的各挡位角度
    bool ifExistSixShift; // 是否存在六挡
    bool ifExistBackShift; // 是否存在倒车挡
    double clutchAngles[2]; // 离合电机轴两个位置的角度 踩下和松开
    double clutchUpSpeed; // 离合松开的速度
    double clutchUpSpeedAtDeparture; // 起步时离合松开的速度
    double pedalRecoveryPercent[2]; // 抬离合时踏板恢复的百分比
    bool ifAutoRecordMidN; // 是否自动补全中间空挡

    double curveMotionSpeed[3]; // 曲线运动速度 目标位置距离当前位置的距离 距离按照总距离的百分比计算 该速度即这个百分比
    double startAccAngleValue; // 起步油门的最终角度

    unsigned int accMotionAtClutchReleasing; // 油门踏板在离合抬起时的动作 1 -- 不动 2 -- 受控 3 -- 恢复

private:
    const QString arrayPrefix;
    const size_t arrayPrefixLen;

    bool FindElement(QDomElement& element, const char* name);
    void LoadString(QDomElement& element, const char* name, std::string& str);
    void LoadNumber(QDomElement& element, const char* name, bool& value);
    void LoadNumber(QDomElement& element, const char* name, double& value);
    void LoadNumber(QDomElement& element, const char* name, unsigned int& value);
    void LoadNumberArray(QDomElement& element, const char* name, double* pValue, const int len);

    void SaveString(QDomDocument& doc, QDomElement& root, const char* name, const std::string& str);
    template<typename T> void SaveNumber(QDomDocument& doc, QDomElement& root, const char* name, const T& value);
    template<typename T> void SaveNumberArray(QDomDocument& doc, QDomElement& root, const char* name, const T* pValue, const int len);

public:
    // /Documents/AutoDrivingRobot                 mainFolder
    //      |
    //      ---- /system_files                                   carTypeFilePath
    //                  |
    //                  ---- /softStop.txt                       softStopFilePath
    //                  ---- /examsoftStop.txt               examsoftStopFilePath
    //                  ---- /origin.txt                            originFilePath
    //                  ---- /log4qt.properties               logCodeConfPath
    //      |
    //      ---- /log_files                                          logFilePath
    //                  |
    //                  ---- /log_curve                            logCurvePath
    //                  ---- /log_code                             logCodePath
    //                  ---- /log_code                             logCodePath
    //      |
    //      ---- /stdand_files                                     stdFilePath
    //                  |
    //                  ---- /NEDC
    //                  ---- /NEDC_ARM
    //                  ---- /WLTC
    //                  ---- /WLTC_ARM
    //                  ---- /SC_ARM

    static const std::string mainFolder;

    static const std::string sysFilePath;

    static const std::string softStopFilePath;
    static const std::string examsoftStopFilePath;

    static const std::string originFilePath;

    static const std::string logFilePath;
    static const std::string logCurvePath;
    static const std::string logCodePath;
    static const std::string logCodeConfPath;

    static const std::string stdFilePath;

    static const std::string temparrivalFilePath;
};

#endif // CONFIGURATION_H
