#ifndef PEDALROBOTUI_H
#define PEDALROBOTUI_H

#include <mutex>

#include <QTimer>
#include <QWidget>
#include <QKeyEvent>
#include <QPushButton>
#include <QThread>

#include "shiftclutch/shiftclutchui.h"
#include "pedalrobot.h"
#include "configuration.h"
#include "settingui.h"

namespace Ui {
class PedalRobotUI;
}

class PedalRobotUI : public QWidget
{
    Q_OBJECT

public:
    explicit PedalRobotUI(QWidget *parent, QLabel *_status, QLabel *_time);
    ~PedalRobotUI();

    void LockMutex(); // 加锁
    void UnlockMutex(); // 拆锁

public slots:
    void PedalTimerDone(); // 定时器函数

private slots:
    void SingleAxisPressed(); // 单轴按下
    void SingleAxisReleased(); // 单轴弹起

    void on_pushButton_origin_clicked(); // “回原”按下
    void on_pushButton_softStop_clicked(); // “停止”按下

    void on_pushButton_softStop_liftPedals_clicked(); // “停止回抬”按下
//    void on_pushButton_slowlybrake_clicked(); // “缓踩刹车”按下
    void on_pushButton_startAction_clicked(); // “曲线运行”按下
    void on_pushButton_saveLoggerFile_clicked(); // “保存日志”按下

//    void on_pushButton_nvh_start_clicked(); // NVH开始
//    void on_pushButton_nvh_stop_clicked(); // NVH结束
//    void on_pushButton_nvh_log_clicked(); // NVH曲线日志保存
//    void on_comboBox_nvh_mode_currentIndexChanged(int index); // 切换NVH模式
//    void on_pushButton_confirmaim_clicked(); // 确认目标点

//    void on_pushButton_nvh_run_clicked(); // NVH曲线运行
//    void on_pushButton_nvh_softstop_clicked(); // NVH曲线停止
//    void on_pushButton_nvh_slowbrake_clicked(); // NVH曲线停止 缓踩刹车

//    /* <ACD> */

//    void on_pushButton_saveModelSelect_clicked();
//    void on_pushButton_refresh_clicked();
//    void on_pushButton_startACD_clicked();
//    void on_pushButton_stopAndLeft_clicked();
//    void on_pushButton_startACDonline_clicked();
//    void on_pushButton_refreshPredef_clicked();
//    void on_tabWidget_currentChanged(int index);
//    void on_pushButton_change_clicked();

//    /* </ACD> */



protected:
    void InitWidgets(); // 初始化界面
    void UpdateWidgets(); // 定时更新界面
    void UpdateCarTypeWidget(); // 更新车型相关控件
    void UpdateGetSpeedWidget(); // 更新车速获得方式相关控件
    void EnableButtonsForGoHome(bool enable); // 回原前后的控件使能变化

    bool eventFilter(QObject *watched, QEvent *event); // 事件过滤 作快捷键用
    bool FilterTabSwitchKey(QEvent *event); // 切换TAB快捷键
    bool FilterSingleAxisKey(QEvent *event); // 单轴运动快捷键
    bool SingleAxisKeyAction(const int axisNum, const bool posDirection, const bool keyPress); // 单轴快捷键识别

    void SetSingleAxisMove(const int axisNum, const int direction); // 设置单轴运动
    void SetSingleAxisButtonStyle(QPushButton *btn, const bool keyPress); // 设置单轴运动控件样式

    void RefreshSoftStopFile(); // 更新softStop.txt
    void RefreshOriginFile(); // 更新origin.txt

    bool RefreshNVHFile(const int index); // 更新NVH文件 包括NVHX和NVHX_ARM

protected:
    Ui::PedalRobotUI *ui;
    QTimer *pdTimer; // 定时器 更新数据、状态、控制等
    QLabel *statusL; // MainWindow状态栏
    QLabel *timeL; // MainWindow时间栏

    ShiftClutchUI *scui; // 挡位离合界面
    SettingUI *stui; // 设置界面

    Configuration *conf; // 车型配置文件
    PedalRobot *pdRobot; // 踏板机器人逻辑实现

    std::mutex m_timerMutex; // 线程互斥锁
    bool m_enableMutex; // 是否使用mutex

    bool ifSendGoHome; // 是否发送了回原指令
    unsigned int GoHomeRound; // 回原指令发送失效的界面周期轮数
};

#endif // PEDALROBOTUI_H
