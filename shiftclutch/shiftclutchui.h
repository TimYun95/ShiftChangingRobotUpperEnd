#ifndef SHIFTCLUTCHUI_H
#define SHIFTCLUTCHUI_H

#include <QWidget>
#include <pedal/syscontrol.h>
#include <sys/time.h>

namespace Ui {
class ShiftClutchUI;
}

class ShiftClutchUI : public QWidget
{
    Q_OBJECT

public:
    explicit ShiftClutchUI(QWidget *parent = 0);
    ~ShiftClutchUI();

    void UpdateSC(); // 更新挡位离合界面
    void UpdateCar(); // 更新车辆及换挡信息

private slots:
    void on_comboBox_way_currentIndexChanged(int index); // 改变换挡的方式“手动”或者“自动”
    void on_checkBox_zero_stateChanged(int arg1); // 改变换挡路径规划方式“先回零挡”或“不回零挡”

    void examtimer_timeout(); // 测试定时器触发

    void on_pushButton_save_clicked(); // 保存挡位离合信息
    void on_pushButton_read_clicked(); // 读取挡位离合信息

    void on_pushButton_record1_clicked(); // 记录对应挡位信息
    void on_pushButton_reset1_clicked(); // 重置对应挡位信息

    void on_comboBox_clutch_currentIndexChanged(int index); // 是否打开离合速度框
    void on_pushButton_record2_clicked(); // 记录对应离合和离合速度信息
    void on_pushButton_reset2_clicked(); // 重置对应离合和离合速度信息

    void on_pushButton_startexam_clicked(); // 开始测试
    void on_pushButton_stopexam_clicked(); // 结束测试

    void on_pushButton_shiftrun_clicked(); // 测试换挡路径
    void on_comboBox_shiftaim_currentIndexChanged(int index); // 更改目标挡位
    void on_pushButton_shiftpause_clicked(); // 暂停测试换挡路径

    void on_pushButton_bottom_clicked(); // 测试到达离合位置 踩住
    void on_pushButton_top_clicked(); // 测试到达离合位置 松开
    void on_pushButton_speed_clicked(); // 测试离合松开速度
    void on_pushButton_clutchpause_clicked(); // 暂停测试离合位置或速度

    bool eventFilter(QObject *watched, QEvent *event); // 按下车型框更改车型

    void on_pushButton_motor3minus_pressed(); // 四个槽控制离合运动
    void on_pushButton_motor3plus_pressed();
    void on_pushButton_motor3minus_released();
    void on_pushButton_motor3plus_released();

private:
    Ui::ShiftClutchUI *ui;

    void initialui(); // 界面初始化
    void initiallist(); // 初始化list1和list2
    void resetcomboBox(); // 重置comboBox_shift和comboBox_shiftaim
    void resetlist(); // 重置list1和list2，并重置相关变量

    void SendMoveCommand(double clutch, double shift1, double shift2, bool run, bool ifboth, bool ifclutch); // 发送移动指令 供挡位离合测试用

private:
    SysControl* mySCControl; // 控制逻辑

    QTimer* examtimer; // 测试用定时器
    QPixmap pic; // 输入的图像

    bool ifenablewaychangedeventhappen; // 是否允许换挡方式的改变
    bool ifenablebackzeroeventhappen; // 是否允许回零方式的改变
    bool ifenablecomboBoxchangedeventhappen; // 是否允许comboBox改变事件发生

    bool shiftexampause = false; // 换挡测试暂停
    bool clutchexampause = false; // 离合测试暂停

    bool startexamtimeflag; // 开始计时标志位
    timeval starttime; // 开始计时的时刻
    timeval stoptime; // 停止计时的时刻

    unsigned int round; // 速度控制轮数
    unsigned int round2; // 速度控制轮数2

    /**
     * @brief examflag 测试标志位
     * 0 ---> 空闲状态
     * 1 ---> 开始测试
     * 2 ---> 换挡测试
     * 3 ---> 离合位置测试 踩住
     * 4 ---> 离合位置测试 松开
     * 5 ---> 离合速度测试
     * 6 ---> 退出测试
     */
    int examflag = 0;

public:
    bool haveReadXML;
};

#endif // SHIFTCLUTCHUI_H
