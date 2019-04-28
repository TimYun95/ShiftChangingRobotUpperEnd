#ifndef SHIFTCLUTCHUI_H
#define SHIFTCLUTCHUI_H

#include <QWidget>
#include <QtWidgets/QComboBox>
#include<QVector>
#include <QPair>
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

    enum class ClutchReleasingMode {
        Normal = 0,
        Slowly,
        SlowlyWithControl,
        SlowlyWithRecovery
    };

    enum class ShiftChangingMode {
        OnlyClutch = 0,
        OnlyShift,
        ThreeAxis,
        FiveAxis,

        AutoShift = 9,
        ManualShift
    };

    enum class ClutchState
    {
        Released = 0,
        Pressed,
        SlowlyReleasing,
        SlowlyReleasingAtDeparture
    };
    QVector<QString> ClutchStateString = {"Up", "Down", "Lifting", "Lifting"};

    enum class ManualShiftState
    {
        Gear_N = 0,
        Gear_1,
        Gear_2,
        Gear_3,
        Gear_4,
        Gear_5,
        Gear_NLeft,
        Gear_NRight,
        Gear_NBack,
        Gear_6,
        Gear_R
    };
    QVector<QString> ManualShiftStateString = {"N", "1", "2", "3", "4", "5", "NLeft", "NRight", "NBack", "6", "R"};

    enum class AutoShiftState
    {
        Gear_P = 0,
        Gear_N,
        Gear_D,
        Gear_R
    };
    QVector<QString> AutoShiftStateString = {"P", "N", "D", "R"};

    void ConnectXMLSignalWithSlot(QWidget* stui); // 关联配置文件读取信号和槽

    void UpdateAnglesForShiftClutch(); // 更新挡位离合相关的角度值

signals:
    void ReadXMLFromShifting(); // 从挡位界面中读取配置文件

private slots:

    void UpdateShowingDatas(); // 更新相关显示信息
    void SetFalseSCConfirm(); // 设置确认挡位离合信息
    bool eventFilter(QObject *watched, QEvent *event); // 按下车型框更改车型


    void examtimer_timeout(); // 测试定时器触发


    void on_comboBox_way_currentIndexChanged(int index); // 改变换挡的方式“手动”或者“自动”

    void on_pushButton_save_clicked(); // 保存挡位离合信息
    void on_pushButton_read_clicked(); // 读取挡位离合信息

    void on_pushButton_confirmSC_clicked(); // 确认挡位离合信息

    void on_checkBox_sixshift_stateChanged(int arg1); // 存在六档开关
    void on_checkBox_backshift_stateChanged(int arg1); // 存在倒档开关
    void on_checkBox_autoset_stateChanged(int arg1); // 自动补齐开关

    void on_pushButton_record1_clicked(); // 记录对应挡位信息
    void on_pushButton_reset1_clicked(); // 重置对应挡位信息

    void on_pushButton_motor3minus_pressed(); // 四个槽控制离合运动
    void on_pushButton_motor3minus_released();
    void on_pushButton_motor3plus_pressed();
    void on_pushButton_motor3plus_released();

    void on_comboBox_clutch_currentIndexChanged(int index); // 是否打开离合速度框

    void on_pushButton_record2_clicked(); // 记录对应离合和离合速度信息
    void on_pushButton_reset2_clicked(); // 重置对应离合和离合速度信息

    void on_pushButton_startexam_clicked(); // 开始测试
    void on_pushButton_stopexam_clicked(); // 结束测试
    void on_pushButton_stopnow_clicked(); // 立即停止

    void on_comboBox_shiftaim_currentIndexChanged(int index); // 更改目标挡位
    void on_pushButton_shiftrun_clicked(); // 测试换挡路径
    void on_pushButton_shiftpause_clicked(); // 暂停测试换挡路径

    void on_pushButton_run3_clicked(); // 三轴换挡测试
    void on_pushButton_run5_clicked(); // 五轴换挡测试
    void on_pushButton_pause_clicked(); // 暂停测试三轴五轴换挡过程

    void on_pushButton_bottom_clicked(); // 测试到达离合位置 踩住
    void on_pushButton_top_clicked(); // 测试到达离合位置 松开
    void on_pushButton_speed_clicked(); // 测试离合松开速度
    void on_pushButton_clutchpause_clicked(); // 暂停测试离合位置或速度

    void on_pushButton_runtest_clicked(); // 手动操作开车换挡测试
    void on_pushButton_runtest_brkp_pressed(); // 手动控制刹车前进
    void on_pushButton_runtest_brkp_released(); // 手动控制刹车停止前进
    void on_pushButton_runtest_brkm_pressed(); // 手动控制刹车后退
    void on_pushButton_runtest_brkm_released(); // 手动控制刹车停止后退
    void on_pushButton_runtest_accp_pressed(); // 手动控制油门前进
    void on_pushButton_runtest_accp_released(); // 手动控制油门停止前进
    void on_pushButton_runtest_accm_pressed(); // 手动控制油门后退
    void on_pushButton_runtest_accm_released(); // 手动控制油门停止后退

private:
    Ui::ShiftClutchUI *ui;

    void SendMotionCmd(const double deltabrk, const double deltaacc, const bool ifshiftmode = false, const bool ifpause = false, const int aimshift = 0, const int aimclutch = (int)ClutchState::Released, const int howtochangeshift = (int)ShiftChangingMode::OnlyClutch, const int howtoreleaseclutch = (int)ClutchReleasingMode::Normal);
    void SendShiftClutchInfo(); // 发送挡位离合信息
    void SendResetSignal(); // 发送重置信号
    void SendEmergencyStopInfo(); // 发送急停位置信息

    bool ifPedalAtPositions(bool isbrklow, bool isacclow); // 踏板是否到位置
    bool SetTempArrivalFile(QVector<QVector<double>> anglelist, int length); // 设置临时到达问文件
    bool ifReachedPositionbyHand(int index); // 手动拉挡位是否到位

    QPair<double, double> CalOrthogonalFootOfPoints(QPair<double, double> p1, QPair<double, double> p2, QPair<double, double> pN); // 计算垂足位置
    void AutoRecordAtManualShift(); // 自动补齐中间挡位数据
    void ModifyShiftInfoItem(int itemindex, double angle1, double angle2); // 修改挡位条目信息
    void ModifyClutchInfoItem(int itemindex, double info); // 修改离合条目信息

    int GetNodeofShiftStatus(int givenshiftstatus); // 获得中间挡位节点
    QString GetShiftChangingRoute(); // 获得换挡轨迹
    int GetComboBoxIndexfromEnumIndexofShift(int enumindex); // 将枚举索引转化为组合框索引
    int GetEnumIndexfromComboBoxIndexofShift(int comboboxindex); // 将组合框索引转化为枚举索引
    void UnenableComboBoxItem(QComboBox* cBox, int num); // 禁止ComboBox的某个Item

    void RefreshShiftMap(); // 更新换挡示意图

    void RefreshCarTypeShowing(); // 更新换挡主控栏中的车型
    void RefreshCarGearTypeShowing(); // 更新换挡主控栏中的挡位类型
    void RefreshConfirmShiftClutchInfoState(); // 更新换挡主控栏中的确认挡位信息的状态
    void RefreshMainControlStrip(); // 更新换挡界面上侧主控栏

    void RefreshShiftLists(bool enableshift = true, bool enableclutch = true); // 更新挡位相关List控件
    void RefreshShiftComboBox(); // 更新挡位相关ComboBox控件
    void RefreshShiftSixAndBack(); // 更新是否存在6档和倒档
    void RefreshShiftAutoSet(); // 更新挡位示教记录自动补全
    void RefreshClutchSpeed(); // 更新离合速度
    void RefreshShiftClutchTeachPanel(); // 更新挡位离合示教界面

    void RefreshExamControlStrip(); // 更新换挡测试上侧控制栏
    void RefreshExamShiftPanel(); // 更新换挡测试中挡位界面
    void RefreshExamClutchPanel(); // 更新换挡测试中离合界面
    void RefreshExamPanel(); // 更新换挡测试界面

    void RefreshConfirmChangeShiftTimeState(); // 更新确认换挡时刻的状态
    void RefreshChangeShiftList(); // 更新换挡时刻界面中的List
    void RefreshChangeShiftPlot(); // 更新换挡时刻界面中的曲线绘制框
    void RefreshChangeShiftPanel(); // 更新换挡时刻界面

    void SetZeroShiftClutchInfo(bool enableshift = true, bool enableclutch = true); // 挡位相关信息和控件重新置零
    void SwitchConfirmShiftClutchInfoState(); // 转变换挡主控栏中的确认挡位信息的状态

    void InitialUI(); // 界面初始化

    void SendMoveCommandAll(QVector<double> values, QVector<int> cmdstate, const int customvariable = 0); // 发送移动指令 供换挡测试用
    void ResolveAndSendCmd(double* cmd); // 解析并发送指令
    void ResolveAndShowTime(double totaltime, double* partialtime, bool ifclutchused = false); // 解析并显示时间

private:

    QTimer* examtimer; // 测试用定时器
    QPixmap pic; // 输入的图像

    bool ifenablewaychangedeventhappen = false; // 是否允许换挡方式的改变
    bool ifenablecomboBoxchangedeventhappen = false; // 是否允许comboBox改变事件发生

    int currentclutchindex = 1; // 当前离合状态索引号
    int currentshiftindex = 1; // 当前挡位状态索引号
    int nextclutchindex = 0; // 目标离合状态索引号 暂时无用
    int nextshiftindex = 2; // 目标挡位状态索引号

    bool shiftexampause = false; // 挡位测试暂停
    bool clutchexampause = false; // 离合测试暂停
    bool exampause = false; // 换挡测试暂停

    /**
     * @brief examflag 测试标志位
     * -1---> 未初始化状态
     * 0 ---> 空闲状态 踏板受控
     * 1 ---> 开始测试
     * 2 ---> 挡位测试
     * 3 ---> 离合位置测试 踩住
     * 4 ---> 离合位置测试 松开
     * 5 ---> 离合速度测试
     * 6 ---> 三轴测试
     * 7 ---> 五轴测试
     * 8 ---> 换挡测试
     * 19 ---> 退出测试
     */
    int examflag = -1;

    int pauseflag = 0; // 暂停标志计数器

    double relativebrk = 0; // 刹车增量
    double relativeacc = 0; // 油门增量
    const double relativevalue = 1; // 增量大小

public:
    bool ifConfirmShiftClutchInfo = false; // 是否确认了挡位离合信息
    bool ifConfirmChangingShiftTime = false; // 是否确认了换挡时刻

    bool isExaming = false; // 正在测试挡位离合信息
};

#endif // SHIFTCLUTCHUI_H
