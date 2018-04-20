#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "pedal/pedalrobotui.h"
#include "shiftclutch/shiftclutchui.h"
#include "autodriverobotapiclient.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    AutoDriveRobotApiClient *robotclient; // 机器人客户端

    PedalRobotUI *pedalui; // 踏板界面

    QLabel* statusLabel;// 左下角状态栏
    QLabel* timeLabel; // 右下角状态栏 显示时间
    QVBoxLayout* vLayout; // 垂直布局
    QHBoxLayout* hLayout; // 水平布局
};

#endif // MAINWINDOW_H
