#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QTimer>
#include <QDateTime>
#include <QDir>
#include "robotparams.h"
#include "configuration.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->setWindowTitle(tr("5轴自动驾驶机器人")); // 设置标题
    this->setWindowState(Qt::WindowMaximized); // 最大化窗体

    // 检查配置等文件夹是否存在 不存在的要新建
    QDir maindir( QString::fromStdString(Configuration::mainFolder) );
    if (!maindir.exists())
    {
        maindir.mkdir(QString::fromStdString(Configuration::mainFolder));
    }

    QDir sysdir( QString::fromStdString(Configuration::carTypeFilePath) );
    if (!sysdir.exists())
    {
        sysdir.mkdir(QString::fromStdString(Configuration::carTypeFilePath));
    }
    QDir pcdir( QString::fromStdString(Configuration::logFilePath) );
    if (!pcdir.exists())
    {
        pcdir.mkdir(QString::fromStdString(Configuration::logFilePath));
    }
    QDir sdddir( QString::fromStdString(Configuration::examFilePath) );
    if (!sdddir.exists())
    {
        sdddir.mkdir(QString::fromStdString(Configuration::examFilePath));
    }

    // 检查配置等文件是否存在 不存在的要新建 // dolater

    robotclient = AutoDriveRobotApiClient::GetInstance(); // 首次实例化客户端
    robotclient->StartUnifiedRobotApiClient("127.0.0.1", 2);

    QFont ft;
    ft.setPointSize(20);
    statusLabel = new QLabel();
    statusLabel->setFont(ft);
    statusLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    timeLabel = new QLabel();
    timeLabel->setFont(ft);
    timeLabel->setAlignment(Qt::AlignVCenter | Qt::AlignRight);

    pedalui = new PedalRobotUI(this, statusLabel, timeLabel); // 踏板界面添加

    vLayout =new QVBoxLayout();
    vLayout->addWidget(pedalui);
    hLayout=new QHBoxLayout();
    hLayout->addWidget(statusLabel);
    hLayout->addWidget(timeLabel);
    vLayout->addLayout(hLayout);
    ui->centralWidget->setLayout(vLayout);
}

MainWindow::~MainWindow()
{
    robotclient->StopUnifiedRobotApiClient();

    delete hLayout;
    delete vLayout;
    delete statusLabel;
    delete timeLabel;
    delete pedalui;
    delete ui;
}
