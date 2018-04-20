#include "settingwidgetpedalrobotgetspeed.h"
#include "ui_settingwidgetpedalrobotgetspeed.h"

SettingWidgetPedalRobotGetSpeed::SettingWidgetPedalRobotGetSpeed(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingWidgetPedalRobotGetSpeed)
{
    ui->setupUi(this);
    level=SettingBase::NormalUser;
}

SettingWidgetPedalRobotGetSpeed::~SettingWidgetPedalRobotGetSpeed()
{
    delete ui;
}

void SettingWidgetPedalRobotGetSpeed::LoadParameters(Configuration &conf)
{
    //测速方法
    ui->comboBox_getSpeedMethod->setCurrentIndex(conf.getSpeedMethod);
    ui->comboBox_calcSpeedErrorMethod->setCurrentIndex(conf.calcSpeedErrorMethod);

    //程序用途
    ui->comboBox_pedalRobotUsage->setCurrentIndex(conf.pedalRobotUsage);

    //调试起始时间
    ui->lineEdit_pedalStartTimeS->setText(QString::number(conf.pedalStartTimeS));
}

bool SettingWidgetPedalRobotGetSpeed::StoreParameters(Configuration &conf)
{
    //测速方法
    conf.getSpeedMethod = ui->comboBox_getSpeedMethod->currentIndex();
    conf.calcSpeedErrorMethod = ui->comboBox_calcSpeedErrorMethod->currentIndex();

    //程序用途
    conf.pedalRobotUsage = ui->comboBox_pedalRobotUsage->currentIndex();

    //调试起始时间
    conf.pedalStartTimeS = ui->lineEdit_pedalStartTimeS->text().toDouble();

    return true;
}
