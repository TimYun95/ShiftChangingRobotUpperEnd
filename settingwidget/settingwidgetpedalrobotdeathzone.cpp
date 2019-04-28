#include "settingwidgetpedalrobotdeathzone.h"
#include "ui_settingwidgetpedalrobotdeathzone.h"

#include <QMessageBox>

SettingWidgetPedalRobotDeathZone::SettingWidgetPedalRobotDeathZone(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingWidgetPedalRobotDeathZone)
{
    ui->setupUi(this);
    level=SettingBase::NormalUser;
}

SettingWidgetPedalRobotDeathZone::~SettingWidgetPedalRobotDeathZone()
{
    delete ui;
}

void SettingWidgetPedalRobotDeathZone::LoadParameters(Configuration &conf)
{
    // 监听模式的正极限
    SetLineEditValue(ui->lineEdit_brakePositionLimit, conf.limPos[0]);
    SetLineEditValue(ui->lineEdit_accPositionLimit, conf.limPos[1]);
    SetLineEditValue(ui->lineEdit_clutchPositionLimit, conf.limPos[2]);
    SetLineEditValue(ui->lineEdit_shift1PositionLimit, conf.limPos[3]);
    SetLineEditValue(ui->lineEdit_shift2PositionLimit, conf.limPos[4]);

    // 死区位置
    SetLineEditValue(ui->lineEdit_brakeDeathOpenValue, conf.deathPos[0]);
    SetLineEditValue(ui->lineEdit_accDeathOpenValue, conf.deathPos[1]);
    SetLineEditValue(ui->lineEdit_clutchDeathOpenValue, conf.deathPos[2]);
    SetLineEditValue(ui->lineEdit_shift1DeathOpenValue, conf.deathPos[3]);
    SetLineEditValue(ui->lineEdit_shift2DeathOpenValue, conf.deathPos[4]);

    // 回原后刹车位置
    SetLineEditValue(ui->lineEdit_brakeThetaAfterGoHome, conf.brakeThetaAfterGoHome);

    // 回原速度
    SetLineEditValue(ui->lineEdit_translatespeed, conf.translateSpeed);
}

bool SettingWidgetPedalRobotDeathZone::StoreParameters(Configuration &conf)
{
    // 监听模式的正极限
    conf.limPos[0] = GetLineEditValue(ui->lineEdit_brakePositionLimit);
    conf.limPos[1] = GetLineEditValue(ui->lineEdit_accPositionLimit);
    conf.limPos[2] = GetLineEditValue(ui->lineEdit_clutchPositionLimit);
    conf.limPos[3] = GetLineEditValue(ui->lineEdit_shift1PositionLimit);
    conf.limPos[4] = GetLineEditValue(ui->lineEdit_shift2PositionLimit);

    // 死区位置
    conf.deathPos[0] = GetLineEditValue(ui->lineEdit_brakeDeathOpenValue);
    conf.deathPos[1] = GetLineEditValue(ui->lineEdit_accDeathOpenValue);
    conf.deathPos[2] = GetLineEditValue(ui->lineEdit_clutchDeathOpenValue);
    conf.deathPos[3] = GetLineEditValue(ui->lineEdit_shift1DeathOpenValue);
    conf.deathPos[4] = GetLineEditValue(ui->lineEdit_shift2DeathOpenValue);

    // 回原后刹车位置
    conf.brakeThetaAfterGoHome = GetLineEditValue(ui->lineEdit_brakeThetaAfterGoHome);

    // 回原速度
    conf.translateSpeed = GetLineEditValue(ui->lineEdit_translatespeed);

    return true;
}
