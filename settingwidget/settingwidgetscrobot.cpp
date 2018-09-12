#include "settingwidgetscrobot.h"
#include "ui_settingwidgetscrobot.h"

SettingWidgetSCRobot::SettingWidgetSCRobot(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingWidgetSCRobot)
{
    ui->setupUi(this);
    level=SettingBase::NormalUser;
}

SettingWidgetSCRobot::~SettingWidgetSCRobot()
{
    delete ui;
}

void SettingWidgetSCRobot::LoadParameters(Configuration &conf)
{
    // 手动角度容差
    SetLineEditValue(ui->lineEdit_c1, conf.angleErr_M[0]);
    SetLineEditValue(ui->lineEdit_s11, conf.angleErr_M[1]);
    SetLineEditValue(ui->lineEdit_s21, conf.angleErr_M[2]);

    // 自动角度容差
    SetLineEditValue(ui->lineEdit_c2, conf.angleErr_A[0]);
    SetLineEditValue(ui->lineEdit_s12, conf.angleErr_A[1]);
    SetLineEditValue(ui->lineEdit_s22, conf.angleErr_A[2]);

    // 过程角度容差
    SetLineEditValue(ui->lineEdit_sp, conf.angleErr_P);

    // 运动速度
    SetLineEditValue(ui->lineEdit_c3, conf.curveMotionSpeed[0]);
    SetLineEditValue(ui->lineEdit_s13, conf.curveMotionSpeed[1]);
    SetLineEditValue(ui->lineEdit_s23, conf.curveMotionSpeed[2]);

    // 踏板恢复百分比
    SetLineEditValue(ui->lineEdit_br, conf.pedalRecoveryPercent[0]);
    SetLineEditValue(ui->lineEdit_ar, conf.pedalRecoveryPercent[1]);

    // 起步油门位置
    SetLineEditValue(ui->lineEdit_startacc, conf.startAccAngleValue);

}

bool SettingWidgetSCRobot::StoreParameters(Configuration &conf)
{
    // 手动角度容差
    conf.angleErr_M[0] = GetLineEditValue(ui->lineEdit_c1);
    conf.angleErr_M[1] = GetLineEditValue(ui->lineEdit_s11);
    conf.angleErr_M[2] = GetLineEditValue(ui->lineEdit_s21);

    // 自动角度容差
    conf.angleErr_A[0] = GetLineEditValue(ui->lineEdit_c2);
    conf.angleErr_A[1] = GetLineEditValue(ui->lineEdit_s12);
    conf.angleErr_A[2] = GetLineEditValue(ui->lineEdit_s22);

    // 过程角度容差
    conf.angleErr_P = GetLineEditValue(ui->lineEdit_sp);

    // 运动速度
    conf.curveMotionSpeed[0] = GetLineEditValue(ui->lineEdit_c3);
    conf.curveMotionSpeed[1] = GetLineEditValue(ui->lineEdit_s13);
    conf.curveMotionSpeed[2] = GetLineEditValue(ui->lineEdit_s23);

    // 踏板恢复百分比
    conf.pedalRecoveryPercent[0] = GetLineEditValue(ui->lineEdit_br);
    conf.pedalRecoveryPercent[1] = GetLineEditValue(ui->lineEdit_ar);

    // 起步油门位置
    conf.startAccAngleValue = GetLineEditValue(ui->lineEdit_startacc);

    return true;
}



