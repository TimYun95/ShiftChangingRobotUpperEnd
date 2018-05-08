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
    SetLineEditValue(ui->lineEdit_s1p, conf.angleErr_P[1]);
    SetLineEditValue(ui->lineEdit_s2p, conf.angleErr_P[2]);

    // 运动速度
    SetLineEditValue(ui->lineEdit_c3, conf.curveMotionSpeed[0]);
    SetLineEditValue(ui->lineEdit_s13, conf.curveMotionSpeed[1]);
    SetLineEditValue(ui->lineEdit_s23, conf.curveMotionSpeed[2]);

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
    conf.angleErr_P[1] = GetLineEditValue(ui->lineEdit_s1p);
    conf.angleErr_P[2] = GetLineEditValue(ui->lineEdit_s2p);

    // 运动速度
    conf.curveMotionSpeed[0] = GetLineEditValue(ui->lineEdit_c3);
    conf.curveMotionSpeed[1] = GetLineEditValue(ui->lineEdit_s13);
    conf.curveMotionSpeed[2] = GetLineEditValue(ui->lineEdit_s23);

    // 起步油门位置
    conf.startAccAngleValue = GetLineEditValue(ui->lineEdit_startacc);

    return true;
}



