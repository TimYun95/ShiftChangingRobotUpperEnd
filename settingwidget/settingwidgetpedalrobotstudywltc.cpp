#include "settingwidgetpedalrobotstudywltc.h"
#include "ui_settingwidgetpedalrobotstudywltc.h"

#include <fstream>

#include <QFileDialog>
#include <QMessageBox>

SettingWidgetPedalRobotStudyWltc::SettingWidgetPedalRobotStudyWltc(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingWidgetPedalRobotStudyWltc)
{
    ui->setupUi(this);
    level=SettingBase::NormalUser;

    mylineEdit[0] = ui->lineEdit;
    mylineEdit[1] = ui->lineEdit_2;
    mylineEdit[2] = ui->lineEdit_3;
    mylineEdit[3] = ui->lineEdit_4;
    mylineEdit[4] = ui->lineEdit_5;
    mylineEdit[5] = ui->lineEdit_6;
    mylineEdit[6] = ui->lineEdit_7;
}

SettingWidgetPedalRobotStudyWltc::~SettingWidgetPedalRobotStudyWltc()
{
    delete ui;
}

void SettingWidgetPedalRobotStudyWltc::LoadParameters(Configuration &conf)
{
    for(int i=0; i<lineEditNum; ++i){
        SetLineEditValue(mylineEdit[i], conf.sysControlParamsWltc[i]);
    }
}

bool SettingWidgetPedalRobotStudyWltc::StoreParameters(Configuration &conf)
{
    for(int i=0; i<lineEditNum; ++i){
        conf.sysControlParamsWltc[i] = GetLineEditValue(mylineEdit[i]);
    }
    return true;
}

void SettingWidgetPedalRobotStudyWltc::ReadCarTypeConfFile(const char *filePath)
{
    std::ifstream ifs(filePath, std::fstream::binary);
    if(ifs.fail()){
        PRINTF(LOG_ERR, "%s: fail to open file=%s\n", __func__, filePath);
        return;
    }

    double value;
    for(size_t i=0; i<lineEditNum; ++i){
        ifs>>value;
        SetLineEditValue(mylineEdit[i], value);
    }
    ifs.close();
}

void SettingWidgetPedalRobotStudyWltc::on_pushButton_readCalibratedParams_clicked()
{
    QString str = QFileDialog::getOpenFileName(NULL, QString("请选择WLTC参数调整文件"), (Configuration::mainFolder+"/").c_str());
    if(str==""){
        return;
    }else if( !QFile::exists(str) ) {
        QMessageBox::warning(Q_NULLPTR, "Warning", tr("所选定的WLTC调整参数文件不存在，请重新选择！")) ;
        return;
    }

    ReadCarTypeConfFile(str.toStdString().c_str());
    emit UpdateWltcParamsSignal();
}
