#include "settingwidgetpedalrobotstudy.h"
#include "ui_settingwidgetpedalrobotstudy.h"

#include <fstream>

#include <QFileDialog>
#include <QMessageBox>

SettingWidgetPedalRobotStudy::SettingWidgetPedalRobotStudy(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingWidgetPedalRobotStudy)
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

SettingWidgetPedalRobotStudy::~SettingWidgetPedalRobotStudy()
{
    delete ui;
}

void SettingWidgetPedalRobotStudy::LoadParameters(Configuration &conf)
{
    for(int i=0; i<lineEditNum; ++i){
       SetLineEditValue(mylineEdit[i], conf.sysControlParams[i]);
    }
}

bool SettingWidgetPedalRobotStudy::StoreParameters(Configuration &conf)
{
    if(CheckValid() == false){
        return false;
    }

    for(int i=0; i<lineEditNum; ++i){
        conf.sysControlParams[i] = GetLineEditValue(mylineEdit[i]);
    }
    return true;
}

bool SettingWidgetPedalRobotStudy::CheckValid()
{
#ifdef DISABLE_STUDY_LIMIT
    return true;
#else
    const double lowerBound[3] = {0.0, 20.0, 0.0};
    const double upperBound[3] = {0.2, 50.0, 1000.0};
    for(int i=0; i<3; ++i){
        double value = GetLineEditValue(mylineEdit[i]);
        if(value < lowerBound[i] || value > upperBound[i]){
            mylineEdit[i]->setText("请重新输入!");
            QMessageBox::warning(NULL, "warning", QObject::tr("数值设定越界!请重新设置!"));
            return false;
        }
    }

    return true;
#endif
}

void SettingWidgetPedalRobotStudy::ReadCarTypeConfFile(const char *filePath)
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

void SettingWidgetPedalRobotStudy::on_pushButton_readCalibratedParams_clicked()
{
    QString str = QFileDialog::getOpenFileName(NULL, QString("请选择NEDC参数调整文件"), (Configuration::mainFolder+"/").c_str());
    if(str==""){
        return;
    }else if( !QFile::exists(str) ) {
        QMessageBox::warning(Q_NULLPTR, "Warning", tr("所选定的NEDC调整参数文件不存在，请重新选择！")) ;
        return;
    }

    ReadCarTypeConfFile(str.toStdString().c_str());
    emit UpdateNedcParamsSignal();
}
