#include "settingui.h"
#include "ui_settingui.h"

#include <QInputDialog>
#include <QFileDialog>
#include <QMessageBox>
#include "configuration.h"
#include "fileoperation/normalfile.h"
#include "settingwidget/settingwidgetpedalrobotdeathzone.h"
#include "settingwidget/settingwidgetpedalrobotgetspeed.h"
#include "settingwidget/settingwidgetpedalrobotstudy.h"
#include "settingwidget/settingwidgetpedalrobotstudywltc.h"
#include "settingwidget/settingwidgetscrobot.h"
#include "autodriverobotapiclient.h"

#ifndef ENABLE_LOGIN_PASSWORD
#define ENABLE_LOGIN_PASSWORD
#endif

SettingUI::SettingUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingUI),
    ifhaveloggedin(false)
{
    ui->setupUi(this);

    InitSettingsWidgetWithRobots();
}

SettingUI::~SettingUI()
{
    delete ui;
}

void SettingUI::ConnectXMLSignalWithSlot(QWidget* sc)
{
    connect( sc, SIGNAL(ReadXMLFromShifting()), this, SLOT(UpdateAllSetUI()) );
}

void SettingUI::on_listWidget_settings_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous)
{
    /*之前的隐藏*/
    if(settingMap.find(previous)!=settingMap.end()){
        SettingBase* sb=settingMap[previous];
        DisplaySettingWidget(sb,false);
    }

    /*现在的显示*/
    if(settingMap.find(current)!=settingMap.end()){
        SettingBase* sb=settingMap[current];
        DisplaySettingWidget(sb,true);
    }
}

void SettingUI::InitSettingsWidgetWithRobots()
{
    QListWidgetItem* pdGetSpeed=new QListWidgetItem(QObject::tr("测速"));
    settingMap[pdGetSpeed]= new SettingWidgetPedalRobotGetSpeed();
    QListWidgetItem* pdDeathZone=new QListWidgetItem(QObject::tr("极限"));
    settingMap[pdDeathZone]=new SettingWidgetPedalRobotDeathZone();

    QListWidgetItem* study=new QListWidgetItem(QObject::tr("学习N"));
    SettingWidgetPedalRobotStudy* studyWidget = new SettingWidgetPedalRobotStudy();
    settingMap[study]=studyWidget;
    connect(studyWidget, SIGNAL(UpdateNedcParamsSignal()), this, SLOT(on_pushButton_saveSettings_clicked()));

    QListWidgetItem* studyWltc=new QListWidgetItem(QObject::tr("学习W"));
    SettingWidgetPedalRobotStudyWltc* studyWltcWidget = new SettingWidgetPedalRobotStudyWltc();
    settingMap[studyWltc]=studyWltcWidget;
    connect(studyWltcWidget, SIGNAL(UpdateWltcParamsSignal()), this, SLOT(on_pushButton_saveSettings_clicked()));

    QListWidgetItem* sc=new QListWidgetItem(QObject::tr("换挡"));
    settingMap[sc]=new SettingWidgetSCRobot();

    // add widget
    for(std::map<QListWidgetItem*,SettingBase*>::iterator iter=settingMap.begin(); iter!=settingMap.end(); iter++){
        SettingBase* sb = iter->second;
        sb->LoadParameters(*Configuration::GetInstance());
        QWidget* w=dynamic_cast<QWidget*>(sb);
        if(w){
            ui->widget->layout()->addWidget(w);
        }
        DisplaySettingWidget(sb,false);
    }

    ui->pushButton_changePassword->setEnabled(false);
    ui->pushButton_saveSettings->setEnabled(false);
    ui->pushButton_readSettings->setEnabled(false);
    ui->pushButton_loginSettings->setEnabled(true);
}

void SettingUI::DisplaySettingWidget(SettingBase *sb, bool b) // b=show or not
{
    QWidget* w=dynamic_cast<QWidget*>(sb);
    if(w){
        if(b) {
            w->show();
        }else{
            w->hide();
        }
    }else{
        PRINTF(LOG_ERR, "####### error DisplaySettingWidget #######\n");
    }
}

void SettingUI::on_pushButton_changePassword_clicked()
{
    bool ok;
    QString password = QInputDialog::getText(this,QObject::tr("提示"),QObject::tr("请输入新密码"),
                                           QLineEdit::Normal,"",&ok);
    if(ok){
        if(password.isEmpty()){
            QMessageBox::information(this,"提示",QObject::tr("密码必须非空"));
            return;
        }

        Configuration::GetInstance()->normalPassword = password.toStdString();
        if(Configuration::GetInstance()->SaveToFile() == 0){
            QMessageBox::information(this,"提示",QObject::tr("密码修改成功"));
        }else{
            QMessageBox::information(this,"提示",QObject::tr("!!!密码修改失败!!!"));
        }
    }
}

void SettingUI::on_pushButton_saveSettings_clicked()
{
    for(std::map<QListWidgetItem*,SettingBase*>::iterator iter=settingMap.begin();iter!=settingMap.end();iter++){
        SettingBase* bs=iter->second;
        if( bs->StoreParameters( *Configuration::GetInstance() ) == false){
            return;
        }
    }

    if(Configuration::GetInstance()->SaveToFile()==0){
        QMessageBox::information(this,"提示",QObject::tr("设置保存成功"));
        int tempindex = RemoveSettings();
        ShowSettings(tempindex);

        emit SaveXMLFromSetting();

        std::vector<double> positiveLimit
                = {Configuration::GetInstance()->limPos[0],
                     Configuration::GetInstance()->limPos[1],
                     Configuration::GetInstance()->limPos[2],
                     Configuration::GetInstance()->limPos[3],
                     Configuration::GetInstance()->limPos[4],
                     Configuration::GetInstance()->limPos[5]};

        std::vector<double> negativeLimit
                = {Configuration::GetInstance()->deathPos[0],
                     Configuration::GetInstance()->deathPos[1],
                     Configuration::GetInstance()->deathPos[2],
                     Configuration::GetInstance()->deathPos[3],
                     Configuration::GetInstance()->deathPos[4],
                     Configuration::GetInstance()->deathPos[5]};

        AutoDriveRobotApiClient::GetInstance()->Send_SetPositionLimitConf(positiveLimit, negativeLimit);

        AutoDriveRobotApiClient::GetInstance()->Send_SaveAndSendConfMsg(true, true);
    }else{
        QMessageBox::information(this,"提示",QObject::tr("!!!设置保存失败!!!"));
    }
}

void SettingUI::on_pushButton_readSettings_clicked()
{
    const QString txtpath = QString::fromStdString(Configuration::sysFilePath);
    QString fileName = QFileDialog::getOpenFileName(this, tr("读取"), txtpath, tr("XML Files(*.xml)"));
    if (fileName == "") return;
    std::string fn = NormalFile::GetFileName(fileName.toStdString().c_str());

    std::string tempcarname = Configuration::GetInstance()->carTypeName;
    Configuration::GetInstance()->carTypeName = fn;

    if(Configuration::GetInstance()->ReadFromFile() == 0){
        QMessageBox::information( this,"提示", tr( (QString("读取").toStdString() + Configuration::GetInstance()->carTypeName + QString("成功").toStdString()).c_str() ) );
        int tempindex = RemoveSettings();
        ShowSettings(tempindex);

        for(std::map<QListWidgetItem*,SettingBase*>::iterator iter=settingMap.begin(); iter!=settingMap.end(); iter++){
            SettingBase* sb = iter->second;
            sb->LoadParameters( *Configuration::GetInstance() );
        }

        emit ReadXMLFromSetting();
    }else{
        QMessageBox::information(this,"提示", tr( (QString("!!!读取").toStdString() + Configuration::GetInstance()->carTypeName + QString("失败!!!").toStdString()).c_str() ) );
        Configuration::GetInstance()->carTypeName = tempcarname;
        return;
    }
}

void SettingUI::on_pushButton_loginSettings_clicked()
{
    if(!ifhaveloggedin){
#ifdef ENABLE_LOGIN_PASSWORD
        bool ok;
        QString password=QInputDialog::getText(this,QObject::tr("Password"),QObject::tr("请输入密码"),
                                               QLineEdit::Password,"",&ok);
        if(ok){
            if(password.toStdString() == Configuration::GetInstance()->normalPassword){
                ifhaveloggedin = true;
            }
#else
        ifhaveloggedin = true;
        {
#endif
            if(ifhaveloggedin){
                ShowSettings();
                ui->pushButton_changePassword->setEnabled(true);
                ui->pushButton_saveSettings->setEnabled(true);
                ui->pushButton_readSettings->setEnabled(true);
                ui->pushButton_loginSettings->setText( tr("退出设置") );
            }else{
                QMessageBox::information( this, "提示", tr("密码错误!") );
            }
        }
    }else{/* exit settings */
        RemoveSettings();
        ifhaveloggedin = false;
        ui->pushButton_changePassword->setEnabled(false);
        ui->pushButton_saveSettings->setEnabled(false);
        ui->pushButton_readSettings->setEnabled(false);
        ui->pushButton_loginSettings->setText( tr("登录设置") );
    }
}

void SettingUI::ShowSettings(int recoveryone)
{
    for(std::map<QListWidgetItem*,SettingBase*>::iterator iter=settingMap.begin();iter!=settingMap.end();iter++){
        QListWidgetItem* lwi=iter->first;

        if (Configuration::GetInstance()->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::NedcControl && lwi->text() != "学习W")
        {
            ui->listWidget_settings->addItem(lwi);
        }

        if (Configuration::GetInstance()->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::WltcControl && lwi->text() != "学习N")
        {
            ui->listWidget_settings->addItem(lwi);
        }

    }
    ui->listWidget_settings->sortItems();
    ui->listWidget_settings->setCurrentRow(recoveryone);
}

int SettingUI::RemoveSettings()
{
    int lastrow = ui->listWidget_settings->currentRow();

    while(ui->listWidget_settings->count()){
        ui->listWidget_settings->takeItem(0);
    }

    for(std::map<QListWidgetItem*,SettingBase*>::iterator iter=settingMap.begin();iter!=settingMap.end();iter++){
        SettingBase* sb=iter->second;
        DisplaySettingWidget(sb,false);
    }

    return lastrow;
}

void SettingUI::UpdateAllSetUI()
{
    int tempindex = RemoveSettings();
    ShowSettings(tempindex);

    for(std::map<QListWidgetItem*,SettingBase*>::iterator iter=settingMap.begin(); iter!=settingMap.end(); iter++){
        SettingBase* sb = iter->second;
        sb->LoadParameters( *Configuration::GetInstance() );
    }
}



