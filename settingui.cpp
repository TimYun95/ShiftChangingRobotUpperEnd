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

#ifndef ENABLE_LOGIN_PASSWORD
#define ENABLE_LOGIN_PASSWORD
#endif

SettingUI::SettingUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingUI),
    ifhaveloggedin(false),
    haveReadXML(false)
{
    ui->setupUi(this);

    InitSettingsWidgetWithRobots();
}

SettingUI::~SettingUI()
{
    delete ui;
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
    QListWidgetItem* pdDeathZone=new QListWidgetItem(QObject::tr("死区"));
    settingMap[pdDeathZone]=new SettingWidgetPedalRobotDeathZone();

    if(Configuration::GetInstance()->pedalRobotUsage == SettingWidgetPedalRobotGetSpeed::NedcControl){
        QListWidgetItem* study=new QListWidgetItem(QObject::tr("学习"));
        SettingWidgetPedalRobotStudy* studyWidget = new SettingWidgetPedalRobotStudy();
        settingMap[study]=studyWidget;
        connect(studyWidget, SIGNAL(UpdateNedcParamsSignal()), this, SLOT(on_pushButton_saveSettings_clicked()));
    }else{
        QListWidgetItem* studyWltc=new QListWidgetItem(QObject::tr("学习"));
        SettingWidgetPedalRobotStudyWltc* studyWltcWidget = new SettingWidgetPedalRobotStudyWltc();
        settingMap[studyWltc]=studyWltcWidget;
        connect(studyWltcWidget, SIGNAL(UpdateWltcParamsSignal()), this, SLOT(on_pushButton_saveSettings_clicked()));
    }

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
    }else{
        QMessageBox::information(this,"提示",QObject::tr("!!!设置保存失败!!!"));
    }
}

void SettingUI::on_pushButton_readSettings_clicked()
{
    const QString txtpath = QString::fromStdString(Configuration::GetInstance()->carTypeFilePath);
    QString fileName = QFileDialog::getOpenFileName(this, tr("读取"), txtpath);
    std::string fn = NormalFile::GetFileName(fileName.toStdString().c_str());

    Configuration::GetInstance()->carTypeName = fn.substr(0, fn.length() - 4);

    if(Configuration::GetInstance()->ReadFromFile() == 0){
        QMessageBox::information( this,"提示", tr( (QString("读取").toStdString() + Configuration::GetInstance()->carTypeName + QString("成功").toStdString()).c_str() ) );
        haveReadXML = true;
    }else{
        QMessageBox::information(this,"提示", tr( (QString("!!!读取").toStdString() + Configuration::GetInstance()->carTypeName + QString("失败!!!").toStdString()).c_str() ) );
        return;
    }

    for(std::map<QListWidgetItem*,SettingBase*>::iterator iter=settingMap.begin(); iter!=settingMap.end(); iter++){
        SettingBase* sb = iter->second;
        sb->LoadParameters( *Configuration::GetInstance() );
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
        ui->pushButton_loginSettings->setText( tr("登录") );
    }
}

void SettingUI::ShowSettings()
{
    for(std::map<QListWidgetItem*,SettingBase*>::iterator iter=settingMap.begin();iter!=settingMap.end();iter++){
        QListWidgetItem* lwi=iter->first;

        ui->listWidget_settings->addItem(lwi);
    }
    ui->listWidget_settings->sortItems();
    ui->listWidget_settings->setCurrentRow(0);
}

void SettingUI::RemoveSettings()
{
    while(ui->listWidget_settings->count()){
        ui->listWidget_settings->takeItem(0);
    }

    for(std::map<QListWidgetItem*,SettingBase*>::iterator iter=settingMap.begin();iter!=settingMap.end();iter++){
        SettingBase* sb=iter->second;
        DisplaySettingWidget(sb,false);
    }
}

void SettingUI::UpdateAllSetUI()
{
    for(std::map<QListWidgetItem*,SettingBase*>::iterator iter=settingMap.begin(); iter!=settingMap.end(); iter++){
        SettingBase* sb = iter->second;
        sb->LoadParameters( *Configuration::GetInstance() );
    }
}



