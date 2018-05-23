#ifndef SETTINGUI_H
#define SETTINGUI_H

#include <QWidget>
#include <QListWidgetItem>
#include "settingwidget/settingbase.h"

namespace Ui {
class SettingUI;
}

class SettingUI : public QWidget
{
    Q_OBJECT

public:
    explicit SettingUI(QWidget *parent = 0);
    ~SettingUI();

    void UpdateAllSetUI(); // 更新所有设置界面信息

private slots:
    void on_listWidget_settings_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous); // 换设置界面
    void on_pushButton_changePassword_clicked(); // 换密码
    void on_pushButton_saveSettings_clicked(); // 保存
    void on_pushButton_readSettings_clicked(); // 读取
    void on_pushButton_loginSettings_clicked(); // 登录和退出

private:
    Ui::SettingUI *ui;

    bool ifhaveloggedin; // 是否以及登录
    std::map<QListWidgetItem* , SettingBase*> settingMap; // 界面索引

    void InitSettingsWidgetWithRobots(); // 初始设置界面
    void DisplaySettingWidget(SettingBase *sb, bool b); // 设置界面可见性
    void ShowSettings(); // 显示设置
    void RemoveSettings(); // 移除设置

public:
    bool haveReadXML; // 读取了车型配置文件
    bool haveChangeUsage; // 更改了使用方式 NEDC/WLTC切换
};

#endif // SETTINGUI_H
