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

    void ConnectXMLSignalWithSlot(QWidget* sc); // 关联配置文件读取信号和槽

signals:
    void ReadXMLFromSetting(); // 从设置界面中读取配置文件
    void SaveXMLFromSetting(); // 从设置界面中保存配置文件

private slots:
    void on_listWidget_settings_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous); // 换设置界面
    void on_pushButton_changePassword_clicked(); // 换密码
    void on_pushButton_saveSettings_clicked(); // 保存
    void on_pushButton_readSettings_clicked(); // 读取
    void on_pushButton_loginSettings_clicked(); // 登录和退出

    void UpdateAllSetUI(); // 更新所有设置界面信息

private:
    Ui::SettingUI *ui;

    bool ifhaveloggedin; // 是否以及登录
    std::map<QListWidgetItem* , SettingBase*> settingMap; // 界面索引

    void InitSettingsWidgetWithRobots(); // 初始设置界面
    void DisplaySettingWidget(SettingBase *sb, bool b); // 设置界面可见性
    void ShowSettings(int recoveryone = 0); // 显示设置
    int RemoveSettings(); // 移除设置
};

#endif // SETTINGUI_H
