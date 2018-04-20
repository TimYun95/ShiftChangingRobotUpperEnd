#ifndef SETTINGWIDGETPEDALROBOTSTUDY_H
#define SETTINGWIDGETPEDALROBOTSTUDY_H

#include <QWidget>
#include <QString>
#include <QLineEdit>

#include "settingbase.h"

namespace Ui {
class SettingWidgetPedalRobotStudy;
}

class SettingWidgetPedalRobotStudy : public QWidget, public SettingBase
{
    Q_OBJECT

public:
    explicit SettingWidgetPedalRobotStudy(QWidget *parent = 0);
    ~SettingWidgetPedalRobotStudy();

    virtual void LoadParameters(Configuration &conf);
    virtual bool StoreParameters(Configuration &conf);

signals:
    void UpdateNedcParamsSignal();

private slots:
    void on_pushButton_readCalibratedParams_clicked();

private:
    bool CheckValid();
    void ReadCarTypeConfFile(const char* filePath);

private:
    Ui::SettingWidgetPedalRobotStudy *ui;

    const static int lineEditNum = 7;
    QLineEdit *mylineEdit[lineEditNum];
};

#endif // SETTINGWIDGETPEDALROBOTSTUDY_H
