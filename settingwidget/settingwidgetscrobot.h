#ifndef SETTINGWIDGETSCROBOT_H
#define SETTINGWIDGETSCROBOT_H

#include <QWidget>

#include "settingbase.h"

namespace Ui {
class SettingWidgetSCRobot;
}

class SettingWidgetSCRobot : public QWidget, public SettingBase
{
    Q_OBJECT

public:
    explicit SettingWidgetSCRobot(QWidget *parent = 0);
    ~SettingWidgetSCRobot();

    virtual void LoadParameters(Configuration &conf);
    virtual bool StoreParameters(Configuration &conf);

private:
    Ui::SettingWidgetSCRobot *ui;
};

#endif // SETTINGWIDGETSCROBOT_H
