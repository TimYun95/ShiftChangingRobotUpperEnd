#ifndef SETTINGWIDGETPEDALROBOTDEATHZONE_H
#define SETTINGWIDGETPEDALROBOTDEATHZONE_H

#include <QWidget>

#include "settingbase.h"

namespace Ui {
class SettingWidgetPedalRobotDeathZone;
}

class SettingWidgetPedalRobotDeathZone : public QWidget, public SettingBase
{
    Q_OBJECT

public:
    explicit SettingWidgetPedalRobotDeathZone(QWidget *parent = 0);
    ~SettingWidgetPedalRobotDeathZone();

    virtual void LoadParameters(Configuration &conf);
    virtual bool StoreParameters(Configuration &conf);

private:
    Ui::SettingWidgetPedalRobotDeathZone *ui;
};

#endif // SETTINGWIDGETPEDALROBOTDEATHZONE_H
