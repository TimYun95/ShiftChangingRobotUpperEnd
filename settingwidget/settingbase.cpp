#include "settingbase.h"

#include <assert.h>

#include <QString>

SettingBase::SettingBase()
{
    level=RootUser;
    if(RobotParams::axisNum > RobotParams::UIAxisNum){
        PRINTF(LOG_CRIT, "%s: axisNum(%d) should never be larger than UIAxisNum(%d)",
               __func__, RobotParams::axisNum, RobotParams::UIAxisNum);
        assert(RobotParams::axisNum <= RobotParams::UIAxisNum);
    }
}

SettingBase::~SettingBase()
{
}

void SettingBase::SetLineEditValue(QLineEdit *lineEdit, const double value)
{
    lineEdit->setText( QString::number(value) );
}

double SettingBase::GetLineEditValue(QLineEdit *lineEdit)
{
    return lineEdit->text().toDouble();
}
