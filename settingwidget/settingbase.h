#ifndef SETTINGBASE_H
#define SETTINGBASE_H

#include <QLineEdit>

#include "configuration.h"
#include "printf.h"

class SettingBase
{
public:
    SettingBase();
    virtual ~SettingBase();
    int level;
    enum taglevel{//注意声明的顺序 程序中使用了大于/小于的处理
        UnLoggedIn=-1,
        NormalUser,
        RootUser
    };

    virtual void LoadParameters(Configuration &)=0;
    virtual bool StoreParameters(Configuration &)=0;

protected:
    void SetLineEditValue(QLineEdit* lineEdit, const double value);
    double GetLineEditValue(QLineEdit* lineEdit);
};

#endif // SETTINGBASE_H
