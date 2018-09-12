#include "configuration.h"

/* LoadNumberArray中使用assert 注意机器人轴数变化 小心数组越界 */
#include <assert.h>

#include <fstream>

#include <QDir>
#include <QFile>

#include "printf.h"

#define SAVE_STRING_ELEMENT(str)        SaveString(doc, root, #str, str);
#define SAVE_NUMBER_ELEMENT(value)      SaveNumber(doc, root, #value, value);
#define SAVE_NUMBER_ARRAY(pValue,len)   SaveNumberArray(doc, root, #pValue, pValue, len);//pValue is a pointer

#define LOAD_NUMBER_ELEMENT(value)      LoadNumber(e, #value, value);
#define LOAD_STRING_ELEMENT(str)        LoadString(e, #str, str);
#define LOAD_NUMBER_ARRAY(pValue,len)   LoadNumberArray(e, #pValue, pValue, len);//pValue is a pointer

const std::string Configuration::mainFolder = QDir::homePath().toStdString()+QString("/Documents/").toStdString()+RobotParams::robotFolder;

const std::string Configuration::sysFilePath = Configuration::mainFolder + "/system_files/";

const std::string Configuration::softStopFilePath = Configuration::sysFilePath + "softStop.txt";
const std::string Configuration::examsoftStopFilePath = Configuration::sysFilePath + "examsoftStop.txt";
const std::string Configuration::nvhsoftStopFilePath = Configuration::sysFilePath + "nvhsoftStop.txt";

const std::string Configuration::originFilePath = Configuration::sysFilePath + "origin.txt";

const std::string Configuration::logFilePath = Configuration::mainFolder + "/log_files/";
const std::string Configuration::logCurvePath = Configuration::logFilePath + "log_curve/";
const std::string Configuration::logCodePath = Configuration::logFilePath + "log_code/";
const std::string Configuration::logCodeConfPath = Configuration::sysFilePath + "log4qt.properties";

const std::string Configuration::stdFilePath = Configuration::mainFolder + "/stdand_files/";

const std::string Configuration::temparrivalFilePath = Configuration::sysFilePath + "temparrival.txt";

Configuration *Configuration::GetInstance()
{
    static Configuration* instance = NULL;
    if(instance == NULL){
        instance = new Configuration();
    }

    return instance;
}

Configuration::Configuration()
    :arrayPrefix("item"), arrayPrefixLen(arrayPrefix.length())
{
    LoadDefaultConfiguration();
    ReadFromFile();
}

Configuration::~Configuration()
{

}

int Configuration::ReadFromFile()
{
    QFile file( (sysFilePath + carTypeName).c_str() );
    if(!file.open(QFile::ReadOnly | QFile::Text)){
        PRINTF(LOG_ERR, "%s: cannot open %s, reading default configuration...\n", __func__, (sysFilePath + carTypeName).c_str());
        LoadDefaultConfiguration();
        return SaveToFile();
    }

    QDomDocument doc;
    if(!doc.setContent(&file)){
        PRINTF(LOG_WARNING, "%s: QDomDocument cannot set content to %s.\n", __func__, (sysFilePath + carTypeName).c_str());
        file.close();
        return -1;
    }

    QDomElement element = doc.documentElement();
    for(QDomNode n = element.firstChild(); n.isNull()==false; n = n.nextSibling()){
        QDomElement e = n.toElement();
        if(e.isNull() == false){
            LOAD_STRING_ELEMENT(carTypeName);
            LOAD_STRING_ELEMENT(defaultFile);
            LOAD_STRING_ELEMENT(normalPassword);
            LOAD_STRING_ELEMENT(rootPassword);
            LOAD_NUMBER_ELEMENT(translateSpeed);

            LOAD_NUMBER_ARRAY(deathPos, RobotParams::axisNum);
            LOAD_NUMBER_ARRAY(limPos, RobotParams::axisNum);
            LOAD_NUMBER_ELEMENT(brakeThetaAfterGoHome);
            LOAD_NUMBER_ELEMENT(getSpeedMethod);
            LOAD_NUMBER_ELEMENT(calcSpeedErrorMethod);
            LOAD_NUMBER_ELEMENT(pedalRobotUsage);
            LOAD_NUMBER_ARRAY(sysControlParams, 7);
            LOAD_NUMBER_ARRAY(sysControlParamsWltc, 7);
            LOAD_NUMBER_ELEMENT(pedalStartTimeS);

            LOAD_NUMBER_ELEMENT(ifManualShift);

            LOAD_NUMBER_ARRAY(angleErr_M, 3);
            LOAD_NUMBER_ARRAY(angleErr_A, 3);
            LOAD_NUMBER_ELEMENT(angleErr_P);
            LOAD_NUMBER_ARRAY(shiftAxisAngles1, 11);
            LOAD_NUMBER_ARRAY(shiftAxisAngles2, 11);
            LOAD_NUMBER_ELEMENT(ifExistSixShift);
            LOAD_NUMBER_ELEMENT(ifExistBackShift);

            LOAD_NUMBER_ARRAY(clutchAngles, 2);
            LOAD_NUMBER_ELEMENT(clutchUpSpeed);
            LOAD_NUMBER_ELEMENT(clutchUpSpeedAtDeparture);
            LOAD_NUMBER_ARRAY(pedalRecoveryPercent, 2);
            LOAD_NUMBER_ELEMENT(ifAutoRecordMidN);

            LOAD_NUMBER_ARRAY(curveMotionSpeed, 3);
            LOAD_NUMBER_ELEMENT(startAccAngleValue); 

            LOAD_NUMBER_ELEMENT(accMotionAtClutchReleasing);
        }
    }
    file.close();
    return 0;
}

int Configuration::SaveToFile()
{
    QDomDocument doc;
    QDomNode instruction = doc.createProcessingInstruction("xml","version=\"1.0\" encoding=\"UTF-8\"");
    doc.insertBefore(instruction, doc.firstChild());

    QDomElement root = doc.createElement("Settings");

    SAVE_STRING_ELEMENT(carTypeName);
    SAVE_STRING_ELEMENT(defaultFile);
    SAVE_STRING_ELEMENT(normalPassword);
    SAVE_STRING_ELEMENT(rootPassword);
    SAVE_NUMBER_ELEMENT(translateSpeed);

    SAVE_NUMBER_ARRAY(deathPos, RobotParams::axisNum);
    SAVE_NUMBER_ARRAY(limPos, RobotParams::axisNum);
    SAVE_NUMBER_ELEMENT(brakeThetaAfterGoHome);
    SAVE_NUMBER_ELEMENT(getSpeedMethod);
    SAVE_NUMBER_ELEMENT(calcSpeedErrorMethod);
    SAVE_NUMBER_ELEMENT(pedalRobotUsage);
    SAVE_NUMBER_ARRAY(sysControlParams, 7);
    SAVE_NUMBER_ARRAY(sysControlParamsWltc, 7);
    SAVE_NUMBER_ELEMENT(pedalStartTimeS);

    SAVE_NUMBER_ELEMENT(ifManualShift);

    SAVE_NUMBER_ARRAY(angleErr_M, 3);
    SAVE_NUMBER_ARRAY(angleErr_A, 3);
    SAVE_NUMBER_ELEMENT(angleErr_P);
    SAVE_NUMBER_ARRAY(shiftAxisAngles1, 11);
    SAVE_NUMBER_ARRAY(shiftAxisAngles2, 11);
    SAVE_NUMBER_ELEMENT(ifExistSixShift);
    SAVE_NUMBER_ELEMENT(ifExistBackShift);

    SAVE_NUMBER_ARRAY(clutchAngles, 2);
    SAVE_NUMBER_ELEMENT(clutchUpSpeed);
    SAVE_NUMBER_ELEMENT(clutchUpSpeedAtDeparture);
    SAVE_NUMBER_ARRAY(pedalRecoveryPercent, 2);
    SAVE_NUMBER_ELEMENT(ifAutoRecordMidN);

    SAVE_NUMBER_ARRAY(curveMotionSpeed, 3);
    SAVE_NUMBER_ELEMENT(startAccAngleValue);

    SAVE_NUMBER_ELEMENT(accMotionAtClutchReleasing);

    doc.appendChild(root);

    QFile file( (sysFilePath + carTypeName).c_str() );
    if (!file.open(QIODevice::WriteOnly)){
        PRINTF(LOG_INFO, "%s: cannot open %s to write configutation.\n", __func__, (sysFilePath + carTypeName).c_str());
        return -1;
    }
    file.write(doc.toString().toLocal8Bit().data());
    file.close();

    return 0;
}

void Configuration::LoadDefaultConfiguration()
{
    carTypeName = "defaultcar.xml";
    defaultFile = Configuration::mainFolder + "/stdand_files/WLTC_ARM";
    normalPassword = "1";
    rootPassword = "2";
    translateSpeed = 50;

    for(unsigned int i=0; i<RobotParams::axisNum; ++i)
    {
        deathPos[i] = 0.0;
        limPos[i] = 80.0;
    }

    brakeThetaAfterGoHome = 75.0;

    getSpeedMethod = 0;
    calcSpeedErrorMethod = 0;
    pedalRobotUsage = 0;

    sysControlParams[0] = 0.1;
    sysControlParams[1] = 40;
    sysControlParams[2] = 40;
    sysControlParams[3] = 1;
    sysControlParams[4] = 1;
    sysControlParams[5] = 1.2;
    sysControlParams[6] = 0.5;

    sysControlParamsWltc[0] = 50;
    sysControlParamsWltc[1] = 1;
    sysControlParamsWltc[2] = 0;
    sysControlParamsWltc[3] = 1;
    sysControlParamsWltc[4] = 1.5;
    sysControlParamsWltc[5] = 0.8;
    sysControlParamsWltc[6] = 0;

    pedalStartTimeS = 0.0;

    ifManualShift = false;

    angleErr_M[0] = 3.0; angleErr_M[1] = 3.0; angleErr_M[2] = 3.0;
    angleErr_A[0] = 1.0; angleErr_A[1] = 1.0; angleErr_A[2] = 1.0;
    angleErr_P = 10.0;

    for (int i = 0; i<9; ++i)
    {
        shiftAxisAngles1[i] = 0.0;
        shiftAxisAngles2[i] = 0.0;
    }
    ifExistSixShift = false;
    ifExistBackShift = false;

    for (int i = 0; i<2; ++i)
    {
        clutchAngles[i] = 0.0;
    }
    clutchUpSpeed = 0.5;
    clutchUpSpeedAtDeparture = 0.1;
    pedalRecoveryPercent[0] = 100;
    pedalRecoveryPercent[1] = 100;
    ifAutoRecordMidN = false;

    curveMotionSpeed[0] = 0.1;curveMotionSpeed[1] = 0.1;curveMotionSpeed[2] = 0.1;

    startAccAngleValue = 10;
    accMotionAtClutchReleasing = 1;
}

/*********************************Load*********************************/
bool Configuration::FindElement(QDomElement &element, const char *name)
{
    return element.tagName() == QString(name);
}

void Configuration::LoadString(QDomElement &element, const char *name, std::string &str)
{
    if(FindElement(element, name)){
        str = element.attribute(name).toStdString();
    }
}

void Configuration::LoadNumber(QDomElement &element, const char *name, bool &value)
{
    if(FindElement(element, name)){
        value = element.attribute(name).toInt();
    }
}

void Configuration::LoadNumber(QDomElement &element, const char *name, double &value)
{
    if(FindElement(element, name)){
        value = element.attribute(name).toDouble();
    }
}

void Configuration::LoadNumber(QDomElement &element, const char *name, unsigned int &value)
{
    if(FindElement(element, name)){
        value = element.attribute(name).toUInt();
    }
}

void Configuration::LoadNumberArray(QDomElement &element, const char *name, double *pValue, const int len)
{
    if(FindElement(element, name)){
        for(QDomNode childNode = element.firstChild(); childNode.isNull()==false; childNode = childNode.nextSibling()){
            QDomElement childElement = childNode.toElement();
            if(childElement.isNull() == false){
                int index = childElement.tagName().mid(arrayPrefixLen).toInt();//索引
                assert(index < len);//配置文件item个数过多
                pValue[index] = childElement.attribute(childElement.tagName()).toDouble();
            }
        }
    }
}

/*********************************Save*********************************/
void Configuration::SaveString(QDomDocument& doc, QDomElement& root, const char* name, const std::string& str)
{
    QDomElement element = doc.createElement(name);
    element.setAttribute(name, str.c_str());
    root.appendChild(element);
}

template<typename T>
void Configuration::SaveNumber(QDomDocument& doc, QDomElement& root, const char* name, const T& value)
{
    QDomElement element = doc.createElement(name);
    element.setAttribute(name, value);
    root.appendChild(element);
}

template<typename T>
void Configuration::SaveNumberArray(QDomDocument& doc, QDomElement& root, const char* name, const T* pValue, const int len)
{
    QDomElement element = doc.createElement(name);
    for(int i=0; i<len; ++i){
        QString childName = arrayPrefix + QString::number(i);
        QDomElement childElement = doc.createElement(childName);
        childElement.setAttribute(childName, pValue[i]);
        element.appendChild(childElement);
    }
    root.appendChild(element);
}
