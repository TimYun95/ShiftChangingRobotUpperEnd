#include "fileassistantfunc.h"

#include <QFile>

std::__cxx11::string FileAssistantFunc::ReadFileContent(const std::__cxx11::string &filePath)
{
    std::string fileContent;

    QFile file( QString::fromStdString(filePath) );
    if(file.open(QIODevice::ReadOnly|QIODevice::Text)){
        fileContent = file.readAll().toStdString();
        file.close();
    }

    return fileContent;
}
