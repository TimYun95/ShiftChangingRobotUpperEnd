#ifndef FILEASSISTANTFUNC_H
#define FILEASSISTANTFUNC_H

#include <string>

class FileAssistantFunc
{
public:
    FileAssistantFunc() = delete;

    static std::string ReadFileContent(const std::string &filePath);
};

#endif // FILEASSISTANTFUNC_H
