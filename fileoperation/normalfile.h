#ifndef NORMALFILE_H
#define NORMALFILE_H

#include <cstddef>
#include <string>

#include <QString>

class NormalFile
{
public:
    NormalFile();

    //获得文件名和后缀
    static std::string GetFileName(const char* filepath);
    //获得文件后缀
    static std::string GetFileExtension(const char* filepath);
    //写入文件
    static int WriteToFile(const char *filePath, const char *content, std::streamsize len);
    //合并文件
    static size_t MergeFiles(const char* filePath1, const char* filePath2, const char* mergedFilepath, const char* connector);
    //文本文件 转换为 Excel文件
    static int Text2Excel(const char* textPath, const char* excelPath);
    //读取文件全部内容
    static int ReadAllContents(const char* filePath, QString& content);
};

#endif // NORMALFILE_H
