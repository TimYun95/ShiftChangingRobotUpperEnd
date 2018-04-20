#include "normalfile.h"

#include <fstream>
#include <sstream>

#include <QFile>

#include "printf.h"
#include "fileoperation/excel/ExcelBase.h"

NormalFile::NormalFile()
{

}

std::__cxx11::string NormalFile::GetFileName(const char *filepath)
{
    //find '/'(linux) or '\'(windows)
    std::string path(filepath);
    std::string::size_type pos = path.find_last_of("/\\");
    if(pos == std::string::npos){
        PRINTF(LOG_ERR, "%s: error! Cannot find file name from %s\n",__func__, filepath);
        return std::string();
    }
    return path.substr(pos+1);
}

std::__cxx11::string NormalFile::GetFileExtension(const char *filepath)
{
    std::string path(filepath);
    std::string::size_type pos = path.find_last_of(".");
    if(pos == std::string::npos){
        PRINTF(LOG_ERR, "%s: error! Cannot find file extension from %s\n",__func__, filepath);
        return std::string();
    }
    return path.substr(pos+1);
}

int NormalFile::WriteToFile(const char *filePath, const char *content, std::streamsize len)
{
    std::ofstream ofs(filePath, std::fstream::trunc | std::fstream::binary);
    if(ofs.fail()){
        PRINTF(LOG_ERR, "%s: fail to open file=%s\n", __func__, filePath);
        return -1;
    }
    ofs.write(content,len);
    ofs.close();

    return 0;
}

size_t NormalFile::MergeFiles(const char* filePath1, const char* filePath2, const char* mergedFilepath, const char* connector)
{
    //注意 合并的两个文件 各自的第一列应当是相同的时间戳(用于同步文件数据的顺序)
    std::ofstream mergedFile(mergedFilepath, std::fstream::trunc | std::fstream::binary);
    if(mergedFile.fail()){
        PRINTF(LOG_ERR, "%s: fail to open mergedFile=%s\n", __func__, mergedFilepath);
        return -1;
    }
    std::ifstream file1(filePath1, std::fstream::binary);
    if(file1.fail()){
        PRINTF(LOG_ERR, "%s: fail to open file1=%s\n", __func__, filePath1);
        return -1;
    }
    std::ifstream file2(filePath2, std::fstream::binary);
    if(file2.fail()){
        PRINTF(LOG_ERR, "%s: fail to open file2=%s\n", __func__, filePath2);
        return -1;
    }

    size_t rows=0;
    char buf1[1024], buf2[1024];
    int timeStamp1, timeStamp2;
    while(!file1.eof() && !file1.bad() &&
          !file2.eof() && !file2.bad() ){
        //同步时间戳
        file1>>timeStamp1;
        file2>>timeStamp2;
        PRINTF(LOG_DEBUG, "%s: %d...%d\n",__func__,timeStamp1,timeStamp2);
        while(timeStamp1 != timeStamp2){
            if(timeStamp1 < timeStamp2){//read more file1
                file1.getline(buf1,sizeof(buf1));//skip this line
                file1>>timeStamp1;
                if(file1.eof() || file1.bad()){
                    PRINTF(LOG_DEBUG, "%s: file1 ends...\n",__func__);
                    break;
                }
                PRINTF(LOG_DEBUG, "%s: timeStamp1=%d\n",__func__,timeStamp1);
            }else if(timeStamp1 > timeStamp2){//read more file2
                file2.getline(buf2,sizeof(buf2));//skip this line
                file2>>timeStamp2;
                if(file2.eof() || file2.bad()){
                    PRINTF(LOG_DEBUG, "%s: file2 ends...\n",__func__);
                    break;
                }
                PRINTF(LOG_DEBUG, "%s: timeStamp2=%d\n",__func__,timeStamp2);
            }
        }
        //合并相同时间戳的项
        PRINTF(LOG_DEBUG, "%s: merge the same stamp=%d/%d\n",__func__, timeStamp1, timeStamp2);
        file1.getline(buf1,sizeof(buf1));
        file2.getline(buf2,sizeof(buf2));
        mergedFile<< buf1<< connector<< buf2<<"\n";
        ++rows;
    }

    PRINTF(LOG_DEBUG, "%s: exit...\n",__func__);
    file2.close();
    file1.close();
    mergedFile.close();
    return rows;//返回已合并的行数
}

int NormalFile::Text2Excel(const char *textPath, const char *excelPath)
{
    //注意 数据内容 应均为数值

    PRINTF(LOG_DEBUG, "%s: %s...to...%s\n",__func__,textPath,excelPath);
    //open text file
    std::ifstream txt(textPath);
    if(txt.fail()){
        PRINTF(LOG_ERR, "%s: error. open text file=%s\n", __func__, textPath);
        return -1;
    }

    //construct data into ExcelDataArray
    ExcelBase::ExcelDataArray xlsDataArray;
    ExcelBase::ExcelDataRow xlsDataRow;
    char buf[1024];
    while( txt.getline(buf,sizeof(buf)) ){//every row
        std::istringstream iss(buf);
        double val;
        while(iss>>val){//the elements in one row
            xlsDataRow.append(val);
        }
        xlsDataArray.append(xlsDataRow);
        xlsDataRow.clear();
    }
    txt.close();//close stream
    PRINTF(LOG_DEBUG, "%s: construct ExcelDataArray...OK\n", __func__);

    //check excel name
    std::string extension = GetFileExtension(excelPath);
    if(extension.compare("xls") != 0){//只接受xls 不能是xlsx
        PRINTF(LOG_ERR, "%s: errror extension. excel file name=%s(%s)\n", __func__, excelPath, extension.c_str());
        return -1;
    }
    PRINTF(LOG_DEBUG, "%s: extension check...OK\n",__func__);

    //create excel
    ExcelBase::shared_excel_ptr xls(new ExcelBase);
    if( !xls->create(excelPath) ){
        PRINTF(LOG_ERR, "%s: error. create excel file=%s\n", __func__, excelPath);
        return -1;
    }
    PRINTF(LOG_DEBUG, "%s: create excel file(%s)..OK\n", __func__, excelPath);

    //write to excel
    if( !xls->setCurrentSheet(1) ){
        PRINTF(LOG_ERR, "%s: error. setCurrentSheet\n", __func__);
        return -1;
    }
    if( !xls->writeCurrentSheet(xlsDataArray) ){
        PRINTF(LOG_ERR, "%s: error. writeCurrentSheet\n", __func__);
        return -1;
    }
    xls->save();
    PRINTF(LOG_DEBUG, "%s: write excel file...OK\n", __func__);

    return 0;
}

int NormalFile::ReadAllContents(const char *filePath, QString &content)
{
    QFile file(filePath);
    if(file.open(QIODevice::ReadOnly|QIODevice::Text) == false){
        PRINTF(LOG_DEBUG, "%s: cannot open file=%s.\n", __func__, filePath);
        return -1;
    }

    content = file.readAll();
    file.close();
    return 0;
}
