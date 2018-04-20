//from:
//https://github.com/czyt1988/czyBlog/tree/master/tech/fastReadExcel

/*****************attention**********************/
//this class can only be used for .xls
//.xlsx is out of its ability.

//usage:
//1) write
//shared_excel_ptr---create---setCurrentSheet---writeCurrentSheet---save
/*  ExcelBase::shared_excel_ptr xls(new ExcelBase);
    xls->create(excelPath);
    xls->setCurrentSheet(1);
    xls->writeCurrentSheet(xlsDataArray);
    xls->save();*/
//2) read
//shared_excel_ptr---open---setCurrentSheet---ExcelDataArray---readAll---close
/*  ExcelBase::shared_excel_ptr xls(new ExcelBase);
    xls->open(excelPath);
    xls->setCurrentSheet(1);
    ExcelBase::ExcelDataArray m_datas;
    xls->readAll(m_datas);
    xls->close();

    for(int i=0; i<m_datas.size(); ++i){
        for(int j=0; j<m_datas[i].size(); ++j){
            PRINTF("[%d,%d]=%f\n",i, j,
                   m_datas[i][j].toDouble());
        }
    }*/

//basic operations of excel
#ifndef EXCELREADER_H
#define EXCELREADER_H

#include <QObject>
#include <QAxObject>
#include <QString>
#include <QStringList>
#include <QVariant>

class ExcelBasePrivate;

class ExcelBase : public QObject
{
public:
    ExcelBase(QObject* par=NULL);
    ~ExcelBase();
    typedef QScopedPointer<ExcelBase> shared_excel_ptr;//excel对象的智能指针(自动管理new出来的内存)
    typedef QList<QList<QVariant> >   ExcelDataArray;//excel的数据(obj[i][j]访问该excel的对应位置的元素)
    typedef QList<QVariant> ExcelDataRow;//excel的一行数据(obj[i]访问该行的对应位置的元素)

private:
    Q_DISABLE_COPY(ExcelBase)
    Q_DECLARE_PRIVATE(ExcelBase)
    ExcelBasePrivate* const d_ptr;

public:
    /// @brief 设置方向的常数
    enum Alignment{
        xlTop    = -4160, ///< 靠上
        xlLeft   = -4131, ///< 靠左
        xlRight  = -4152, ///< 靠右
        xlCenter = -4108, ///< 居中
        xlBottom = -4107  ///< 靠下
    };

    /// @brief 创建一个Microsoft Excel文件
    bool create(const QString& filename = QString());
    /// @brief 打开一个Microsoft Excel文件
    bool open(const QString& filename = QString());
    /// @brief 另存Microsoft Excel文件
    void saveAs(const QString& filename);
    void save();
    /// @brief 关闭Microsoft Excel文件
    void close();
    /// @brief 踢出当前打开的 Microsoft Excel<br>
    /// @brief 放弃此对象对该 Excel 的控制权<br>
    /// @brief Excel 文件仍保持打开，但丧失了控制权
    void kick();

    /// @brief 设置当前打开的 Excel 是否可见
    void setVisible(bool value);

    /// @brief 设置 Excel 文档的标题
    void setCaption(const QString& value);

    /// @brief 新建一本 Excel 文档
    bool addBook();

    /// @brief 返回当前 Excel 的 Sheet 数量
    int sheetCount();

    /// @brief 返回当前打开的 Excel 的 Sheet 名
    QString currentSheetName();
    /// @brief 返回当前打开的 Excel 全部 Sheet 名
    QStringList sheetNames();

    /// @brief 返回当前 Sheet.
    bool currentSheet();
    /// @brief 设置并指定当前 Sheet.
    /// @param [in] 当前 Sheet 索引，从 1 开始
    bool setCurrentSheet(int index);

    /// @brief 读取单元格 Sheet 的内容
    /// @param [in] row 行号，从 1 开始
    /// @param [in] col 列号，从 1 开始
    /// @return 返回指定单元格的内容
    QVariant read(int row, int col);
    /// @brief 读取单元格 Sheet 的内容
    /// @param [in] row 行号，从 1 开始
    /// @param [in] col 列号，从 1 开始
    /// @return 返回指定单元格的内容
    inline QVariant cell(int row, int col) { return read(row, col); }

    /// @brief 写入单元格 Sheet 的内容
    /// @param [in] row 行号，从 1 开始
    /// @param [in] col 列号，从 1 开始
    /// @param [in] value 准备写入的内容
    void write(int row, int col, const QVariant& value);

    /// @brief 获取单元格 Sheet 的对齐方式
    /// @param [in] row 行号，从 1 开始
    /// @param [in] col 列号，从 1 开始
    /// @param [in] format 该单元格的格式
    void cellFormat(int row, int col, const QString& format);
    /// @brief 写入单元格 Sheet 的对齐方式
    /// @param [in] row 行号，从 1 开始
    /// @param [in] col 列号，从 1 开始
    /// @param [in] hAlign 水平方向格式
    /// @param [in] vAlign 竖直方向格式
    void cellAlign(int row, int col, Alignment hAlign, Alignment vAlign);

    /// @brief 获取有效区域信息
    /// @see rowStart() const
    /// @see rowEnd() const
    /// @see colStart() const
    /// @see colEnd() const
    bool usedRange(int& rowStart, int& colStart, int &rowEnd, int &colEnd);

    /// \brief 读取整个sheet
    /// \return
    QVariant readAll();
    /// \brief 读取整个sheet的数据，并放置到cells中
    /// \param cells
    void readAll(QList<QList<QVariant> > &cells);

    /// \brief 写入一个表格内容
    /// \param cells
    /// \return 成功写入返回true
    /// \see readAllSheet
    bool writeCurrentSheet(const QList<QList<QVariant> > &cells);

    /// \brief 把列数转换为excel的字母列号
    /// \param data 大于0的数
    /// \return 字母列号，如1->A 26->Z 27 AA
    static void convertToColName(int data, QString &res);
    /// \brief 数字转换为26字母
    /// 1->A 26->Z
    /// \param data
    /// \return
    static QString to26AlphabetString(int data);
    /// \brief QList<QList<QVariant> >转换为QVariant
    /// \param cells
    /// \return
    static void castListListVariant2Variant(const QList<QList<QVariant> > &cells,QVariant& res);
    /// \brief 把QVariant转为QList<QList<QVariant> >
    /// \param var
    /// \param res
    static void castVariant2ListListVariant(const QVariant& var,QList<QList<QVariant> > &res);
};

#endif // EXCELREADER_H
