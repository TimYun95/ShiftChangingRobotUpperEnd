#This file is included in .pro

SOURCES += $$PWD/qcustomplot.cpp

HEADERS  += $$PWD/qcustomplot.h

greaterThan(QT_MAJOR_VERSION, 4): QT += printsupport

#to deal with "warning: "QT_HAS_BUILTIN" redefined"
#edit "qendian.h" as "http://www.jsjtt.com/bianchengyuyan/cyuyan/46.html" says
