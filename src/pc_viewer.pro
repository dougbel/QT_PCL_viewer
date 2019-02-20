#-------------------------------------------------
#
# Project created by QtCreator 2019-01-17T16:21:17
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pc_viewer
TEMPLATE = app


SOURCES += main.cpp\
        viewerwindow.cpp \
    clouddata.cpp \
    cloudqtdata.cpp

HEADERS  += viewerwindow.h \
    clouddata.h \
    cloudqtdata.h

FORMS    += viewerwindow.ui

INCLUDEPATH += /usr/include/pcl-1.7\
         /usr/local/include/vtk-7.0/

DISTFILES += \
    CMakeLists.txt
