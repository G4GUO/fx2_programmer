#-------------------------------------------------
#
# Project created by QtCreator 2014-04-26T13:48:50
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = fx2programmer
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    fx2.cpp

HEADERS  += mainwindow.h \
    fx2.h

FORMS    += mainwindow.ui

unix{
    CONFIG += link_pkgconfig
    PKGCONFIG += libusb-1.0
}
