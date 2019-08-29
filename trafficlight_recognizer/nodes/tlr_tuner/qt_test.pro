#-------------------------------------------------
#
# Project created by QtCreator 2015-06-22T11:47:14
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = qt_test
TEMPLATE = app

INCLUDEPATH += $$_PRO_FILE_PWD_/../../include/trafficlight_recognizer/tlr_tuner

SOURCES += main.cpp\
        mainwindow.cpp \
    tunerBody.cpp

HEADERS  += mainwindow.h \
    tunerBody.h

FORMS    += mainwindow.ui
