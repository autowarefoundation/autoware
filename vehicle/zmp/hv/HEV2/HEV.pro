#-------------------------------------------------
#
# Project created by QtCreator 2012-09-07T13:03:51
#
#-------------------------------------------------

QT       += core gui

TARGET = HEV
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    RingBuffer.cpp \
    HevCnt.cpp \
    GameControl.cpp

HEADERS  += mainwindow.h \
    RingBuffer.h \
    HevCnt.h \
    GameControl.h

FORMS    += mainwindow.ui


unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/zmp/ -lcanif

INCLUDEPATH += $$PWD/../../../../../usr/local/include/zmp
DEPENDPATH += $$PWD/../../../../../usr/local/include/zmp

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/zmp/ -lhevcontrol

INCLUDEPATH += $$PWD/../../../../../usr/local/include/zmp
DEPENDPATH += $$PWD/../../../../../usr/local/include/zmp
