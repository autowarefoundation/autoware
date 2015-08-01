#-------------------------------------------------
#
# Project created by QtCreator 2013-11-20T09:44:53
#
#-------------------------------------------------

QT       += core gui

TARGET = PHV
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    RingBuffer.cpp \
    PhvCnt.cpp \
    GameControl.cpp

HEADERS  += mainwindow.h \
    RingBuffer.h \
    PhvCnt.h \
    GameControl.h

FORMS    += mainwindow.ui




unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/zmp/ -lcanif

INCLUDEPATH += $$PWD/../../../../../usr/local/include/zmp
DEPENDPATH += $$PWD/../../../../../usr/local/include/zmp

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/zmp/ -lCarTomoUSB

INCLUDEPATH += $$PWD/../../../../../usr/local/include/zmp
DEPENDPATH += $$PWD/../../../../../usr/local/include/zmp

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/zmp/ -lphvcontrol

INCLUDEPATH += $$PWD/../../../../../usr/local/include/zmp
DEPENDPATH += $$PWD/../../../../../usr/local/include/zmp
