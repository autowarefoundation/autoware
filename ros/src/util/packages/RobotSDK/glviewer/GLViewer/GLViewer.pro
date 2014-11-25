HEADERS += ./glviewer.h
SOURCES += ./glviewer.cpp

unix{
    INCLUDEPATH += /usr/include/eigen3
}

PROJNAME = GLViewer
INSTTYPE = SDK
include(RobotSDK_Main.pri)
