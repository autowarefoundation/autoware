SOURCES += main.cpp\
        mainwindow.cpp
HEADERS  += mainwindow.h
FORMS    += mainwindow.ui

unix{
    INCLUDEPATH += $$(HOME)/SDK/CalibrationToolkit/include
    INCLUDEPATH += $$(HOME)/SDK/ROSInterface/include
    INCLUDEPATH += $$(HOME)/SDK/GLViewer/include
    CONFIG(debug, debug|release){
        LIBS += -L$$(HOME)/SDK/CalibrationToolkit/lib -lCalibrationToolkit_Debug
        LIBS += -L$$(HOME)/SDK/ROSInterface/lib/ -lROSInterface_Debug
        LIBS += -L$$(HOME)/SDK/GLViewer/lib -lGLViewer_Debug
    }else{
        LIBS += -L$$(HOME)/SDK/CalibrationToolkit/lib -lCalibrationToolkit_Release
        LIBS += -L$$(HOME)/SDK/ROSInterface/lib/ -lROSInterface_Release
        LIBS += -L$$(HOME)/SDK/GLViewer/lib -lGLViewer_Release
    }

    INCLUDEPATH += /usr/include
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_core
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_highgui
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_features2d
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_objdetect
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_contrib
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_calib3d
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_imgproc

    INCLUDEPATH += /usr/include/pcl-1.7
    INCLUDEPATH += /usr/include/eigen3
    LIBS += -L/usr/lib -lpcl_common
    LIBS += -L/usr/lib -lpcl_filters
    LIBS += -L/usr/lib -lpcl_search
    LIBS += -L/usr/lib -lpcl_kdtree
    LIBS += -L/usr/lib -lpcl_features

    INCLUDEPATH += /opt/ros/indigo/include
    LIBS += -L/opt/ros/indigo/lib -lroscpp
    LIBS += -L/opt/ros/indigo/lib -lrosconsole
    LIBS += -L/opt/ros/indigo/lib -lroscpp_serialization
    LIBS += -L/opt/ros/indigo/lib -lrostime
    LIBS += -L/opt/ros/indigo/lib -lxmlrpcpp
    LIBS += -L/opt/ros/indigo/lib -lcpp_common
    LIBS += -L/opt/ros/indigo/lib -lrosconsole_log4cxx
    LIBS += -L/opt/ros/indigo/lib -lrosconsole_backend_interface
    LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

    LIBS += -L/usr/lib/x86_64-linux-gnu -lnlopt

    LIBS += -L/usr/lib/i386-linux-gnu -lGLU
}

PROJNAME = CalibrationToolkit
INSTTYPE = APP
include(RobotSDK_Main.pri)
