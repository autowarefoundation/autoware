#-------------------------------------------------
#
# Project created by QtCreator 2014-11-20T12:48:46
#
#-------------------------------------------------

QT       += core widgets

QT       -= gui

TARGET = points_to_image
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

unix{
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

    INCLUDEPATH += $$(HOME)/SDK/ROSInterface/include
    CONFIG(debug, debug|release){
        LIBS += -L$$(HOME)/SDK/ROSInterface/lib -lROSInterface_Debug
    }else{
        LIBS += -L$$(HOME)/SDK/ROSInterface/lib -lROSInterface_Release
    }

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


}
