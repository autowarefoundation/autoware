#-------------------------------------------------
#
# Project created by QtCreator 2014-11-18T22:21:04
#
#-------------------------------------------------

SOURCES += points_to_image.cpp
HEADERS += points_to_image.h

unix{
    INCLUDEPATH += /usr/include
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_core
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_highgui
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_features2d
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_objdetect
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_contrib
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_calib3d
    LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_imgproc

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

    INCLUDEPATH += $$(HOME)/SDK/ROSInterface/include
    CONFIG(debug, debug|release){
        LIBS += -L$$(HOME)/SDK/ROSInterface/lib -lROSInterface_Debug
    }else{
        LIBS += -L$$(HOME)/SDK/ROSInterface/lib -lROSInterface_Release
    }
}

PROJNAME = points_to_image
INSTTYPE = SDK
include(RobotSDK_Main.pri)
