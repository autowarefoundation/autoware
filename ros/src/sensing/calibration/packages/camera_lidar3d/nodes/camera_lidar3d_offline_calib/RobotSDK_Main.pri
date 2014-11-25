INCLUDEPATH += .

equals(INSTTYPE, "SDK") {
    QT += core opengl widgets gui
    TEMPLATE = lib
    CONFIG += qt staticlib
}
equals(INSTTYPE, "APP") {
    QT += core opengl xml widgets gui
    TEMPLATE = app
    CONFIG += qt
    DEFINES -= RobotSDK_ModuleDev
}
equals(INSTTYPE, "MOD") {
    QT += core opengl xml widgets gui
    TEMPLATE = lib
    CONFIG += qt
    DEFINES += RobotSDK_ModuleDev
    include($${PROJNAME}.pri)
}

unix{
    DESTDIR = $$(HOME)/Build/$$INSTTYPE/$$PROJNAME
    MOC_DIR = $$(HOME)/Build/$$INSTTYPE/$$PROJNAME/MOC
    OBJECTS_DIR = $$(HOME)/Build/$$INSTTYPE/$$PROJNAME/OBJ
    UI_DIR = $$(HOME)/Build/$$INSTTYPE/$$PROJNAME/UI    

    equals(INSTTYPE, "SDK") {
        target.path = $$(HOME)/$$INSTTYPE/$$PROJNAME/lib
	CONFIG(debug, debug|release){
            TARGET = $${PROJNAME}_Debug
        }
        else {
            TARGET = $${PROJNAME}_Release
        }
        INSTALL_PREFIX = $$(HOME)/$$INSTTYPE/$$PROJNAME/include
        INSTALL_HEADERS = $$HEADERS
        include(RobotSDK_Install.pri)
    }
    equals(INSTTYPE, "APP") {
        INCLUDEPATH += $$(HOME)/SDK/RobotSDK/Kernel/include
        INCLUDEPATH += $$(HOME)/SDK/RobotSDK/Module/include
	
        target.path = $$(HOME)/$$INSTTYPE/$$PROJNAME
	CONFIG(debug, debug|release){
            TARGET = $${PROJNAME}_Debug
            LIBS += -L$$(HOME)/SDK/RobotSDK/Kernel/lib/Debug -lKernel
        }
        else {
            TARGET = $${PROJNAME}_Release
            LIBS += -L$$(HOME)/SDK/RobotSDK/Kernel/lib/Release -lKernel
        }
    }
    equals(INSTTYPE, "MOD") {
        INCLUDEPATH += $$(HOME)/SDK/RobotSDK/Kernel/include
        INCLUDEPATH += $$(HOME)/SDK/RobotSDK/ModuleDev
        INCLUDEPATH += $$(HOME)/SDK/RobotSDK/Module/include	
        
        target.path = $$(HOME)/SDK/RobotSDK/Module/SharedLibrary
        CONFIG(debug, debug|release){
            TARGET = $${PROJNAME}_Debug
            LIBS += -L$$(HOME)/SDK/RobotSDK/Kernel/lib/Debug -lKernel
        }
        else {
            TARGET = $${PROJNAME}_Release
            LIBS += -L$$(HOME)/SDK/RobotSDK/Kernel/lib/Release -lKernel
        }
        INSTALL_PREFIX = $$(HOME)/SDK/RobotSDK/Module/include
        INSTALL_HEADERS = $$HEADERS
        include(RobotSDK_Install.pri)
    }

    INSTALLS += target
}

win32{
    TMPPATH=$$(RobotSDK_Kernel)
    isEmpty(TMPPATH) {
        error($$TMPPATH is not Specified.)
    }
    DESTDIR = $$(RobotSDK_Kernel)/../../../Build/$$INSTTYPE/$$PROJNAME
    MOC_DIR = $$(RobotSDK_Kernel)/../../../Build/$$INSTTYPE/$$PROJNAME/MOC
    OBJECTS_DIR = $$(RobotSDK_Kernel)/../../../Build/$$INSTTYPE/$$PROJNAME/OBJ
    UI_DIR = $$(RobotSDK_Kernel)/../../../Build/$$INSTTYPE/$$PROJNAME/UI  

    equals(INSTTYPE, "SDK") {
        TMPPATH=$$(RobotDep_Include)
        isEmpty(TMPPATH) {
            error($$TMPPATH is not Specified.)
        }
        else{
            INCLUDEPATH += $$split(TMPPATH,;)
        }
		target.path = $$(RobotSDK_Kernel)/../../../$$INSTTYPE/$$PROJNAME/lib
        CONFIG(debug, debug|release){
            TARGET = $${PROJNAME}_Debug
        }
        else {
            TARGET = $${PROJNAME}_Release
        }
        INSTALL_PREFIX = $$(RobotSDK_Kernel)/../../../$$INSTTYPE/$$PROJNAME/include
        INSTALL_HEADERS = $$HEADERS
        include(RobotSDK_Install.pri)
    }
    equals(INSTTYPE, "APP") {    
        TMPPATH=$$(RobotDep_Include)
        isEmpty(TMPPATH) {
            error($$TMPPATH is not Specified.)
        }
        else{
            INCLUDEPATH += $$split(TMPPATH,;)
        }
        TMPPATH=$$(RobotSDK_Kernel)
        isEmpty(TMPPATH) {
            error($$TMPPATH is not Specified.)
        }
        else{
            INCLUDEPATH += $$TMPPATH/include
            INCLUDEPATH += $$TMPPATH/../Module/include
        }

        target.path = $$(RobotSDK_Kernel)/../../../$$INSTTYPE/$$PROJNAME
        CONFIG(debug, debug|release){
            TARGET = $${PROJNAME}_Debug
            LIBS += $$(RobotSDK_Kernel)/lib/Debug/Kernel.lib
        }
        else{
            TARGET = $${PROJNAME}_Release
            LIBS += $$(RobotSDK_Kernel)/lib/Release/Kernel.lib
        }
    }
    equals(INSTTYPE, "MOD") {       
        TMPPATH=$$(RobotDep_Include)
        isEmpty(TMPPATH) {
            error($$TMPPATH is not Specified.)
        }
        else{
            INCLUDEPATH += $$split(TMPPATH,;)
        }
        TMPPATH=$$(RobotSDK_Kernel)
        isEmpty(TMPPATH) {
            error($$TMPPATH is not Specified.)
        }
        else{
            INCLUDEPATH += $$TMPPATH/include
        }
        TMPPATH=$$(RobotSDK_ModuleDev)
        isEmpty(TMPPATH) {
            error($$TMPPATH is not Specified.)
        }
        else{
            INCLUDEPATH += $$TMPPATH
            INCLUDEPATH += $$TMPPATH/../Module/include
        }

        target.path = $$(RobotSDK_Kernel)/../Module/SharedLibrary
        CONFIG(debug, debug|release){
            TARGET = $${PROJNAME}_Debug
            LIBS += $$(RobotSDK_Kernel)/lib/Debug/Kernel.lib
        }
        else{
            TARGET = $${PROJNAME}_Release
            LIBS += $$(RobotSDK_Kernel)/lib/Release/Kernel.lib
        }
        INSTALL_PREFIX = $$(RobotSDK_Kernel)/../Module/include
        INSTALL_HEADERS = $$HEADERS
        include(RobotSDK_Install.pri)
    }

    INSTALLS += target
}
