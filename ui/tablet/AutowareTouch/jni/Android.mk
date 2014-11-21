LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE	:= soundsender
LOCAL_SRC_FILES	:= sound_sender.c
LOCAL_LDLIBS	:= -llog -lGLESv2

include $(BUILD_SHARED_LIBRARY)