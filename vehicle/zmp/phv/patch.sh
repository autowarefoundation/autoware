#!/bin/sh

patch -u -d PHV/ < ./patch/Makefile.patch
patch -u -d PHV/ < ./patch/mainwindow.cpp.patch
patch -u -d PHV/ < ./patch/mainwindow.h.patch
patch -u -d PHV/ < ./patch/PhvCnt.cpp.patch
patch -u -d PHV/ < ./patch/PhvCnt.h.patch

echo "copying file autoware_config.h"
cp -f ./patch/autoware_config.h ./PHV/autoware_config.h
