#!/bin/sh

patch -u -d HEV/ < ./patch/Makefile.patch
patch -u -d HEV/ < ./patch/mainwindow.cpp.patch
patch -u -d HEV/ < ./patch/mainwindow.h.patch
patch -u -d HEV/ < ./patch/HevCnt.cpp.patch
patch -u -d HEV/ < ./patch/HevCnt.h.patch

echo "copying file autoware_config.h"
cp -f ./patch/autoware_config.h ./HEV/autoware_config.h
