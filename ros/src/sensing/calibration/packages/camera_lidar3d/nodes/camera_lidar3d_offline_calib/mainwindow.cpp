/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    CalibrateCameraVelodyneChessboardROS * calibration=new CalibrateCameraVelodyneChessboardROS("/image_raw",1000,10,"/velodyne_points",1000,10,30,cv::Size2f(0.108,0.108),cv::Size2i(8,6));
    ui->tabWidget->addTab(calibration,"Calibration");

    connect(ui->grab,SIGNAL(clicked()),calibration,SLOT(grabCalibDataSlot()));
    connect(ui->remove,SIGNAL(clicked()),calibration,SLOT(removeCalibDataSlot()));
    connect(ui->calibrate,SIGNAL(clicked()),calibration,SLOT(calibrateSensorSlot()));
    connect(ui->load,SIGNAL(clicked()),calibration,SLOT(loadCalibResultSlot()));
    connect(ui->save,SIGNAL(clicked()),calibration,SLOT(saveCalibResultSlot()));
    connect(ui->Project,SIGNAL(clicked()),calibration,SLOT(projectVelodynePointsSlot()));
    connect(ui->refresh,SIGNAL(clicked()),calibration,SLOT(refreshParametersSlot()));
}

MainWindow::~MainWindow()
{
    delete ui;
}
