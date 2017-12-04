#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "tunerBody.h"
#include <QtCore/QString>
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  /* tuner controller setup */
  ui->setupUi(this);
}


MainWindow::~MainWindow()
{
  delete ui;
}


void MainWindow::on_radioButton_green_clicked()
{
  TunerBody::setColor(TunerBody::GREEN);
  return;
}


void MainWindow::on_radioButton_yellow_clicked()
{
  TunerBody::setColor(TunerBody::YELLOW);
  return;
}


void MainWindow::on_radioButton_red_clicked()
{
  TunerBody::setColor(TunerBody::RED);
  return;
}


void MainWindow::on_pushButton_reloadImage_clicked()
{
  TunerBody::setUpdateImage();
  return;
}


void MainWindow::on_pushButton_save_clicked()
{
  QString fileName = QFileDialog::getSaveFileName(this,
                                                  tr("Save tuning result"), "",
                                                  tr("yaml file (*.yaml);;All Files(*)"));
  std::string save_filePath(fileName.toLatin1());

  if (!save_filePath.empty()) {
    TunerBody::saveResult(save_filePath);

    /* show notification */
    QMessageBox msgBox(this);
    QString message = tr("<font color = white> <p>tuning result was saved into </p> <p>\"%1\"</p> <p>successfully</p></font>").arg(fileName);
    msgBox.setWindowTitle(tr("Message"));
    msgBox.setText(message);
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.exec();
  }


}


void MainWindow::on_pushButton_loadSetting_clicked()
{
  QString fileName = QFileDialog::getOpenFileName(this,
                                                  tr("Open setting file"), "",
                                                  tr("yaml file (*.yaml);;All Files(*)"));
  std::string open_filePath(fileName.toLatin1());

  if (!open_filePath.empty()) {
    TunerBody::openSetting(open_filePath);
  }
}


void MainWindow::on_pushButton_exit_clicked()
{
  exit(EXIT_SUCCESS);
}
