#include "mainwindow.h"
#include "tunerBody.h"
#include <QApplication>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  TunerBody tuner;

  w.show();
  tuner.launch();

  return a.exec();
}
