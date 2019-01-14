#include "mainwindow.h"
#include <QApplication>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "points2vscan");
  QApplication a(argc, argv);
  MainWindow w;
#ifdef DEBUG_GUI
  w.show();  // Don't show GUI window
#endif
  return a.exec();
}
