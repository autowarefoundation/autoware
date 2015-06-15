#include "mainwindow.h"
#include "mainwindow_moc.cpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
#ifdef DEBUG_GUI
    w.show(); //Don't show GUI window
#endif
    return a.exec();
}
