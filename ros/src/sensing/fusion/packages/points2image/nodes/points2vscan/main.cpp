#include "mainwindow.h"
#include "mainwindow_moc.cpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    //w.show(); //Don't show GUI window

    return a.exec();
}
