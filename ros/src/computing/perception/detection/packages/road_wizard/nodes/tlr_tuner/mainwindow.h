#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    private slots:
        void on_radioButton_green_clicked();
        void on_radioButton_yellow_clicked();
        void on_radioButton_red_clicked();
        void on_pushButton_reloadImage_clicked();
        void on_pushButton_save_clicked();
        void on_pushButton_loadSetting_clicked();
        void on_pushButton_exit_clicked();
};



#endif // MAINWINDOW_H
