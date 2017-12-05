/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QFormLayout *formLayout;
    QHBoxLayout *horizontalLayout;
    QRadioButton *radioButton_green;
    QRadioButton *radioButton_yellow;
    QRadioButton *radioButton_red;
    QLabel *label;
    QGridLayout *gridLayout;
    QPushButton *pushButton_reloadImage;
    QPushButton *pushButton_save;
    QPushButton *pushButton_loadSetting;
    QPushButton *pushButton_exit;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(267, 254);
        MainWindow->setStyleSheet(QStringLiteral("background-color: rgba(110, 123, 139, 230);"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout_2 = new QVBoxLayout(centralWidget);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        formLayout = new QFormLayout();
        formLayout->setSpacing(6);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        radioButton_green = new QRadioButton(centralWidget);
        radioButton_green->setObjectName(QStringLiteral("radioButton_green"));
        radioButton_green->setCursor(QCursor(Qt::PointingHandCursor));
        radioButton_green->setLayoutDirection(Qt::LeftToRight);
        radioButton_green->setStyleSheet(QStringLiteral("color: rgb(255, 255, 255);"));
        radioButton_green->setChecked(true);

        horizontalLayout->addWidget(radioButton_green);

        radioButton_yellow = new QRadioButton(centralWidget);
        radioButton_yellow->setObjectName(QStringLiteral("radioButton_yellow"));
        radioButton_yellow->setCursor(QCursor(Qt::PointingHandCursor));
        radioButton_yellow->setStyleSheet(QStringLiteral("color: rgb(255, 255, 255);"));

        horizontalLayout->addWidget(radioButton_yellow);

        radioButton_red = new QRadioButton(centralWidget);
        radioButton_red->setObjectName(QStringLiteral("radioButton_red"));
        radioButton_red->setCursor(QCursor(Qt::PointingHandCursor));
        radioButton_red->setStyleSheet(QStringLiteral("color: rgb(255, 255, 255);"));

        horizontalLayout->addWidget(radioButton_red);


        formLayout->setLayout(2, QFormLayout::LabelRole, horizontalLayout);

        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setUnderline(false);
        font.setWeight(75);
        label->setFont(font);
        label->setStyleSheet(QStringLiteral("color: rgb(255, 255, 255);"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label);


        verticalLayout->addLayout(formLayout);

        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        pushButton_reloadImage = new QPushButton(centralWidget);
        pushButton_reloadImage->setObjectName(QStringLiteral("pushButton_reloadImage"));
        QFont font1;
        font1.setPointSize(12);
        font1.setBold(true);
        font1.setWeight(75);
        pushButton_reloadImage->setFont(font1);
        pushButton_reloadImage->setStyleSheet(QStringLiteral("color: rgb(255, 255, 255);"));

        gridLayout->addWidget(pushButton_reloadImage, 1, 0, 1, 1);

        pushButton_save = new QPushButton(centralWidget);
        pushButton_save->setObjectName(QStringLiteral("pushButton_save"));
        pushButton_save->setFont(font1);
        pushButton_save->setStyleSheet(QLatin1String("color: rgb(255, 255, 255);\n"
"alternate-background-color: rgb(255, 255, 255);"));

        gridLayout->addWidget(pushButton_save, 3, 0, 1, 1);

        pushButton_loadSetting = new QPushButton(centralWidget);
        pushButton_loadSetting->setObjectName(QStringLiteral("pushButton_loadSetting"));
        pushButton_loadSetting->setFont(font1);
        pushButton_loadSetting->setStyleSheet(QStringLiteral("color: rgb(255, 255, 255);"));

        gridLayout->addWidget(pushButton_loadSetting, 2, 0, 1, 1);

        pushButton_exit = new QPushButton(centralWidget);
        pushButton_exit->setObjectName(QStringLiteral("pushButton_exit"));
        pushButton_exit->setFont(font1);
        pushButton_exit->setStyleSheet(QStringLiteral("color: rgb(255, 255, 255);"));

        gridLayout->addWidget(pushButton_exit, 4, 0, 1, 1);


        verticalLayout->addLayout(gridLayout);


        verticalLayout_2->addLayout(verticalLayout);

        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Tuner controller", 0));
        radioButton_green->setText(QApplication::translate("MainWindow", "GREEN", 0));
        radioButton_yellow->setText(QApplication::translate("MainWindow", "YELLOW", 0));
        radioButton_red->setText(QApplication::translate("MainWindow", "RED", 0));
        label->setText(QApplication::translate("MainWindow", "Tuning color", 0));
        pushButton_reloadImage->setText(QApplication::translate("MainWindow", "Reload Image", 0));
        pushButton_save->setText(QApplication::translate("MainWindow", "Save", 0));
        pushButton_loadSetting->setText(QApplication::translate("MainWindow", "Load setting", 0));
        pushButton_exit->setText(QApplication::translate("MainWindow", "Exit", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
