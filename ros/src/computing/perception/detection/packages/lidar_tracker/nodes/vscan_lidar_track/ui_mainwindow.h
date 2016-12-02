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
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *layout;
    QHBoxLayout *control;
    QPushButton *next;
    QPushButton *stop;
    QCheckBox *trigger;
    QCheckBox *local;
    QSpacerItem *horizontalSpacer;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(639, 594);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        layout = new QVBoxLayout();
        layout->setSpacing(6);
        layout->setObjectName(QStringLiteral("layout"));

        verticalLayout->addLayout(layout);

        control = new QHBoxLayout();
        control->setSpacing(6);
        control->setObjectName(QStringLiteral("control"));
        next = new QPushButton(centralWidget);
        next->setObjectName(QStringLiteral("next"));

        control->addWidget(next);

        stop = new QPushButton(centralWidget);
        stop->setObjectName(QStringLiteral("stop"));

        control->addWidget(stop);

        trigger = new QCheckBox(centralWidget);
        trigger->setObjectName(QStringLiteral("trigger"));
        trigger->setChecked(true);

        control->addWidget(trigger);

        local = new QCheckBox(centralWidget);
        local->setObjectName(QStringLiteral("local"));

        control->addWidget(local);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        control->addItem(horizontalSpacer);


        verticalLayout->addLayout(control);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        next->setText(QApplication::translate("MainWindow", "Next", 0));
        stop->setText(QApplication::translate("MainWindow", "Stop", 0));
        trigger->setText(QApplication::translate("MainWindow", "Auto", 0));
        local->setText(QApplication::translate("MainWindow", "Local", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
