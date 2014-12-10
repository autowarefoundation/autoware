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
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QPushButton *load;
    QPushButton *save;
    QPushButton *refresh;
    QSpacerItem *horizontalSpacer;
    QPushButton *grab;
    QPushButton *remove;
    QPushButton *calibrate;
    QPushButton *Project;
    QTabWidget *tabWidget;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(785, 395);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        load = new QPushButton(centralWidget);
        load->setObjectName(QStringLiteral("load"));

        horizontalLayout->addWidget(load);

        save = new QPushButton(centralWidget);
        save->setObjectName(QStringLiteral("save"));

        horizontalLayout->addWidget(save);

        refresh = new QPushButton(centralWidget);
        refresh->setObjectName(QStringLiteral("refresh"));

        horizontalLayout->addWidget(refresh);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        grab = new QPushButton(centralWidget);
        grab->setObjectName(QStringLiteral("grab"));

        horizontalLayout->addWidget(grab);

        remove = new QPushButton(centralWidget);
        remove->setObjectName(QStringLiteral("remove"));

        horizontalLayout->addWidget(remove);

        calibrate = new QPushButton(centralWidget);
        calibrate->setObjectName(QStringLiteral("calibrate"));

        horizontalLayout->addWidget(calibrate);

        Project = new QPushButton(centralWidget);
        Project->setObjectName(QStringLiteral("Project"));

        horizontalLayout->addWidget(Project);


        verticalLayout->addLayout(horizontalLayout);

        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));

        verticalLayout->addWidget(tabWidget);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        load->setText(QApplication::translate("MainWindow", "Load", 0));
        save->setText(QApplication::translate("MainWindow", "Save", 0));
        refresh->setText(QApplication::translate("MainWindow", "Refresh", 0));
        grab->setText(QApplication::translate("MainWindow", "Grab", 0));
        remove->setText(QApplication::translate("MainWindow", "Remove", 0));
        calibrate->setText(QApplication::translate("MainWindow", "Calibrate", 0));
        Project->setText(QApplication::translate("MainWindow", "Project", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
