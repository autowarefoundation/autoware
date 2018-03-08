/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
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
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QLineEdit *patternwidth;
    QLabel *label_2;
    QLineEdit *patternheight;
    QFrame *line;
    QLabel *label_3;
    QLineEdit *patterncolumn;
    QLabel *label_4;
    QLineEdit *patternrow;
    QSpacerItem *horizontalSpacer_2;
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

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout_2->addWidget(label);

        patternwidth = new QLineEdit(centralWidget);
        patternwidth->setObjectName(QStringLiteral("patternwidth"));

        horizontalLayout_2->addWidget(patternwidth);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        patternheight = new QLineEdit(centralWidget);
        patternheight->setObjectName(QStringLiteral("patternheight"));

        horizontalLayout_2->addWidget(patternheight);

        line = new QFrame(centralWidget);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        horizontalLayout_2->addWidget(line);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_2->addWidget(label_3);

        patterncolumn = new QLineEdit(centralWidget);
        patterncolumn->setObjectName(QStringLiteral("patterncolumn"));

        horizontalLayout_2->addWidget(patterncolumn);

        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout_2->addWidget(label_4);

        patternrow = new QLineEdit(centralWidget);
        patternrow->setObjectName(QStringLiteral("patternrow"));

        horizontalLayout_2->addWidget(patternrow);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout_2);

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
        label->setText(QApplication::translate("MainWindow", "Pattern Size (m)", 0));
#ifndef QT_NO_TOOLTIP
        patternwidth->setToolTip(QApplication::translate("MainWindow", "width", 0));
#endif // QT_NO_TOOLTIP
        patternwidth->setText(QApplication::translate("MainWindow", "0.035", 0));
        label_2->setText(QApplication::translate("MainWindow", "X", 0));
#ifndef QT_NO_TOOLTIP
        patternheight->setToolTip(QApplication::translate("MainWindow", "height", 0));
#endif // QT_NO_TOOLTIP
        patternheight->setText(QApplication::translate("MainWindow", "0.035", 0));
        label_3->setText(QApplication::translate("MainWindow", "Pattern Number", 0));
#ifndef QT_NO_TOOLTIP
        patterncolumn->setToolTip(QApplication::translate("MainWindow", "column", 0));
#endif // QT_NO_TOOLTIP
        patterncolumn->setText(QApplication::translate("MainWindow", "6", 0));
        label_4->setText(QApplication::translate("MainWindow", "X", 0));
#ifndef QT_NO_TOOLTIP
        patternrow->setToolTip(QApplication::translate("MainWindow", "row", 0));
#endif // QT_NO_TOOLTIP
        patternrow->setText(QApplication::translate("MainWindow", "8", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
