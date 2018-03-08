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
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QComboBox *comboBox;
    QDoubleSpinBox *step;
    QDoubleSpinBox *rotation;
    QDoubleSpinBox *minrange;
    QLabel *tc;
    QSpacerItem *horizontalSpacer;
    QLabel *MinFloorID;
    QTableWidget *tableWidget;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QLabel *label;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1159, 827);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        comboBox = new QComboBox(centralWidget);
        comboBox->setObjectName(QStringLiteral("comboBox"));

        horizontalLayout->addWidget(comboBox);

        step = new QDoubleSpinBox(centralWidget);
        step->setObjectName(QStringLiteral("step"));
        step->setMinimum(0.01);
        step->setMaximum(1);
        step->setSingleStep(0.01);
        step->setValue(0.3);

        horizontalLayout->addWidget(step);

        rotation = new QDoubleSpinBox(centralWidget);
        rotation->setObjectName(QStringLiteral("rotation"));
        rotation->setMinimum(-30);
        rotation->setMaximum(30);

        horizontalLayout->addWidget(rotation);

        minrange = new QDoubleSpinBox(centralWidget);
        minrange->setObjectName(QStringLiteral("minrange"));
        minrange->setSingleStep(0.1);

        horizontalLayout->addWidget(minrange);

        tc = new QLabel(centralWidget);
        tc->setObjectName(QStringLiteral("tc"));

        horizontalLayout->addWidget(tc);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        MinFloorID = new QLabel(centralWidget);
        MinFloorID->setObjectName(QStringLiteral("MinFloorID"));

        horizontalLayout->addWidget(MinFloorID);


        verticalLayout->addLayout(horizontalLayout);

        tableWidget = new QTableWidget(centralWidget);
        tableWidget->setObjectName(QStringLiteral("tableWidget"));

        verticalLayout->addWidget(tableWidget);

        MainWindow->setCentralWidget(centralWidget);
        dockWidget = new QDockWidget(MainWindow);
        dockWidget->setObjectName(QStringLiteral("dockWidget"));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        label = new QLabel(dockWidgetContents);
        label->setObjectName(QStringLiteral("label"));
        label->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(label);

        dockWidget->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
#ifndef QT_NO_TOOLTIP
        comboBox->setToolTip(QApplication::translate("MainWindow", "Beam ID", 0));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        step->setToolTip(QApplication::translate("MainWindow", "Height Grid Step (m)", 0));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        rotation->setToolTip(QApplication::translate("MainWindow", "Rotation (degree)", 0));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        minrange->setToolTip(QApplication::translate("MainWindow", "Minimum Range (m)", 0));
#endif // QT_NO_TOOLTIP
        tc->setText(QApplication::translate("MainWindow", "TextLabel", 0));
        MinFloorID->setText(QApplication::translate("MainWindow", "TextLabel", 0));
        label->setText(QApplication::translate("MainWindow", "TextLabel", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
