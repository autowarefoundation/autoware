#include "module_panel.hpp"
#include <QApplication>
#include <QDesktopWidget>
#include <QGridLayout>
#include <QStyle>
#include <QStyleOption>
#include <QPainter>

#include <iostream>
using namespace std;

namespace {

QPushButton* create_push_button(QString title)
{
    auto button = new QPushButton(title);
    button->setEnabled(false);
    button->setCheckable(true);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    return button;
}

}

namespace autoware_launcher_rviz {

ModulePanel::ModulePanel(QWidget* parent) : rviz::Panel(parent)
{
    QRect screen = QApplication::desktop()->screenGeometry();
    int font_size = min(screen.width(), screen.height()) / 50;
    setStyleSheet(QString("* {font-size: %1px; background-color: #FFFFFF;} QPushButton{color: #FFFFFF; background-color: #223A70;}").arg(font_size));

    auto layout = new QGridLayout();
    setLayout(layout);

    std::string rootpath = "root/";
    std::vector<const char*> nodetexts = {"Localization", "Detection", "Prediction", "Decision", "Mission", "Motion"};
    std::vector<const char*> nodenames = {"localization", "detection", "prediction", "decision", "mission", "motion"};
    for(size_t i = 0; i < nodenames.size(); ++i)
    {
        auto button = create_push_button(nodetexts[i]);
        buttons[button] = rootpath + nodenames[i];
        layout->addWidget(button, i/3, i%3);
        connect(button, &QPushButton::toggled, this, &ModulePanel::launch_button_toggled);
    }

    socket = new QTcpSocket(this);
    connect(socket, &QTcpSocket::connected,    this, &ModulePanel::server_connected   );
    connect(socket, &QTcpSocket::disconnected, this, &ModulePanel::server_disconnected);
    connect(socket, &QTcpSocket::readyRead,    this, &ModulePanel::server_ready_read  );
    connect(socket, static_cast<void(QTcpSocket::*)(QAbstractSocket::SocketError)>(&QTcpSocket::error), this, &ModulePanel::server_error);
    socket->connectToHost("localhost", 33136);
}

void ModulePanel::paintEvent(QPaintEvent* event)
{
    QStyleOption option;
    option.init(this);
    QPainter painter(this);
    style()->drawPrimitive(QStyle::PE_Widget, &option, &painter, this);
}


void ModulePanel::launch_button_toggled(bool checked)
{
    auto button = static_cast<QPushButton*>(sender());
    QString json = R"({"command":"%1", "path":"%2"})";
    json = json.arg(checked ? "launch" : "terminate").arg(QString::fromStdString(buttons[button]));
    cout << json.toStdString() << endl;
    socket->write(json.toUtf8().append('\0'));
}

void ModulePanel::server_connected()
{
    cout << "connected" << endl;
    for(const auto& pair : buttons)
    {
        pair.first->setEnabled(true);
    }
}

void ModulePanel::server_disconnected()
{
    cout << "disconnected" << endl;
    for(const auto& pair : buttons)
    {
        pair.first->setEnabled(false);
    }
}

void ModulePanel::server_error()
{
    cout << "error" << endl;
    cout << socket->errorString().toStdString() << endl;
}

void ModulePanel::server_ready_read()
{
    cout << "ready_read" << endl;
    cout << socket->readAll().toStdString() << endl;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_launcher_rviz::ModulePanel, rviz::Panel)
