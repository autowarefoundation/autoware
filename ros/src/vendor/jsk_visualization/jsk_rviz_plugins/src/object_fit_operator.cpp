#include <QLineEdit>
#include <QToolButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QCheckBox>
#include <QLabel>
#include <ros/package.h>

#include "object_fit_operator.h"

using namespace rviz;
namespace jsk_rviz_plugins
{
  ObjectFitOperatorAction::ObjectFitOperatorAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    layout = new QVBoxLayout;

    horizontal_layout1_ = new QHBoxLayout();
    horizontal_layout2_ = new QHBoxLayout();

    //Button to send cancel topic
    std::string fit_button_name, reverse_fit_button_name, near_button_name, other_button_name;
    nh_.param<std::string>("/object_fit_icon", fit_button_name, ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/fit.jpg"));
    nh_.param<std::string>("/object_near_icon", near_button_name, ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/near.jpg"));
    nh_.param<std::string>("/object_other_icon", other_button_name, ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/other.jpg"));

    QSize iconSize(150, 150);
    fit_button_ = new QToolButton();
    fit_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    fit_button_->setIcon(QIcon(QPixmap(QString(fit_button_name.c_str()))));
    fit_button_->setText("Onto");
    fit_button_->setIconSize(iconSize);
    horizontal_layout1_->addWidget( fit_button_ );

    check_box_ = new QCheckBox(QString("Reverse"));
    horizontal_layout1_->addWidget(check_box_);

    near_button_ = new QToolButton();
    near_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    near_button_->setIcon(QIcon(QPixmap(QString(near_button_name.c_str()))));
    near_button_->setIconSize(iconSize);
    near_button_->setText("Parallel");
    horizontal_layout2_->addWidget( near_button_ );

    other_button_ = new QToolButton();
    other_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    other_button_->setIcon(QIcon(QPixmap(QString(other_button_name.c_str()))));
    other_button_->setText("Perpendicular");
    other_button_->setIconSize(iconSize);
    horizontal_layout2_->addWidget( other_button_ );

    layout->addLayout(horizontal_layout1_);
    layout->addLayout(horizontal_layout2_);
    setLayout( layout );

    connect( fit_button_, SIGNAL( clicked() ), this, SLOT( commandFit()));
    connect( check_box_, SIGNAL(clicked(bool)), this, SLOT(checkBoxChanged(bool)));
    connect( near_button_, SIGNAL( clicked() ), this, SLOT( commandNear()));
    connect( other_button_, SIGNAL( clicked() ), this, SLOT( commandOther()));

    pub_ = nh_.advertise<jsk_rviz_plugins::ObjectFitCommand>( "/object_fit_command", 1 );
  }

  void ObjectFitOperatorAction::checkBoxChanged(bool state){
    reverse_ = state;
  }

  void ObjectFitOperatorAction::commandFit(){
    if(reverse_)
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::REVERSE_FIT);
    else
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::FIT);
  }

  void ObjectFitOperatorAction::commandNear(){
    if(reverse_)
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::REVERSE_NEAR);
    else
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::NEAR);
  }

  void ObjectFitOperatorAction::commandOther(){
    if(reverse_)
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::REVERSE_OTHER);
    else
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::OTHER);
  }

  void ObjectFitOperatorAction::publishObjectFitOder(int type){
    jsk_rviz_plugins::ObjectFitCommand msg;
    msg.command = type;
    pub_.publish(msg);
  }

  void ObjectFitOperatorAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void ObjectFitOperatorAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::ObjectFitOperatorAction, rviz::Panel )
