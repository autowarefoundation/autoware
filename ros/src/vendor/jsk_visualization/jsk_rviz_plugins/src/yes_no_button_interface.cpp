#include "yes_no_button_interface.h"
#include <boost/thread.hpp>
#include <rviz/config.h>
#include <ros/package.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>

#include <jsk_gui_msgs/YesNo.h>


namespace jsk_rviz_plugins
{

  YesNoButtonInterface::YesNoButtonInterface(QWidget* parent)
    : rviz::Panel(parent)
  {
    layout_ = new QHBoxLayout;

    yes_button_ = new QPushButton("Yes");
    layout_->addWidget(yes_button_);
    yes_button_->setEnabled(false);

    no_button_ = new QPushButton("No");
    layout_->addWidget(no_button_);
    no_button_->setEnabled(false);

    connect(yes_button_, SIGNAL(clicked()), this, SLOT(respondYes()));
    connect(no_button_, SIGNAL(clicked()), this, SLOT(respondNo()));

    setLayout(layout_);
  }

  void YesNoButtonInterface::onInitialize()
  {
    ros::NodeHandle nh;
    if (!ros::service::exists("/rviz/yes_no_button", /*print_failure_reason*/false)) {
      yes_no_button_service_ = nh.advertiseService(
        "/rviz/yes_no_button",
        &YesNoButtonInterface::requested,
        this);
    }
  }

  bool YesNoButtonInterface::requested(
      jsk_gui_msgs::YesNo::Request& req,
      jsk_gui_msgs::YesNo::Response& res)
  {
    need_user_input_ = true;
    yes_button_->setEnabled(true);
    no_button_->setEnabled(true);
    while (need_user_input_) {
      QApplication::processEvents(QEventLoop::AllEvents, 100);
    }
    yes_button_->setEnabled(false);
    no_button_->setEnabled(false);
    res.yes = yes_;
    return true;
  }

  void YesNoButtonInterface::respondYes()
  {
    boost::mutex::scoped_lock lock(mutex_);
    yes_ = true;
    need_user_input_ = false;
  }

  void YesNoButtonInterface::respondNo()
  {
    boost::mutex::scoped_lock lock(mutex_);
    yes_ = false;
    need_user_input_ = false;
  }

  void YesNoButtonInterface::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }

  void YesNoButtonInterface::load(const rviz::Config& config)
  {
    rviz::Panel::load(config);
  }

}  // namespace jsk_rviz_plugins


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::YesNoButtonInterface, rviz::Panel)
