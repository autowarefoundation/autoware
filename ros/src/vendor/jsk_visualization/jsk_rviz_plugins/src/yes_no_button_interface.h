#ifndef YES_NO_BUTTON_INTERFACE_H
#define YES_NO_BUTTON_INTERFACE_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <boost/thread.hpp>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#endif

#include <jsk_gui_msgs/YesNo.h>


namespace jsk_rviz_plugins
{

  class YesNoButtonInterface: public rviz::Panel
  {
  Q_OBJECT
  public:
    YesNoButtonInterface(QWidget* parent = 0);

    virtual void onInitialize();
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

  protected Q_SLOTS:
    void respondYes();
    void respondNo();
  protected:
    virtual bool requested(
      jsk_gui_msgs::YesNo::Request& req,
      jsk_gui_msgs::YesNo::Response& res);
    QHBoxLayout* layout_;
    QPushButton* yes_button_;
    QPushButton* no_button_;
    bool yes_;
    bool need_user_input_;
    boost::mutex mutex_;
    ros::ServiceServer yes_no_button_service_;
  };

}  // namespace jsk_rviz_plugins


#endif  // YES_NO_BUTTON_INTERFACE_H
