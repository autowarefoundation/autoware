#ifndef DATA_RATE_CHECKER_PLUGIN_H
#define DATA_RATE_CHECKER_PLUGIN_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <rviz/panel.h>
#include <string>
#include <map>

#include <QStringList>
#include <QWidget>
#include <QEvent>

#include "convert_image.h"
#include "ui_data_rate_checker_form.h"
#endif

namespace integrated_viewer
{
  class DummyMsg {
  public:
    static std::string md5;
    static std::string data_type;
    static std::string const &__s_getMD5Sum() { return md5; }
    static std::string const &__s_getDataType() { return data_type; }
    void deserialize(void *) {}
}; // End class DummyMsg

  class DataRateCheckerPlugin: public rviz::Panel {
  Q_OBJECT
  public:
    explicit DataRateCheckerPlugin(QWidget* parent = 0);

    // Override resize event
    virtual void resizeEvent(QResizeEvent *);

  protected:
    // The function to update topic list that can be selected from the UI
    void UpdateTopicList(void);

    // The event filter to catch clicking on combo box
    bool eventFilter(QObject* object, QEvent* event);

    // The Callback functions
    void MessageCallback(const DummyMsg &msg);
    void TimerCallback(const ros::WallTimerEvent &event);

    // The blank topic name
    static const QString kBlankTopic;

    // The ROS node handle
    ros::NodeHandle node_handle_;

    // The ROS subscriber
    ros::Subscriber topic_sub_;

    // The ROS timer
    ros::WallTimer timer_;

    // Variables for message timing
    unsigned int window_size_ = 10000;
    double last_message_time_ = -1;
    std::deque<double> times_;

    // Save and load overrides
    virtual void save(rviz::Config config) const;
    virtual void load(const rviz::Config& config);

  private:
    // The UI components
    Ui::data_rate_checker_form ui_;

    // The behavior definition of the UI
    private Q_SLOTS:
      void on_topic_combo_box__activated(int index);

  }; // End class DataRateCheckerPlugin

} // End namespace integrated_viewer

#endif // DATA_RATE_CHECKER_PLUGIN_H
