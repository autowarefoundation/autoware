/*
 rviz Rosbag recording plugin for opensource autonomous driving platform Autoware

 Chenxi TU
*/

#ifndef AUTOWARE_ROSBAG_PLUGIN_H
#define AUTOWARE_ROSBAG_PLUGIN_H

#include <QMainWindow>
#include <QWidget>
#include <QEvent>
#include <QKeyEvent>
#include <QStatusBar>
#include <QImage>
#include <QPainter>
#include <QLabel>
#include <QTabWidget>
#include <QTime>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/panel.h"
#include "rviz/default_plugin/tools/measure_tool.h"
#include "rviz/tool_manager.h"
#include "rviz/default_plugin/tools/point_tool.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <unistd.h>
#include <stdlib.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "../src/core/player.h"
#include "../src/core/recorder.h"

namespace Ui {
class Autoware_Rosbag_Plugin;
}

class Autoware_Rosbag_Plugin : public rviz::Panel
{
    Q_OBJECT

public:


    explicit Autoware_Rosbag_Plugin(QWidget *parent = 0);
    ~Autoware_Rosbag_Plugin();

    typedef struct {
      std::string filename;
      std::vector<std::string> topics;
      int max_duration;
    } RecordParam;

protected:

    boost::scoped_ptr<rosbag_control::Recorder> recorder_;
    boost::scoped_ptr<rosbag_control::RecorderOptions> recorder_opts_;

    int recordReq( RecordParam &recoParam );
    int doRecord( rosbag_control::RecorderOptions &opt );
    void updateRecordTime(ros::Duration record_time_duration_);

protected Q_SLOTS:

    void on_edit_record_filename_textChanged(const QString &arg1);
    void on_button_record_save_clicked();
    void on_button_record_start_clicked();
    void on_button_record_stop_clicked();
    void timeShow();
    void on_botton_topic_refresh_clicked();
    void on_button_record_configure_clicked();

private:

    static const QString DEFAULT_SAVE_PATH;
    static const QString DEFAULT_CONFIGURE_PATH;
    static const int     TIMER_FREQ;

    Ui::Autoware_Rosbag_Plugin *ui;
    QTimer *ui_timer_;

    std::string record_filepath_;
    std::string record_filename_;
    std::vector<std::string> record_topics_;
    bool record_status;

    ros::Time record_time_start_;
    std::vector<std::string> conf_topics_;
};

#endif

