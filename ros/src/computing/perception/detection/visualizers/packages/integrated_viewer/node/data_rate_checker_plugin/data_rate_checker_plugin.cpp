#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QString>
#include <QImage>

#include "data_rate_checker_plugin.h"

namespace integrated_viewer {

    const QString DataRateCheckerPlugin::kBlankTopic = "-----";
    std::string DummyMsg::md5 = "*";
    std::string DummyMsg::data_type = "/";

    DataRateCheckerPlugin::DataRateCheckerPlugin(QWidget *parent)
            : rviz::Panel(parent) {

        // Initialize form
        ui_.setupUi(this);

        // Set minimum frequency parameter
        ui_.min_frequency_spin_box_->setMinimum(1);
        ui_.min_frequency_spin_box_->setMaximum(10000);
        ui_.min_frequency_spin_box_->setValue(5);
        ui_.topic_frequency_lcd_->setMinimumWidth(ui_.topic_frequency_lcd_->width()+2);

        UpdateTopicList();

        // If combobox is clicked, topic list will be update
        ui_.topic_combo_box_->installEventFilter(this);
    } // DataRateCheckerPlugin::DataRateCheckerPlugin


    void DataRateCheckerPlugin::UpdateTopicList(void) {
        // The topic list that can be selected from the UI
        QStringList topic_list;

        // The topic name currently chosen
        QString topic_current = ui_.topic_combo_box_->currentText();

        if (topic_current == "") {
            topic_current = kBlankTopic;
            ui_.status_icon_->setStyleSheet("QLabel {color: #b3b3b3;}");
            ui_.status_text_->setText(QString("No topic selected"));
            ui_.topic_frequency_lcd_->setStyleSheet("QLCDNumber {color: #b3b3b3;}");
            ui_.topic_frequency_lcd_->display(0.0);
        }

        // Insert blank topic name to the top of the lists
        topic_list << kBlankTopic;

        // Get all available topic
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);

        // Convert topic names
        for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
            const ros::master::TopicInfo &info = *it;
            const QString topic_name = QString::fromStdString(info.name);
            const QString topic_type = QString::fromStdString(info.datatype);
            topic_list << topic_name;
        }

        // Sort alphabetically
        topic_list.sort();

        // Remove all list items from combo box
        ui_.topic_combo_box_->clear();

        // Set new items to combo box
        ui_.topic_combo_box_->addItems(topic_list);

        ui_.topic_combo_box_->insertSeparator(1);

        // Set last topic as current
        int topic_index = ui_.topic_combo_box_->findText(topic_current);

        if (topic_index != -1) {
            ui_.topic_combo_box_->setCurrentIndex(topic_index);
        }
        else {
          ui_.topic_combo_box_->setCurrentIndex(ui_.topic_combo_box_->findText(kBlankTopic));
          topic_sub_.shutdown();
          timer_.stop();
          ui_.status_icon_->setStyleSheet("QLabel {color: #b3b3b3;}");
          ui_.status_text_->setText(QString("No topic selected"));
          ui_.topic_frequency_lcd_->setStyleSheet("QLCDNumber {color: #b3b3b3;}");
          ui_.topic_frequency_lcd_->display(0.0);
          times_.clear();
          last_message_time_ = -1;
        }
    } // DataRateCheckerPlugin::UpdateTopicList

    // The behavior of combo box
    void DataRateCheckerPlugin::on_topic_combo_box__activated(int index) {
        // Extract selected topic name from combo box
        std::string selected_topic = ui_.topic_combo_box_->itemText(index).toStdString();
        if (selected_topic == kBlankTopic.toStdString() || selected_topic == "") {
            topic_sub_.shutdown();
            timer_.stop();
            ui_.status_icon_->setStyleSheet("QLabel {color: #b3b3b3;}");
            ui_.status_text_->setText(QString("No topic selected"));
            ui_.topic_frequency_lcd_->setStyleSheet("QLCDNumber {color: #b3b3b3;}");
            ui_.topic_frequency_lcd_->display(0.0);
            times_.clear();
            last_message_time_ = -1;
            return;
        }
        times_.clear();
        last_message_time_ = -1;
        // If selected topic is not blank or empty, start callback functions
        topic_sub_ = node_handle_.subscribe(selected_topic,
                                            1000,
                                            &DataRateCheckerPlugin::MessageCallback,
                                            this);

        timer_ = node_handle_.createWallTimer(ros::WallDuration(1.0),
                                              &DataRateCheckerPlugin::TimerCallback,
                                              this);

    } // DataRateCheckerPlugin::on_topic_combo_box__activated

    void DataRateCheckerPlugin::MessageCallback(const DummyMsg &msg) {

      // Record time message arrives
      ros::Time ros_message_time = ros::Time::now();

      if (ros_message_time.is_zero()) {
        times_.clear();
        last_message_time_ = -1;
        return;
      }

      double message_time = ros_message_time.toSec();

      if (last_message_time_ < 0 || last_message_time_ > message_time) {
        times_.clear();
        last_message_time_ = message_time;
        return;
      }
      times_.push_back(message_time - last_message_time_);
      last_message_time_ = message_time;

      if (times_.size() + 1 > window_size_) {
        times_.pop_front();
      }
    } // DataRateCheckerPlugin::MessageCallback

    void DataRateCheckerPlugin::TimerCallback(const ros::WallTimerEvent &event) {

      // Warn if no new messages have arrived
      if (last_message_time_ == -1 || times_.size() == 0) {
        ui_.status_icon_->setStyleSheet("QLabel {color: #ff0000}");
        ui_.status_text_->setText(QString("No incoming messages"));
        ui_.topic_frequency_lcd_->setStyleSheet("QLCDNumber {color: #ff0000;}");
        ui_.topic_frequency_lcd_->display(0.0);

        // Clear the statistics buffer
        times_.clear();
        last_message_time_ = -1;
        return;
      }

      // Check average frame rates
      double avg_delay = std::accumulate(times_.begin(), times_.end(), 0.0) / times_.size();
      double frequency = 1.0 / avg_delay;

      // Warn if message frequency is too low
      if (frequency < (double)(ui_.min_frequency_spin_box_->value())) {
        ui_.status_icon_->setStyleSheet("QLabel {color: #ff0000;}");
        ui_.status_text_->setText(QString("Message rate is too low"));
        ui_.topic_frequency_lcd_->setStyleSheet("QLCDNumber {color: #ff0000;}");
        ui_.topic_frequency_lcd_->display(frequency);
        return;
      }

      // Otherwise, we are good!
      else {
        ui_.status_icon_->setStyleSheet("QLabel {color: #00ff00;}");
        ui_.status_text_->setText(QString ("Message rate is OK"));
        ui_.topic_frequency_lcd_->setStyleSheet("QLCDNumber {color: #00ff00;}");
        ui_.topic_frequency_lcd_->display(frequency);
        return;
      }
    } // DataRateCheckerPlugin::TimerCallback

    void DataRateCheckerPlugin::resizeEvent(QResizeEvent *) {
    } // DataRateCheckerPlugin::resizeEvent

    bool DataRateCheckerPlugin::eventFilter(QObject *object, QEvent *event) {
        if (event->type() == QEvent::MouseButtonPress) {
            // Combo box will update its contents if this filter is applied
            UpdateTopicList();
        }
        return QObject::eventFilter(object, event);
    } // DataRateCheckerPlugin::eventFilter

    void DataRateCheckerPlugin::save(rviz::Config config) const {
      rviz::Panel::save(config);
      config.mapSetValue("Topic", ui_.topic_combo_box_->currentText());
      config.mapSetValue("Min rate", ui_.min_frequency_spin_box_->value());
    } // DataRateCheckerPlugin::save

    void DataRateCheckerPlugin::load(const rviz::Config& config) {
      rviz::Panel::load(config);
      QString topic;
      int min_frequency;
      if(config.mapGetString ("Topic", &topic))
      {
        // Extract selected topic name from combo box
        std::string selected_topic = topic.toStdString();
        if (selected_topic != kBlankTopic.toStdString() && selected_topic != "") {
          UpdateTopicList();
          int topic_index = ui_.topic_combo_box_->findText(topic);
          // If the load topic doesn't exist, load it anyway to wait for the topic to become active
          if (topic_index == -1) {
            QStringList dummy_topic_list;
            dummy_topic_list << topic;
            ui_.topic_combo_box_->addItems(dummy_topic_list);
            topic_index = ui_.topic_combo_box_->findText(topic);
          }
          times_.clear();
          last_message_time_ = -1;
          ui_.topic_combo_box_->setCurrentIndex(topic_index);
          topic_sub_ = node_handle_.subscribe(selected_topic,
                                              1000,
                                              &DataRateCheckerPlugin::MessageCallback,
                                              this);

          timer_ = node_handle_.createWallTimer(ros::WallDuration(1.0),
                                                &DataRateCheckerPlugin::TimerCallback,
                                                this);
        }
      }
      if(config.mapGetInt ("Min rate", &min_frequency)) {
        ui_.min_frequency_spin_box_->setValue(min_frequency);
      }
    } // DataRateCheckerPlugin::load

} // End namespace integrated_viewer

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(integrated_viewer::DataRateCheckerPlugin, rviz::Panel)
