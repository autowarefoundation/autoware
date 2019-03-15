/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 rviz Rosbag recording plugin for opensource autonomous driving platform Autoware

 Chenxi TU
*/

#include "autoware_rosbag_plugin.h"
#include "ui_autoware_rosbag_plugin.h"
#include <QTimer>
#include <QFileDialog>
#include <QSlider>
#include <QCheckBox>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/common.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>


using namespace std;

const QString Autoware_Rosbag_Plugin::DEFAULT_SAVE_PATH = "/home/user/";
const QString Autoware_Rosbag_Plugin::DEFAULT_CONFIGURE_PATH = "/home/user/";
const int     Autoware_Rosbag_Plugin::TIMER_FREQ     = 1000;

Autoware_Rosbag_Plugin::Autoware_Rosbag_Plugin(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Autoware_Rosbag_Plugin)
{
  record_filepath_.clear();
  record_filename_.clear();
  split_duration = 0;
  split_size = 0;

  ui->setupUi(this);
  ui_timer_ = new QTimer();
  ui_timer_->setInterval(TIMER_FREQ);
  QObject::connect(ui_timer_, SIGNAL(timeout()), this, SLOT(timeShow()));

  ui->button_record_start->setDisabled(true);
  ui->button_record_stop->setDisabled(true);
}

Autoware_Rosbag_Plugin::~Autoware_Rosbag_Plugin()
{
  delete ui;
  delete ui_timer_;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Autoware_Rosbag_Plugin, rviz::Panel)

void Autoware_Rosbag_Plugin::on_edit_record_filename_textChanged(const QString &arg1)
{
  if (!arg1.isEmpty())
  {
    record_filename_ = arg1.toStdString();
    ui->button_record_start->setEnabled(true);
  }

}

void Autoware_Rosbag_Plugin::on_button_record_save_clicked()
{
  /* set default filename */
  QDateTime now = QDateTime::currentDateTime();
  QString timestamp = now.toString(QLatin1String("yyyyMMddhhmmss"));
  QString initial_filename = "autoware-" + timestamp + ".bag";

  /* open dialog */
  QString pathInfo =
      QFileDialog::getSaveFileName(
              this,
              tr("path of saving bag"), DEFAULT_SAVE_PATH + initial_filename);

  if (!pathInfo.isEmpty())
  {
    QString filepath, filename, strTemp;

    strTemp = pathInfo;
    int idx = strTemp.lastIndexOf('/');

    if( idx != -1 )
    {
      filepath = strTemp.left(idx+1);
      filename = strTemp.remove(0, idx+1);
    }
    else
    {
      ROS_ERROR("Invalid Path for recording!!");
      return;
    }

    idx = filename.lastIndexOf(".bag");
    if(idx == -1)
    {
      filename.append(".bag");
    }

    record_filepath_ = filepath.toStdString();
    ui->edit_record_filename->setText(filepath + filename);

  }
}

void Autoware_Rosbag_Plugin::on_button_record_start_clicked()
{
  RecordParam recoParam;

  record_topics_.clear();

  /* Set topics parameter from checkboxes */
  QList<QCheckBox *> list_checkboxes = ui->scrollAreaWidgetContents->findChildren<QCheckBox *>();
  string topic_name;
  for (int i = 0; i < list_checkboxes.size(); ++i) {
      if (list_checkboxes.at(i)->isChecked())
      {
        topic_name = list_checkboxes.at(i)->text().toStdString();
        if (topic_name.find("!") == string::npos)
          record_topics_.push_back(topic_name);
        else
          record_topics_.push_back(topic_name.substr(0, topic_name.length() - strlen("(Not published!!)")));
      }
  }

  std::vector<std::string>::iterator ite = record_topics_.begin();
  while( ite != record_topics_.end() )
  {
    recoParam.topics.push_back(*ite);
    ite++;
  }

  /* Set filename parameter */
  recoParam.filename = record_filename_;

  /* Caculate max split size */
  if (split_size != 0 && ui->checkBox_split_size->isChecked())
    recoParam.max_size = ceil(split_size * 1000 * 1000 * 1000);
  else
    recoParam.max_size = 0;

  /* Caculate max split size */
  if (split_duration != 0 && ui->checkBox_split_duration->isChecked())
    recoParam.max_duration = ceil(split_duration * 60);
  else
    recoParam.max_duration = -1.0;

  /* Start record */
  recordReq(recoParam);

  /*Start Timer*/
  ui_timer_->start();

  ui->button_record_save->setDisabled(true);
  ui->button_record_stop->setEnabled(true);
  ui->button_record_start->setDisabled(true);
  ui->button_record_configure->setDisabled(true);
}

int Autoware_Rosbag_Plugin::recordReq( RecordParam &recoParam )
{
  recorder_opts_.reset( new rosbag_controller::RecorderOptions() );
  recorder_opts_->prefix = recoParam.filename;
  recorder_opts_->append_date = false;

  /* Default options */
  recorder_opts_->buffer_size   = 1048576 * 256;
  recorder_opts_->chunk_size    = 1024 * 768;
  recorder_opts_->min_space     = 1 * 1073741824ull;
  recorder_opts_->min_space_str = "1G";

  /* Set max split size MB */
  if (recoParam.max_size != 0)
  {
    recorder_opts_->split = true;
    recorder_opts_->max_size = recoParam.max_size;
  }

  /* Set max duration Sec */
  if (recoParam.max_duration != -1.0)
  {
    recorder_opts_->split = true;
    recorder_opts_->max_duration = ros::Duration(recoParam.max_duration);
  }

  /* No specified topics then record all */
  if (recoParam.topics.empty())
  {
    recorder_opts_->record_all = true;
  }
  else
  {
    /* Give specified topics to request */
    std::vector<std::string>::iterator ite = recoParam.topics.begin();
    while (ite < recoParam.topics.end())
    {
      recorder_opts_->topics.push_back( *ite );
      ite++;
    }
  }

  ROS_INFO("%s L.%d - Set Recorder Option", __FUNCTION__, __LINE__);
  ROS_INFO("%s L.%d -   record_all      [%d]", __FUNCTION__, __LINE__, recorder_opts_->record_all);
  ROS_INFO("%s L.%d -   regex           [%d]", __FUNCTION__, __LINE__, recorder_opts_->regex);
  ROS_INFO("%s L.%d -   do_exclude      [%d]", __FUNCTION__, __LINE__, recorder_opts_->do_exclude);
  ROS_INFO("%s L.%d -   quiet           [%d]", __FUNCTION__, __LINE__, recorder_opts_->quiet);
  ROS_INFO("%s L.%d -   append_date     [%d]", __FUNCTION__, __LINE__, recorder_opts_->append_date);
  ROS_INFO("%s L.%d -   verbose         [%d]", __FUNCTION__, __LINE__, recorder_opts_->verbose);
  ROS_INFO("%s L.%d -   compression     [%d]", __FUNCTION__, __LINE__, recorder_opts_->compression);
  ROS_INFO("%s L.%d -   prefix          [%s]", __FUNCTION__, __LINE__, recorder_opts_->prefix.c_str());
  ROS_INFO("%s L.%d -   name            [%s]", __FUNCTION__, __LINE__, recorder_opts_->name.c_str());
  ROS_INFO("%s L.%d -   buffer_size     [%d]", __FUNCTION__, __LINE__, recorder_opts_->buffer_size);
  ROS_INFO("%s L.%d -   chunk_size      [%d]", __FUNCTION__, __LINE__, recorder_opts_->chunk_size);
  ROS_INFO("%s L.%d -   limit           [%d]", __FUNCTION__, __LINE__, recorder_opts_->limit);
  ROS_INFO("%s L.%d -   split           [%d]", __FUNCTION__, __LINE__, recorder_opts_->split);
  ROS_INFO("%s L.%d -   max_size        [%.2fGB]", __FUNCTION__, __LINE__, recorder_opts_->max_size / 1000.0 / 1000.0 / 1000.0);
  ROS_INFO("%s L.%d -   max_duration    [%.1fmin]", __FUNCTION__, __LINE__, recorder_opts_->max_duration.toSec() / 60.0 );
  ROS_INFO("%s L.%d -   node            [%s]", __FUNCTION__, __LINE__, recorder_opts_->node.c_str() );
  ROS_INFO("%s L.%d -   min_space       [%d]", __FUNCTION__, __LINE__, recorder_opts_->min_space );
  ROS_INFO("%s L.%d -   min_space_str   [%s]", __FUNCTION__, __LINE__, recorder_opts_->min_space_str.c_str() );
  ROS_INFO("%s L.%d -   topic num       [%d]", __FUNCTION__, __LINE__, recorder_opts_->topics.size() );

  for( size_t i=0; i<recorder_opts_->topics.size(); i++ ) {
    ROS_INFO("%s L.%d - [%d] %s", __FUNCTION__, __LINE__, i, recorder_opts_->topics.at(i).c_str() );
  }

  doRecord( *recorder_opts_ );

  return 0;
}

int Autoware_Rosbag_Plugin::doRecord( rosbag_controller::RecorderOptions &opt )
{
  recorder_.reset(new rosbag_controller::Recorder(opt) );
  record_status_ = 1;

  record_time_start_ = ros::Time::now();

  if (recorder_->start() == 0)
  {
//    ROS_INFO("Start now!!!");
  }
  else
  {
    recorder_.reset();
    recorder_opts_.reset();
  }

  return 0;
}

void Autoware_Rosbag_Plugin::on_button_record_stop_clicked()
{
  recorder_->stop();
  recorder_.reset();
  recorder_opts_.reset();

  ui->button_record_start->setEnabled(true);
  ui->button_record_save->setEnabled(true);
  ui->button_record_stop->setDisabled(true);
  ui->button_record_configure->setEnabled(true);
  record_status_ = 0;

  ros::Duration record_time_reset_ = ros::Duration(0);
  Autoware_Rosbag_Plugin::updateRecordTime(record_time_reset_);
}

void Autoware_Rosbag_Plugin::updateRecordTime(ros::Duration record_time_visual)
{

  QString fmt("hh:mm:ss");
  QTime bufTime(0,0,0);

  QTime curTime = bufTime.addSecs(record_time_visual.toSec());
  QString buf = curTime.toString(fmt);

  ui->label_recTime->setText(buf);
}

void Autoware_Rosbag_Plugin::timeShow()
{
  ros::Duration record_time_duration_;
  if (record_status_ == 1)
    record_time_duration_ = ros::Time::now() - record_time_start_;
  else
    record_time_duration_ = ros::Time::now() - ros::Time::now();
  updateRecordTime(record_time_duration_);
}

void Autoware_Rosbag_Plugin::on_botton_topic_refresh_clicked()
{
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  QLayout* layout = ui->scrollAreaWidgetContents->layout();

  /* Delete previous topic scan result */
  if (layout != 0)
  {
    QLayoutItem *item;
    while ((item = layout->takeAt(0)) != 0)
    {
      delete item->widget();
      delete item;
    }
    delete layout;
  }

  QVBoxLayout *lay = new QVBoxLayout(ui->scrollAreaWidgetContents);
  int topic_num = 0;

  /* Scan current topics */
  std::vector<std::string> current_topic;
  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    current_topic.push_back(info.name);
  }

  /* Soft topics */
  std::sort(current_topic.begin(),current_topic.end());

  /* Build checkboxs for configure file */
  if (!conf_topics_.empty())
  {
    std::vector<std::string>::iterator ite = conf_topics_.begin();
    while (ite != conf_topics_.end())
    {
      if (std::find(current_topic.begin(), current_topic.end(), *ite) == current_topic.end())
      {
        QCheckBox *dynamic = new QCheckBox(QString::fromStdString(*ite + "(Not published!!)"));
        dynamic->setChecked (true);
        dynamic->setStyleSheet("color: red;"
                               "background-color: yellow;");
        lay->addWidget(dynamic);
        topic_num++;
        ite++;
      }
      else
      {
        QCheckBox *dynamic = new QCheckBox(QString::fromStdString(*ite));
        dynamic->setChecked (true);
        lay->addWidget(dynamic);
        topic_num++;
        ite++;
      }
    }
  }

  /* Build checkboxs for current topics */
  std::vector<std::string>::iterator ite = current_topic.begin();
  while (ite != current_topic.end())
  {
    if (std::find(conf_topics_.begin(), conf_topics_.end(), *ite) == conf_topics_.end())
    {
      QCheckBox *dynamic = new QCheckBox(QString::fromStdString(*ite));
      dynamic->setChecked (false);
      lay->addWidget(dynamic);
      topic_num++;
    }
    ite++;
  }
  ui->scrollAreaWidgetContents->setLayout(lay);
}

void Autoware_Rosbag_Plugin::on_button_record_configure_clicked()
{
  /* open dialog */
  QString configureInfo =
      QFileDialog::getOpenFileName(
              this,
              tr("path of record configure file"),
              DEFAULT_CONFIGURE_PATH);

  if (!configureInfo.isEmpty())
  {
    QString filepath, filename, strTemp;

    strTemp = configureInfo;
    int idx = strTemp.lastIndexOf('/');

    if (idx != -1)
    {
      filepath = strTemp.left(idx+1);
      filename = strTemp.remove(0, idx+1);
    }
    else
    {
      ROS_ERROR("Invalid Path for configure file!!");
      return;
    }

    idx = filename.lastIndexOf(".yaml");
    if (idx == -1)
    {
      ROS_ERROR("Need .yaml file!!");
      return;
    }

    ui->edit_record_configure->setText(filename);

    conf_topics_.clear();

    /* read configure file */
    try
    {
      YAML::Node conf = YAML::LoadFile(filepath.toStdString() + filename.toStdString());
      conf_topics_ = conf["topics"].as<std::vector<std::string> >();
    }
    catch(YAML::Exception exception)
    {
      ROS_ERROR(exception.what());
    }
  }
}

void Autoware_Rosbag_Plugin::on_checkBox_split_size_stateChanged(int arg1)
{
  if (ui->checkBox_split_size->isChecked())
  {
    ui->checkBox_split_duration->setDisabled(true);
    ui->lineEdit_duration->setDisabled(true);
    ui->lineEdit_size->setEnabled(true);
  }
  else
  {
    ui->checkBox_split_duration->setEnabled(true);
    ui->lineEdit_duration->setDisabled(true);
    ui->lineEdit_size->setDisabled(true);
  }
}

void Autoware_Rosbag_Plugin::on_checkBox_split_duration_stateChanged(int arg1)
{
  if (ui->checkBox_split_duration->isChecked())
  {
    ui->checkBox_split_size->setDisabled(true);
    ui->lineEdit_size->setDisabled(true);
    ui->lineEdit_duration->setEnabled(true);
  }
  else
  {
    ui->checkBox_split_size->setEnabled(true);
    ui->lineEdit_size->setDisabled(true);
    ui->lineEdit_duration->setDisabled(true);
  }
}

void Autoware_Rosbag_Plugin::on_lineEdit_duration_textEdited(const QString &arg1)
{
    split_duration = arg1.toDouble();
}

void Autoware_Rosbag_Plugin::on_lineEdit_size_textEdited(const QString &arg1)
{
    split_size = arg1.toDouble();
}
