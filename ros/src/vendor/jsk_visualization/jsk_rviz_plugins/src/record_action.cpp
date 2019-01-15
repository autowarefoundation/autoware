#include <stdio.h>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include "record_action.h"

namespace jsk_rviz_plugins
{

  RecordAction::RecordAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    layout = new QVBoxLayout;

    //Text Box and Add Button to add new topic
    QHBoxLayout* motion_record_layout = new QHBoxLayout;
    record_motion_name_editor_ = new QLineEdit;
    record_motion_name_editor_->setPlaceholderText(QString("Motion Name"));
    motion_record_layout->addWidget( record_motion_name_editor_ );

    record_interface_button_ = new QPushButton("Record");
    motion_record_layout->addWidget( record_interface_button_ );

    layout->addLayout( motion_record_layout );
    //End of Text Box and Add Button
    m_play_sigmap_ = new QSignalMapper(this);
    connect(m_play_sigmap_, SIGNAL(mapped(int)),this, SLOT(OnClickPlayButton(int)));

    m_delete_sigmap_ = new QSignalMapper(this);
    connect(m_delete_sigmap_, SIGNAL(mapped(int)),this, SLOT(OnClickDeleteButton(int)));

    setLayout( layout );
    connect( record_interface_button_, SIGNAL( clicked() ), this, SLOT( recordClick() ));
    pub_ = nh_.advertise<jsk_rviz_plugins::RecordCommand>( "/record_command", 1 );
    rstate_ = IDLE;
  }

  void RecordAction::OnClickPlayButton(int id){
    std::vector<motionListLayout>::iterator it = motion_list_layouts_.begin();
    while( it != motion_list_layouts_.end()){
      if(it->id == id){
        jsk_rviz_plugins::RecordCommand msg;
        msg.target = (it->target_name_)->text().toStdString();
        msg.command = jsk_rviz_plugins::RecordCommand::PLAY;
        pub_.publish(msg);
        break;
      }
      ++it;
    }
  }

  void RecordAction::OnClickDeleteButton(int id){
    std::vector<motionListLayout>::iterator it = motion_list_layouts_.begin();
    while( it != motion_list_layouts_.end()){
      if(it->id == id){
        it->target_name_->hide();
        delete it->target_name_;

        it->play_button_->hide();
        delete it->play_button_;

        it->remove_button_->hide();
        delete it->remove_button_;

        delete it->layout_;
        it = motion_list_layouts_.erase( it );
        Q_EMIT configChanged();
      }else{
        ++it;
      }
    }
  }

  void RecordAction::recordClick()
  {
    output_topic_ = record_motion_name_editor_->text();
    if( output_topic_ != "" ){
      addTopicList(output_topic_.toStdString());
    }
    Q_EMIT configChanged();
  }

  void RecordAction::addTopicList(std::string topic_name){
    if(rstate_ == IDLE){
      jsk_rviz_plugins::RecordCommand msg;
      msg.target = topic_name;
      msg.command = jsk_rviz_plugins::RecordCommand::RECORD;
      pub_.publish(msg);
      rstate_ = RECORD;
      record_interface_button_->setText(QString("Stop"));
      record_motion_name_editor_->setDisabled(true);
    }else{
      record_interface_button_->setText(QString("Record"));
      record_motion_name_editor_->setDisabled(false);

      jsk_rviz_plugins::RecordCommand msg;
      msg.target = topic_name;
      msg.command = jsk_rviz_plugins::RecordCommand::RECORD_STOP;
      pub_.publish(msg);
      rstate_ = IDLE;

      motionListLayout tll;
      if(!motion_list_layouts_.empty()){
        motionListLayout lastTll = motion_list_layouts_.back();
        tll.id = lastTll.id + 1;
      }else{
        tll.id = 0;
      }

      tll.layout_ = new QHBoxLayout;
      tll.target_name_ = new QLabel( topic_name.c_str() );
      tll.layout_->addWidget( tll.target_name_ );
      tll.play_button_ = new QPushButton("Play");
      tll.layout_->addWidget( tll.play_button_ );
      tll.remove_button_ = new QPushButton("Delete");
      tll.layout_->addWidget( tll.remove_button_ );
      layout->addLayout(tll.layout_);

      motion_list_layouts_.push_back(tll);

      connect(tll.play_button_, SIGNAL(clicked()), m_play_sigmap_, SLOT(map()));
      m_play_sigmap_->setMapping(tll.play_button_, tll.id);

      connect(tll.remove_button_, SIGNAL(clicked()), m_delete_sigmap_, SLOT(map()));
      m_delete_sigmap_->setMapping(tll.remove_button_, tll.id);
    }
  }

  void RecordAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void RecordAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::RecordAction, rviz::Panel )
