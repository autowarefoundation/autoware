#ifndef RECORD_ACTION_H
#define RECORD_ACTION_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#include <jsk_rviz_plugins/RecordCommand.h>
#endif

class QLineEdit;
class QLabel;
class QPushButton;

namespace jsk_rviz_plugins
{
  class RecordAction: public rviz::Panel
  {
    enum RecordState{
      IDLE = 0,
      RECORD = 1
    };
    Q_OBJECT
    public:
    RecordAction( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

  public Q_SLOTS:

    void setTopic( const QString& topic ) {}

  protected Q_SLOTS:

    void updateTopic() {};

    void commandPlay() {};
    void recordClick();

    void addTopicList(std::string topic_name);

    void OnClickPlayButton(int id);
    void OnClickDeleteButton(int id);

  protected:
    QLineEdit* record_motion_name_editor_;

    QString output_topic_;

    QPushButton* record_interface_button_;

    QComboBox* add_topic_box_;

    QSignalMapper *m_delete_sigmap_;
    QSignalMapper *m_play_sigmap_;

    QVBoxLayout* layout;

    struct motionListLayout{
      int id;
      QHBoxLayout* layout_;
      QPushButton* play_button_;
      QPushButton* remove_button_;
      QLabel* target_name_;
    };

    std::vector<motionListLayout> motion_list_layouts_;

    ros::Publisher pub_;
    ros::NodeHandle nh_;
    RecordState rstate_;
  };

}

#endif
