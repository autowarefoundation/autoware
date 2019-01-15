#ifndef CANCEL_ACTION_H
#define CANCEL_ACTION_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#endif

class QLineEdit;
class QLabel;
class QPushButton;
//class QSignalMapper;

namespace jsk_rviz_plugins
{
  class CancelAction: public rviz::Panel
    {
      // This class uses Qt slots and is a subclass of QObject, so it needs
      // the Q_OBJECT macro.
Q_OBJECT
  public:
      CancelAction( QWidget* parent = 0 );

      
      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

      public Q_SLOTS:

      void setTopic( const QString& topic ) {};

      protected Q_SLOTS:

      void updateTopic() {};

      void sendTopic();
      void addTopic();
      void initComboBox();

      void addTopicList(std::string topic_name);

      void OnClickDeleteButton(int id);

    protected:
      QString output_topic_;

      QPushButton* add_topic_button_;

      QComboBox* add_topic_box_;

      QPushButton* send_topic_button_;

      QSignalMapper *m_sigmap;

      QVBoxLayout* layout;

      struct topicListLayout{
	int id;
	QHBoxLayout* layout_;
	QPushButton* remove_button_;
	QLabel* topic_name_;
	ros::Publisher publisher_;
      };

      std::vector<topicListLayout> topic_list_layouts_;

      // The ROS publisher for the command velocity.
      ros::Publisher velocity_publisher_;

      // The ROS node handle.
      ros::NodeHandle nh_;

    };

}

#endif // TELEOP_PANEL_H
