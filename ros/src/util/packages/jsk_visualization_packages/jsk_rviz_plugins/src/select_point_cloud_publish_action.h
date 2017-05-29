#ifndef SELECT_POINT_CLOUD_PUBLISH_ACTION_H
#define SELECT_POINT_CLOUD_PUBLISH_ACTION_H

#include <ros/ros.h>

#include <rviz/panel.h>
#include <QtGui>

class QLineEdit;
class QLabel;
class QPushButton;
//class QSignalMapper;
class PropertyTreeWidget;


namespace jsk_rviz_plugins
{
  class SelectPointCloudPublishAction: public rviz::Panel
    {
      // This class uses Qt slots and is a subclass of QObject, so it needs
      // the Q_OBJECT macro.
Q_OBJECT
  public:
      SelectPointCloudPublishAction( QWidget* parent = 0 );

      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

      protected Q_SLOTS:

      void publishPointCloud();
    protected:
      QPushButton* publish_pointcloud_button_;

      QVBoxLayout* layout;

      // The ROS publisher for the command velocity.
      ros::Publisher select_pointcloud_publisher_;

      // The ROS node handle.
      ros::NodeHandle nh_;

    };

}

#endif // TELEOP_PANEL_H
