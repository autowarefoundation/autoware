#ifndef TRANSFORMABLE_MARKER_OPERATOR_H
#define TRANSFORMABLE_MARKER_OPERATOR_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QtGui>
#include <jsk_rviz_plugins/RequestMarkerOperate.h>

class QLineEdit;
class QPushButton;

namespace jsk_rviz_plugins
{
  class TransformableMarkerOperatorAction: public rviz::Panel
    {
      Q_OBJECT
      public:
      TransformableMarkerOperatorAction( QWidget* parent = 0 );

      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

    protected Q_SLOTS:

      void callRequestMarkerOperateService(jsk_rviz_plugins::RequestMarkerOperate srv);
      void insertBoxService();
      void insertCylinderService();
      void insertTorusService();
      void eraseWithIdService();
      void eraseAllService();
      void eraseFocusService();

    protected:
      QPushButton* insert_box_button_;
      QPushButton* insert_cylinder_button_;
      QPushButton* insert_torus_button_;

      QPushButton* erase_with_id_button_;
      QPushButton* erase_all_button_;
      QPushButton* erase_focus_button_;

      QVBoxLayout* layout;

      QLineEdit* name_editor_;
      QLineEdit* description_editor_;
      QLineEdit* frame_editor_;
      QLineEdit* id_editor_;

      ros::NodeHandle nh_;
    };
}

#endif
