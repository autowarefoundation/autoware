#ifndef OBJECT_FIT_OPERATOR_H
#define OBJECT_FIT_OPERATOR_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QtGlobal>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#include <QToolButton>
#include <QCheckBox>
#include <jsk_rviz_plugins/ObjectFitCommand.h>
#endif

class QLineEdit;
class QToolButton;

namespace jsk_rviz_plugins
{
  class ObjectFitOperatorAction: public rviz::Panel
    {
      Q_OBJECT
      public:
      ObjectFitOperatorAction( QWidget* parent = 0 );

      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

    protected Q_SLOTS:
      void commandFit();
      void commandNear();
      void commandOther();
      void checkBoxChanged(bool state);
      void publishObjectFitOder(int type);

    protected:
      QToolButton* fit_button_;
      QToolButton* near_button_;
      QToolButton* other_button_;
      QCheckBox* check_box_;

      QHBoxLayout* horizontal_layout1_;
      QHBoxLayout* horizontal_layout2_;
      QVBoxLayout* layout;
      bool reverse_;

      ros::NodeHandle nh_;
      ros::Publisher pub_;
    };
}

#endif
