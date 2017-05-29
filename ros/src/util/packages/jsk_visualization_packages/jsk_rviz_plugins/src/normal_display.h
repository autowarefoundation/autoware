// -*- mode: C++ -*-
#ifndef NORMAL_DISPLAY_H
#define NORMAL_DISPLAY_H
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <QColor>

#include <boost/circular_buffer.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <rviz/message_filter_display.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/validate_floats.h>
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include "normal_visual.h"

namespace jsk_rviz_plugins
{

class NormalDisplay: public rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>
{
Q_OBJECT
public:
  NormalDisplay();
  virtual ~NormalDisplay();
  rviz::EnumProperty* style_property_;
  rviz::ColorProperty* color_property_;
  rviz::ColorProperty* min_color_property_;
  rviz::ColorProperty* max_color_property_;
  rviz::FloatProperty* skip_rate_property_;
  rviz::BoolProperty* rainbow_property_;
  rviz::FloatProperty* scale_property_;
  rviz::FloatProperty* alpha_property_;
  float skip_rate_;
  float scale_;
  float alpha_;

  enum ColorTypes{
    POINTS_COLOR,
    FLAT_COLOR,
    DIRECTION_COLOR,
    CURVATURE_COLOR
  };

protected:
  virtual void onInitialize();

  virtual void reset();

  boost::circular_buffer<boost::shared_ptr<NormalVisual> > visuals_;

  // Function to handle an incoming ROS message.
private Q_SLOTS:
  void processMessage( const sensor_msgs::PointCloud2::ConstPtr& msg );
  void updateStyle();
  void updateSkipRate();
  void updateRainbow();
  void updateScale();
  void updateAlpha();
  void getRainbow(float value , float& rf, float& gf, float& bf);
};

}

#endif
