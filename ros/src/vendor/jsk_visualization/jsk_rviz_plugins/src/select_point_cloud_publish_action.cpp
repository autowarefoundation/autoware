#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QColor>
#include <QFont>

#include "rviz/properties/property_tree_widget.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"

#include "rviz/config.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/status_list.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/color_property.h"

#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>

#include "select_point_cloud_publish_action.h"
#include "ros/time.h"

using namespace rviz;

namespace jsk_rviz_plugins
{

  SelectPointCloudPublishAction::SelectPointCloudPublishAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    select_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("selected_pointcloud", 1);
    layout = new QVBoxLayout;

    //Button to send cancel topic
    publish_pointcloud_button_ = new QPushButton("SelectPointCloudPublish Action");
    layout->addWidget( publish_pointcloud_button_ );

    setLayout( layout );

    connect( publish_pointcloud_button_, SIGNAL( clicked() ), this, SLOT( publishPointCloud ()));
  }

  void SelectPointCloudPublishAction::publishPointCloud(){
    PropertyTreeModel* model_ =vis_manager_->getSelectionManager()->getPropertyModel();
    int num_children = model_->rowCount();
    if( num_children > 0 )
      {
        ROS_INFO("num > %d!", num_children);
        sensor_msgs::PointCloud2 pc2;
        pc2.header.stamp = ros::Time::now();
        pc2.header.frame_id = "camera_depth_optical_frame";
        pc2.height = 1;
        pc2.width  = num_children;

        pc2.fields.resize(4);
        pc2.fields[0].name = "x";
        pc2.fields[1].name = "y";
        pc2.fields[2].name = "z";
        pc2.fields[3].name = "rgb";
        pc2.fields[0].offset = 0;
        pc2.fields[1].offset = 4;
        pc2.fields[2].offset = 8;
        pc2.fields[3].offset = 12;
        pc2.fields[0].count =  pc2.fields[1].count =  pc2.fields[2].count =  pc2.fields[3].count = 1;
        pc2.fields[0].datatype =  pc2.fields[1].datatype =  pc2.fields[2].datatype =  pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

        pc2.data.resize(num_children * 4 * sizeof(float));
        for( int i = 0; i < num_children; i++ )
          {
            QModelIndex child_index = model_->index( i, 0, QModelIndex());
            VectorProperty* vec_data = qobject_cast<VectorProperty* >(model_->getProp( child_index )->childAt(0));
            ColorProperty* color_data = qobject_cast<ColorProperty* >(model_->getProp( child_index )->childAt(1));

            Ogre::Vector3 point_vec = vec_data->getVector();
            // check if color_data is available
            // if not color_data is available, set the color to black(0,0,0)
            int rgb_int = 0;
            if (color_data != NULL && color_data->getColor().isValid()) {
              Ogre::ColourValue point_color = color_data->getOgreColor();
              rgb_int = (int)point_color.r << 16 | (int)point_color.g << 8 |  (int)point_color.b << 0;
            }
            float x = point_vec.x, y = point_vec.y, z = point_vec.z;
            //Tty to add color, but point_color's value are all zero!!!!!!
            float rgb_float = *reinterpret_cast<float*>(&rgb_int);
            memcpy(&pc2.data[i*4*sizeof(float)], &x, sizeof(float));
            memcpy(&pc2.data[(i*4+1)*sizeof(float)], &y, sizeof(float));
            memcpy(&pc2.data[(i*4+2)*sizeof(float)], &z, sizeof(float));
            memcpy(&pc2.data[(i*4+3)*sizeof(float)], &rgb_float, sizeof(float));
          }

        pc2.point_step = 16;
        pc2.row_step = pc2.point_step * pc2.width;
        pc2.is_dense = false;
        select_pointcloud_publisher_.publish(pc2);
      }
  }

  void SelectPointCloudPublishAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  // Load all configuration data for this panel from the given Config object.
  void SelectPointCloudPublishAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::SelectPointCloudPublishAction, rviz::Panel )
