#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QLabel>

#include "transformable_marker_operator.h"

using namespace rviz;
namespace jsk_rviz_plugins
{
  TransformableMarkerOperatorAction::TransformableMarkerOperatorAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    layout = new QVBoxLayout;

    QVBoxLayout* layout1 = new QVBoxLayout;
    QVBoxLayout* layout2 = new QVBoxLayout;

    QTabWidget* tabs = new QTabWidget();

    QWidget* tab_1 = new QWidget();
    QWidget* tab_2 = new QWidget();

    //Button to send cancel topic
    insert_box_button_ = new QPushButton("Insert New Box Marker");
    layout1->addWidget( insert_box_button_ );

    insert_cylinder_button_ = new QPushButton("Insert New Cylinder Marker");
    layout1->addWidget( insert_cylinder_button_ );

    insert_torus_button_ = new QPushButton("Insert New Torus Marker");
    layout1->addWidget( insert_torus_button_ );

    QHBoxLayout* name_layout = new QHBoxLayout;
    name_layout->addWidget( new QLabel( "Name:" ));
    name_editor_ = new QLineEdit;
    name_layout->addWidget( name_editor_ );
    layout1->addLayout( name_layout );

    QHBoxLayout* description_layout = new QHBoxLayout;
    description_layout->addWidget( new QLabel( "Description:" ));
    description_editor_ = new QLineEdit;
    description_layout->addWidget( description_editor_ );
    layout1->addLayout( description_layout );

    QHBoxLayout* frame_layout = new QHBoxLayout;
    frame_layout->addWidget( new QLabel( "Frame:" ));
    frame_editor_ = new QLineEdit;
    frame_layout->addWidget( frame_editor_ );
    layout1->addLayout( frame_layout );

    erase_with_id_button_ = new QPushButton("Erase with id");
    layout2->addWidget( erase_with_id_button_ );

    QHBoxLayout* id_layout = new QHBoxLayout;
    id_layout->addWidget( new QLabel( "Id:" ));
    id_editor_ = new QLineEdit;
    id_layout->addWidget( id_editor_ );
    layout2->addLayout( id_layout );

    erase_all_button_ = new QPushButton("Erase all");
    layout2->addWidget( erase_all_button_ );

    erase_focus_button_ = new QPushButton("Erase focus");
    layout2->addWidget( erase_focus_button_ );

    tab_1->setLayout( layout1 );
    tab_2->setLayout( layout2 );

    tabs->addTab(tab_1, QString("Insert"));
    tabs->addTab(tab_2, QString("Erase"));

    layout->addWidget( tabs );
    setLayout( layout );

    connect( insert_box_button_, SIGNAL( clicked() ), this, SLOT( insertBoxService ()));
    connect( insert_cylinder_button_, SIGNAL( clicked() ), this, SLOT( insertCylinderService ()));
    connect( insert_torus_button_, SIGNAL( clicked() ), this, SLOT( insertTorusService ()));
    connect( erase_with_id_button_, SIGNAL( clicked() ), this, SLOT( eraseWithIdService ()));
    connect( erase_all_button_, SIGNAL( clicked() ), this, SLOT( eraseAllService ()));
    connect( erase_focus_button_, SIGNAL( clicked() ), this, SLOT( eraseFocusService ()));
  }

  void TransformableMarkerOperatorAction::insertBoxService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.type = jsk_rviz_plugins::TransformableMarkerOperate::BOX;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::INSERT;
    operator_srv.request.operate.name = name_editor_->text().toStdString();
    operator_srv.request.operate.description = description_editor_->text().toStdString();
    operator_srv.request.operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::insertCylinderService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.type = jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::INSERT;
    operator_srv.request.operate.name = name_editor_->text().toStdString();
    operator_srv.request.operate.description = description_editor_->text().toStdString();
    operator_srv.request.operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::insertTorusService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.type = jsk_rviz_plugins::TransformableMarkerOperate::TORUS;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::INSERT;
    operator_srv.request.operate.name = name_editor_->text().toStdString();
    operator_srv.request.operate.description = description_editor_->text().toStdString();
    operator_srv.request.operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::eraseWithIdService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::ERASE;
    operator_srv.request.operate.name = id_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::eraseAllService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::ERASEALL;
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::eraseFocusService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::ERASEFOCUS;
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::callRequestMarkerOperateService(jsk_rviz_plugins::RequestMarkerOperate srv){
    ros::ServiceClient client = nh_.serviceClient<jsk_rviz_plugins::RequestMarkerOperate>("request_marker_operate", true);
    if(client.call(srv))
      {
        ROS_INFO("Call Success");
      }
    else{
      ROS_ERROR("Service call FAIL");
    };
  }

  void TransformableMarkerOperatorAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void TransformableMarkerOperatorAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::TransformableMarkerOperatorAction, rviz::Panel )
