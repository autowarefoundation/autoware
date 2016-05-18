#include "rviz/config.h"
#include "empty_service_call_interface.h"
#include <ros/package.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>

using namespace rviz;
namespace jsk_rviz_plugins
{
  EmptyServiceCallInterfaceAction::EmptyServiceCallInterfaceAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    parseROSParameters();

    QHBoxLayout* h_layout = new QHBoxLayout;
    h_layout->setAlignment(Qt::AlignLeft);
    layout = new QVBoxLayout();
    signal_mapper = new QSignalMapper(this);

    for(size_t i = 0; i < service_call_button_infos_.size();i++){
      ServiceCallButtonInfo target_button = service_call_button_infos_[i];
      QToolButton* tbutton = new QToolButton(this);
      tbutton->setText(target_button.text.c_str());
      tbutton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
      tbutton->setIconSize(QSize(100, 100));
      tbutton->setIcon(QIcon(QPixmap(QString(target_button.icon_file_path.c_str()))));
      connect(tbutton, SIGNAL(clicked()), signal_mapper, SLOT(map()));
      signal_mapper->setMapping(tbutton, i);
      h_layout->addWidget(tbutton);
    };
    connect(signal_mapper, SIGNAL(mapped(int)),
            this, SLOT(callRequestEmptyCommand(int)));
    layout->addLayout(h_layout);
    setLayout( layout );
  }

  void EmptyServiceCallInterfaceAction::parseROSParameters(){
    ros::NodeHandle nh("~");
    std::string buttons_names("emtpy_call_buttons");

    //Check the buttons
    std::string button_names;
    nh.param<std::string>(buttons_names.c_str(), button_names, std::string(""));

    //icon file package file_name
    std::string icon_package_name;
    nh.param<std::string>("icon_include_package", icon_package_name, std::string("jsk_rviz_plugins"));
    ROS_INFO("Find Icons In %s package.", icon_package_name.c_str());

    std::string icon_path_prefix;
    if(!icon_package_name.empty())
      icon_path_prefix = ros::package::getPath(icon_package_name) + std::string("/icons/");

    XmlRpc::XmlRpcValue buttons_list;
    nh.getParam("rviz_service_call/buttons", buttons_list);
    for (int32_t i = 0; i < buttons_list.size(); ++i)
      {
        ServiceCallButtonInfo new_button;
        new_button.icon_file_path = icon_path_prefix + static_cast<std::string>(buttons_list[i]["icon"]);
        new_button.service_name = static_cast<std::string>(buttons_list[i]["service_name"]);
        new_button.text = static_cast<std::string>(buttons_list[i]["text"]);
        service_call_button_infos_.push_back(new_button);
      }
  };

  void EmptyServiceCallInterfaceAction::callRequestEmptyCommand(const int button_id){
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>(service_call_button_infos_[button_id].service_name, true);
    std_srvs::Empty srv;
    if(client.call(srv))
      ROS_INFO("Call Success");
    else{
      ROS_ERROR("Service call FAIL");
    };
  }

  void EmptyServiceCallInterfaceAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void EmptyServiceCallInterfaceAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::EmptyServiceCallInterfaceAction, rviz::Panel )
