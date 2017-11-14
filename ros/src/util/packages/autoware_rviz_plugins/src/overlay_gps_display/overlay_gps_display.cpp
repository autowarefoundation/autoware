#include "overlay_gps_display.h"

namespace autoware_rviz_plugins {
OverlayGpsDisplay::OverlayGpsDisplay() {
  ros::param::get("/overlay_gps_display/google_static_map_api_key", api_key_);
  map_image_path_ =
      ros::package::getPath("autoware_rviz_plugins") + "/media/map.png";
  load_map_downloader_script();
  zoom_property_ = new rviz::IntProperty("Zoom", 19, "zoom of map", this,
                                         SLOT(updateGooleMapAPIProperty()));
  zoom_property_->setMax(22);
  zoom_property_->setMin(0);
  width_property_ =
      new rviz::IntProperty("Width", 320, "request map image width", this,
                            SLOT(updateGooleMapAPIProperty()));
  width_property_->setMax(640);
  width_property_->setMin(0);
  height_property_ =
      new rviz::IntProperty("Height", 320, "request map image height", this,
                            SLOT(updateGooleMapAPIProperty()));
  height_property_->setMax(640);
  height_property_->setMin(0);
  scale_property_ =
      new rviz::IntProperty("Scale", 1, "request map image scale", this,
                            SLOT(updateGooleMapAPIProperty()));
  scale_property_->setMax(2);
  scale_property_->setMin(1);
  history_length_property_ =
      new rviz::IntProperty("History Length", 15, "history length", this,
                            SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  fix_buffer_ = boost::circular_buffer<sensor_msgs::NavSatFix>(
      history_length_property_->getInt());
  // api_key_property_ = new rviz::StringProperty("API Key", "", "Google Static
  // Map API Key", this, SLOT(updateGooleMapAPIProperty()));
  maptype_property_ =
      new rviz::EnumProperty("Map Type", "roadmap", "map type", this,
                             SLOT(updateGooleMapAPIProperty()));
  maptype_property_->addOption("roadmap", ROADMAP);
  maptype_property_->addOption("terrain", TERRAIN);
  maptype_property_->addOption("satellite", SATELLITE);
  maptype_property_->addOption("hybrid", HYBRID);
  alpha_property_ = new rviz::FloatProperty("Alpha", 0.8, "image alpha", this,
                                            SLOT(updateDisplayProperty()));
  alpha_property_->setMax(1);
  alpha_property_->setMin(0);
  position_x_property_ =
      new rviz::IntProperty("Position X", 0, "map image position x", this,
                            SLOT(updateDisplayProperty()));
  position_y_property_ =
      new rviz::IntProperty("Position Y", 0, "map image position y", this,
                            SLOT(updateDisplayProperty()));
  messages_per_plot_property_ =
      new rviz::IntProperty("Message per plot", 5, "message per plot", this,
                            SLOT(updateDisplayProperty()));
  messages_per_plot_property_->setMin(1);
}

OverlayGpsDisplay::~OverlayGpsDisplay() {
  delete zoom_property_;
  delete width_property_;
  delete height_property_;
  delete scale_property_;
  delete alpha_property_;
  // delete api_key_property_;
  delete maptype_property_;
  remove(map_image_path_.c_str());
}

bool OverlayGpsDisplay::download_map(std::string request_url) {
  PyObject *args = PyTuple_New(1);
  PyObject *kw_args = PyDict_New();
  PyObject *request_url_str = PyString_FromString(request_url.c_str());
  PyTuple_SetItem(args, 0, request_url_str);
  try {
    PyObject *responce = PyObject_Call(map_downloader_function_, args, kw_args);
  } catch (...) {
    Py_DECREF(args);
    Py_DECREF(kw_args);
    return false;
  }
  Py_DECREF(args);
  Py_DECREF(kw_args);
  return true;
}

void OverlayGpsDisplay::onInitialize() { MFDClass::onInitialize(); }

void OverlayGpsDisplay::reset() { MFDClass::reset(); }

void OverlayGpsDisplay::processMessage(
    const sensor_msgs::NavSatFix::ConstPtr &msg) {
  if (msg->header.seq % messages_per_plot_property_->getInt() == 0) {
    fix_buffer_.push_back(*msg);
    std::string request_url;
    if (build_request_url(msg, request_url) == false) {
      return;
    }
    try {
      download_map(request_url);
    } catch (...) {
      ROS_ERROR_STREAM("failed to request map");
    }
    if (check_map_image_file() == false) {
      this->setStatus(rviz::StatusProperty::Level::Error,
                      "MapImageFileNotExist",
                      "map image file does not exist. Check API Key.");
      return;
    }
    this->setStatus(rviz::StatusProperty::Level::Ok, "MapImageFileNotExist",
                    "map image file exist");
    cv::Mat map_image = cv::imread(map_image_path_);
    if (map_image.cols <= 0 || map_image.rows <= 0) {
      this->setStatus(rviz::StatusProperty::Level::Error, "InvalidRequestURL",
                      "request url is invalid. Check API Key.");
      return;
    }
    this->setStatus(rviz::StatusProperty::Level::Ok, "InvalidRequestURL",
                    "request url is valid.");
    if (!overlay_) {
      static int count = 0;
      rviz::UniformStringStream ss;
      ss << "OverlayGpsDisplayObject" << count++;
      overlay_.reset(new OverlayObject(ss.str()));
      overlay_->show();
    }
    if (overlay_) {
      overlay_->setDimensions(width_property_->getInt(),
                              height_property_->getInt());
      overlay_->setPosition(position_x_property_->getInt(),
                            position_y_property_->getInt());
    }
    overlay_->updateTextureSize(width_property_->getInt(),
                                height_property_->getInt());
    ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage Hud = buffer.getQImage(*overlay_);
    for (int i = 0; i < overlay_->getTextureWidth(); i++) {
      for (int j = 0; j < overlay_->getTextureHeight(); j++) {
        QColor color(
            map_image.data[j * map_image.step + i * map_image.elemSize() + 2],
            map_image.data[j * map_image.step + i * map_image.elemSize() + 1],
            map_image.data[j * map_image.step + i * map_image.elemSize() + 0],
            alpha_property_->getFloat() * 255.0);
        Hud.setPixel(i, j, color.rgba());
      }
    }
  }
}

void OverlayGpsDisplay::updateGooleMapAPIProperty() {}

void OverlayGpsDisplay::updateDisplayProperty() {}

void OverlayGpsDisplay::updateHistoryLength() {
  fix_buffer_.clear();
  fix_buffer_ = boost::circular_buffer<sensor_msgs::NavSatFix>(
      history_length_property_->getInt());
}

void OverlayGpsDisplay::load_map_downloader_script() {
  std::string map_downloader_path =
      ros::package::getPath("autoware_rviz_plugins") +
      "/scripts/map_downloader.py";
  std::ifstream ifs(map_downloader_path.c_str());
  int begin = static_cast<int>(ifs.tellg());
  ifs.seekg(0, ifs.end);
  int end = static_cast<int>(ifs.tellg());
  int size = end - begin;
  ifs.clear();
  ifs.seekg(0, ifs.beg);
  char *map_downloader_script_ = new char[size + 1];
  map_downloader_script_[size] = '\0';
  ifs.read(map_downloader_script_, size);
  Py_Initialize();
  PyRun_SimpleString(map_downloader_script_);
  PyObject *script_obj = PyModule_GetDict(PyImport_ImportModule("__main__"));
  map_downloader_function_ = PyDict_GetItemString(script_obj, "download_map");
}

bool OverlayGpsDisplay::build_request_url(
    const sensor_msgs::NavSatFix::ConstPtr &msg, std::string &request_url) {
  request_url = "https://maps.googleapis.com/maps/api/staticmap?";
  /*
  if(api_key_property_->getStdString() == "")
  {
    this->setStatus(rviz::StatusProperty::Level::Error, "APIKey", "API key is
  not exist");
    return false;
  }
  */
  // this->setStatus(rviz::StatusProperty::Level::Ok, "APIKey", "API key is
  // exist");
  std::string center_request = "center=" + std::to_string(msg->latitude) + "," +
                               std::to_string(msg->longitude);
  request_url = request_url + center_request;
  std::string markers_request = "&markers=color:red%7C" +
                                std::to_string(msg->latitude) + "," +
                                std::to_string(msg->longitude);
  request_url = request_url + markers_request;
  std::string zoom_request =
      "&zoom=" + std::to_string(zoom_property_->getInt());
  request_url = request_url + zoom_request;
  std::string size_request = "&size=" +
                             std::to_string(width_property_->getInt()) + "x" +
                             std::to_string(height_property_->getInt());
  request_url = request_url + size_request;
  if (maptype_property_->getOptionInt() == ROADMAP) {
    std::string maptype_url = "&maptype=roadmap";
    request_url = request_url + maptype_url;
  }
  if (maptype_property_->getOptionInt() == TERRAIN) {
    std::string maptype_url = "&maptype=terrain";
    request_url = request_url + maptype_url;
  }
  if (maptype_property_->getOptionInt() == SATELLITE) {
    std::string maptype_url = "&maptype=satellite";
    request_url = request_url + maptype_url;
  }
  if (maptype_property_->getOptionInt() == HYBRID) {
    std::string maptype_url = "&maptype=hybrid";
    request_url = request_url + maptype_url;
  }
  std::string path_url = "&path=color:blue|weight:5";
  for (auto fix_data = fix_buffer_.rbegin(); fix_data != fix_buffer_.rend();
       ++fix_data) {
    path_url = path_url + "|" + std::to_string(fix_data->latitude) + "," +
               std::to_string(fix_data->longitude);
  }
  request_url = request_url + path_url;
  // request_url = request_url + "&format=jpg";

  std::string key_request =
      "&key=" + api_key_; // api_key_property_->getStdString();
  request_url = request_url + key_request;
  if (request_url.size() > MAX_REQUEST_URL_LENGTH) {
    QString message =
        QString("%1%2").arg(request_url.size()).arg(" request url is too long");
    this->setStatus(rviz::StatusProperty::Level::Error, "TooLongRequestUrl",
                    message);
    return false;
  } else {
    QString message = QString("%1%2").arg(request_url.size()).arg("characters");
    this->setStatus(rviz::StatusProperty::Level::Ok, "TooLongRequestUrl",
                    message);
  }
  // ROS_INFO_STREAM("request url:" << request_url);
  return true;
}

bool OverlayGpsDisplay::check_map_image_file() {
  std::ifstream ifs(map_image_path_);
  return ifs.is_open();
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::OverlayGpsDisplay, rviz::Display)
