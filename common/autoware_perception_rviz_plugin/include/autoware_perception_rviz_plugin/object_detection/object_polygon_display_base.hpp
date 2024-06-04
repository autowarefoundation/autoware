// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__OBJECT_POLYGON_DISPLAY_BASE_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__OBJECT_POLYGON_DISPLAY_BASE_HPP_

#include "autoware_perception_rviz_plugin/common/color_alpha_property.hpp"
#include "autoware_perception_rviz_plugin/object_detection/object_polygon_detail.hpp"
#include "autoware_perception_rviz_plugin/visibility_control.hpp"

#include <rviz_common/display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <bitset>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
/// \brief Base rviz plugin class for all object msg types. The class defines common properties
///        for the plugin and also defines common helper functions that can be used by its derived
///        classes.
/// \tparam MsgT PredictedObjects or TrackedObjects or DetectedObjects type
template <typename MsgT>
class AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC ObjectPolygonDisplayBase
: public rviz_common::RosTopicDisplay<MsgT>
{
public:
  using Color = std::array<float, 3U>;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerCommon = rviz_default_plugins::displays::MarkerCommon;
  using ObjectClassificationMsg = autoware_perception_msgs::msg::ObjectClassification;
  using RosTopicDisplay = rviz_common::RosTopicDisplay<MsgT>;

  using PolygonPropertyMap =
    std::unordered_map<ObjectClassificationMsg::_label_type, common::ColorAlphaProperty>;

  explicit ObjectPolygonDisplayBase(const std::string & default_topic)
  : m_marker_common(this),
    // m_display_type_property{"Polygon Type", "3d", "Type of the polygon to display object", this},
    m_display_label_property{"Display Label", true, "Enable/disable label visualization", this},
    m_display_uuid_property{"Display UUID", true, "Enable/disable uuid visualization", this},
    m_display_velocity_text_property{
      "Display Velocity", true, "Enable/disable velocity text visualization", this},
    m_display_acceleration_text_property{
      "Display Acceleration", true, "Enable/disable acceleration text visualization", this},
    m_display_pose_covariance_property{
      "Display Pose Covariance", true, "Enable/disable pose covariance visualization", this},
    m_display_yaw_covariance_property{
      "Display Yaw Covariance", false, "Enable/disable yaw covariance visualization", this},
    m_display_twist_property{"Display Twist", true, "Enable/disable twist visualization", this},
    m_display_twist_covariance_property{
      "Display Twist Covariance", false, "Enable/disable twist covariance visualization", this},
    m_display_yaw_rate_property{
      "Display Yaw Rate", false, "Enable/disable yaw rate visualization", this},
    m_display_yaw_rate_covariance_property{
      "Display Yaw Rate Covariance", false, "Enable/disable yaw rate covariance visualization",
      this},
    m_display_predicted_paths_property{
      "Display Predicted Paths", true, "Enable/disable predicted paths visualization", this},
    m_display_path_confidence_property{
      "Display Predicted Path Confidence", true, "Enable/disable predicted paths visualization",
      this},

    m_display_existence_probability_property{
      "Display Existence Probability", false, "Enable/disable existence probability visualization",
      this},

    m_line_width_property{"Line Width", 0.03, "Line width of object-shape", this},
    m_default_topic{default_topic}
  {
    m_display_type_property = new rviz_common::properties::EnumProperty(
      "Polygon Type", "3d", "Type of the polygon to display object.", this);
    // Option values here must correspond to indices in palette_textures_ array in onInitialize()
    // below.
    m_display_type_property->addOption("3d", 0);
    m_display_type_property->addOption("2d", 1);
    m_display_type_property->addOption("Disable", 2);
    m_simple_visualize_mode_property = new rviz_common::properties::EnumProperty(
      "Visualization Type", "Normal", "Simplicity of the polygon to display object.", this);
    m_simple_visualize_mode_property->addOption("Normal", 0);
    m_simple_visualize_mode_property->addOption("Simple", 1);
    // Confidence interval property
    m_confidence_interval_property = new rviz_common::properties::EnumProperty(
      "Confidence Interval", "95%", "Confidence interval of state estimations.", this);
    m_confidence_interval_property->addOption("70%", 0);
    m_confidence_interval_property->addOption("85%", 1);
    m_confidence_interval_property->addOption("95%", 2);
    m_confidence_interval_property->addOption("99%", 3);

    m_object_fill_type_property = new rviz_common::properties::EnumProperty(
      "Object Fill Type", "skeleton", "Change object fill type in visualization", this);
    m_object_fill_type_property->addOption(
      "skeleton", static_cast<int>(detail::ObjectFillType::Skeleton));
    m_object_fill_type_property->addOption("Fill", static_cast<int>(detail::ObjectFillType::Fill));

    // iterate over default values to create and initialize the properties.
    for (const auto & map_property_it : detail::kDefaultObjectPropertyValues) {
      const auto & class_property_values = map_property_it.second;
      const auto & color = class_property_values.color;
      // This is just a parent property to contain the necessary properties for the given class:
      m_class_group_properties.emplace_back(
        class_property_values.label.c_str(), QVariant(),
        "Groups polygon properties for the given class", this);
      auto & parent_property = m_class_group_properties.back();
      // Associate a color and opacity property for the given class and attach them to the
      // parent property of the class so they can have a drop down view from the label property:
      m_polygon_properties.emplace(
        std::piecewise_construct, std::forward_as_tuple(map_property_it.first),
        std::forward_as_tuple(
          QColor{color[0], color[1], color[2]}, class_property_values.alpha, &parent_property));
    }
    init_color_list(predicted_path_colors);
  }

  void onInitialize() override
  {
    RosTopicDisplay::RTDClass::onInitialize();
    m_marker_common.initialize(this->context_, this->scene_node_);
    QString message_type = QString::fromStdString(rosidl_generator_traits::name<MsgT>());
    this->topic_property_->setMessageType(message_type);
    this->topic_property_->setValue(m_default_topic.c_str());
    this->topic_property_->setDescription("Topic to subscribe to.");
  }

  void load(const rviz_common::Config & config) override
  {
    RosTopicDisplay::Display::load(config);
    m_marker_common.load(config);
  }

  void update(float wall_dt, float ros_dt) override { m_marker_common.update(wall_dt, ros_dt); }

  void reset() override
  {
    RosTopicDisplay::reset();
    m_marker_common.clearMarkers();
  }

  void clear_markers() { m_marker_common.clearMarkers(); }

  void add_marker(visualization_msgs::msg::Marker::ConstSharedPtr marker_ptr)
  {
    m_marker_common.addMessage(marker_ptr);
  }

  void add_marker(visualization_msgs::msg::MarkerArray::ConstSharedPtr markers_ptr)
  {
    m_marker_common.addMessage(markers_ptr);
  }

  void deleteMarker(rviz_default_plugins::displays::MarkerID marker_id)
  {
    m_marker_common.deleteMarker(marker_id);
  }

protected:
  /// \brief Convert given shape msg into a Marker
  /// \tparam ClassificationContainerT List type with ObjectClassificationMsg
  /// \param shape_msg Shape msg to be converted
  /// \param centroid Centroid position of the shape in Object.header.frame_id frame
  /// \param orientation Orientation of the shape in Object.header.frame_id frame
  /// \param labels List of ObjectClassificationMsg objects
  /// \param line_width Line thickness around the object
  /// \return Marker ptr. Id and header will have to be set by the caller
  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_shape_marker_ptr(
    const autoware_perception_msgs::msg::Shape & shape_msg,
    const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
    const ClassificationContainerT & labels, const double & line_width,
    const bool & is_orientation_available) const
  {
    const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
    const auto fill_type =
      static_cast<detail::ObjectFillType>(m_object_fill_type_property->getOptionInt());

    if (m_display_type_property->getOptionInt() == 0) {
      return detail::get_shape_marker_ptr(
        shape_msg, centroid, orientation, color_rgba, line_width, is_orientation_available,
        fill_type);
    } else if (m_display_type_property->getOptionInt() == 1) {
      return detail::get_2d_shape_marker_ptr(
        shape_msg, centroid, orientation, color_rgba, line_width, is_orientation_available);
    } else {
      return std::nullopt;
    }
  }

  template <typename ClassificationContainerT>
  visualization_msgs::msg::Marker::SharedPtr get_2d_shape_marker_ptr(
    const autoware_perception_msgs::msg::Shape & shape_msg,
    const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
    const std_msgs::msg::ColorRGBA & color_rgba, const double & line_width,
    const bool & is_orientation_available);

  /// \brief Convert given shape msg into a Marker to visualize label name
  /// \tparam ClassificationContainerT List type with ObjectClassificationMsg
  /// \param centroid Centroid position of the shape in Object.header.frame_id frame
  /// \param labels List of ObjectClassificationMsg objects
  /// \return Marker ptr. Id and header will have to be set by the caller
  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_label_marker_ptr(
    const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
    const ClassificationContainerT & labels) const
  {
    if (m_display_label_property.getBool()) {
      const std::string label = get_best_label(labels);
      const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
      return detail::get_label_marker_ptr(centroid, orientation, label, color_rgba);
    } else {
      return std::nullopt;
    }
  }
  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_existence_probability_marker_ptr(
    const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
    const float existence_probability, const ClassificationContainerT & labels) const
  {
    if (m_display_existence_probability_property.getBool()) {
      const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
      return detail::get_existence_probability_marker_ptr(
        centroid, orientation, existence_probability, color_rgba);
    } else {
      return std::nullopt;
    }
  }

  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_uuid_marker_ptr(
    const unique_identifier_msgs::msg::UUID & uuid, const geometry_msgs::msg::Point & centroid,
    const ClassificationContainerT & labels) const
  {
    if (m_display_uuid_property.getBool()) {
      const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
      const std::string uuid_str = uuid_to_string(uuid);
      return detail::get_uuid_marker_ptr(uuid_str, centroid, color_rgba);
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_pose_covariance_marker_ptr(
    const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance) const
  {
    if (m_display_pose_covariance_property.getBool()) {
      return detail::get_pose_covariance_marker_ptr(pose_with_covariance, get_confidence_region());
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_yaw_covariance_marker_ptr(
    const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance, const double & length,
    const double & line_width) const
  {
    if (m_display_yaw_covariance_property.getBool()) {
      return detail::get_yaw_covariance_marker_ptr(
        pose_with_covariance, length, get_confidence_interval(), line_width);
    } else {
      return std::nullopt;
    }
  }

  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_velocity_text_marker_ptr(
    const geometry_msgs::msg::Twist & twist, const geometry_msgs::msg::Point & vis_pos,
    const ClassificationContainerT & labels) const
  {
    if (m_display_velocity_text_property.getBool()) {
      const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
      return detail::get_velocity_text_marker_ptr(twist, vis_pos, color_rgba);
    } else {
      return std::nullopt;
    }
  }

  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_acceleration_text_marker_ptr(
    const geometry_msgs::msg::Accel & accel, const geometry_msgs::msg::Point & vis_pos,
    const ClassificationContainerT & labels) const
  {
    if (m_display_acceleration_text_property.getBool()) {
      const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
      return detail::get_acceleration_text_marker_ptr(accel, vis_pos, color_rgba);
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_twist_marker_ptr(
    const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
    const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance,
    const double & line_width) const
  {
    if (m_display_twist_property.getBool()) {
      return detail::get_twist_marker_ptr(pose_with_covariance, twist_with_covariance, line_width);
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_twist_covariance_marker_ptr(
    const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
    const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance) const
  {
    if (m_display_twist_covariance_property.getBool()) {
      return detail::get_twist_covariance_marker_ptr(
        pose_with_covariance, twist_with_covariance, get_confidence_region());
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_yaw_rate_marker_ptr(
    const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
    const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance,
    const double & line_width) const
  {
    if (m_display_yaw_rate_property.getBool()) {
      return detail::get_yaw_rate_marker_ptr(
        pose_with_covariance, twist_with_covariance, line_width);
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_yaw_rate_covariance_marker_ptr(
    const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
    const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance,
    const double & line_width) const
  {
    if (m_display_yaw_rate_covariance_property.getBool()) {
      return detail::get_yaw_rate_covariance_marker_ptr(
        pose_with_covariance, twist_with_covariance, get_confidence_interval(), line_width);
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_predicted_path_marker_ptr(
    const unique_identifier_msgs::msg::UUID & uuid,
    const autoware_perception_msgs::msg::Shape & shape,
    const autoware_perception_msgs::msg::PredictedPath & predicted_path) const
  {
    if (m_display_predicted_paths_property.getBool()) {
      const std::string uuid_str = uuid_to_string(uuid);
      const std_msgs::msg::ColorRGBA predicted_path_color = get_color_from_uuid(uuid_str);
      return detail::get_predicted_path_marker_ptr(
        shape, predicted_path, predicted_path_color,
        m_simple_visualize_mode_property->getOptionInt() == 1);
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_path_confidence_marker_ptr(
    const unique_identifier_msgs::msg::UUID & uuid,
    const autoware_perception_msgs::msg::PredictedPath & predicted_path) const
  {
    if (m_display_path_confidence_property.getBool()) {
      const std::string uuid_str = uuid_to_string(uuid);
      const std_msgs::msg::ColorRGBA path_confidence_color = get_color_from_uuid(uuid_str);
      return detail::get_path_confidence_marker_ptr(predicted_path, path_confidence_color);
    } else {
      return std::nullopt;
    }
  }

  /// \brief Get color and alpha values based on the given list of classification values
  /// \tparam ClassificationContainerT Container of ObjectClassification
  /// \param labels list of classifications
  /// \return Color and alpha for the best class in the given list. Unknown class is used in
  ///         degenerate cases
  template <typename ClassificationContainerT>
  std_msgs::msg::ColorRGBA get_color_rgba(const ClassificationContainerT & labels) const
  {
    static const std::string kLoggerName("ObjectPolygonDisplayBase");
    const auto label = detail::get_best_label(labels, kLoggerName);
    auto it = m_polygon_properties.find(label);
    if (it == m_polygon_properties.end()) {
      it = m_polygon_properties.find(ObjectClassificationMsg::UNKNOWN);
    }
    return it->second;
  }

  /// \brief Get color and alpha values based on the given list of classification values
  /// \tparam ClassificationContainerT Container of ObjectClassification
  /// \param labels list of classifications
  /// \return best label string
  template <typename ClassificationContainerT>
  std::string get_best_label(const ClassificationContainerT & labels) const
  {
    static const std::string kLoggerName("ObjectPolygonDisplayBase");
    const auto label = detail::get_best_label(labels, kLoggerName);
    auto it = detail::kDefaultObjectPropertyValues.find(label);
    if (it == detail::kDefaultObjectPropertyValues.end()) {
      it = detail::kDefaultObjectPropertyValues.find(ObjectClassificationMsg::UNKNOWN);
    }
    return (it->second).label;
  }
  std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & u) const
  {
    std::stringstream ss;
    for (auto i = 0; i < 16; ++i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << +u.uuid[i];
    }
    return ss.str();
  }

  std_msgs::msg::ColorRGBA AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC
  get_color_from_uuid(const std::string & uuid) const
  {
    int i = (static_cast<int>(uuid.at(0)) * 4 + static_cast<int>(uuid.at(1))) %
            static_cast<int>(predicted_path_colors.size());

    std_msgs::msg::ColorRGBA color;
    color.r = predicted_path_colors.at(i).r;
    color.g = predicted_path_colors.at(i).g;
    color.b = predicted_path_colors.at(i).b;
    return color;
  }

  void init_color_list(std::vector<std_msgs::msg::ColorRGBA> & colors) const
  {
    std_msgs::msg::ColorRGBA sample_color;
    sample_color.r = 1.0;
    sample_color.g = 0.65;
    sample_color.b = 0.0;
    colors.push_back(sample_color);  // orange
    sample_color.r = 1.0;
    sample_color.g = 1.0;
    sample_color.b = 0.0;
    colors.push_back(sample_color);  // yellow
    sample_color.r = 0.69;
    sample_color.g = 1.0;
    sample_color.b = 0.18;
    colors.push_back(sample_color);  // green yellow
    sample_color.r = 0.59;
    sample_color.g = 1.0;
    sample_color.b = 0.59;
    colors.push_back(sample_color);  // pale green
    sample_color.r = 0.5;
    sample_color.g = 1.0;
    sample_color.b = 0.0;
    colors.push_back(sample_color);  // chartreuse green
    sample_color.r = 0.0;
    sample_color.g = 1.0;
    sample_color.b = 1.0;
    colors.push_back(sample_color);  // cyan
    sample_color.r = 0.53;
    sample_color.g = 0.81;
    sample_color.b = 0.98;
    colors.push_back(sample_color);  // light skyblue
    sample_color.r = 1.0;
    sample_color.g = 0.41;
    sample_color.b = 0.71;
    colors.push_back(sample_color);  // hot pink
  }

  double get_line_width() { return m_line_width_property.getFloat(); }

  double get_confidence_interval() const
  {
    switch (m_confidence_interval_property->getOptionInt()) {
      case 0:
        // 70%
        return 1.036;
      case 1:
        // 85%
        return 1.440;
      case 2:
        // 95%
        return 1.960;
      case 3:
        // 99%
        return 2.576;
      default:
        return 1.960;
    }
  }

  double get_confidence_region() const
  {
    switch (m_confidence_interval_property->getOptionInt()) {
      case 0:
        // 70%
        return 1.552;
      case 1:
        // 85%
        return 1.802;
      case 2:
        // 95%
        return 2.448;
      case 3:
        // 99%
        return 3.035;
      default:
        return 2.448;
    }
  }

private:
  // All rviz plugins should have this. Should be initialized with pointer to this class
  MarkerCommon m_marker_common;
  // List is used to store the properties for classification in case we need to access them:
  std::list<rviz_common::properties::Property> m_class_group_properties;
  // Map to store class labels and its corresponding properties
  PolygonPropertyMap m_polygon_properties;
  // Property to choose type of visualization polygon
  rviz_common::properties::EnumProperty * m_display_type_property;
  // Property to choose simplicity of visualization polygon
  rviz_common::properties::EnumProperty * m_simple_visualize_mode_property;
  // Property to set confidence interval of state estimations
  rviz_common::properties::EnumProperty * m_confidence_interval_property;
  // Property to set visualization type
  rviz_common::properties::EnumProperty * m_object_fill_type_property;
  // Property to enable/disable label visualization
  rviz_common::properties::BoolProperty m_display_label_property;
  // Property to enable/disable uuid visualization
  rviz_common::properties::BoolProperty m_display_uuid_property;
  // Property to enable/disable velocity text visualization
  rviz_common::properties::BoolProperty m_display_velocity_text_property;
  // Property to enable/disable acceleration text visualization
  rviz_common::properties::BoolProperty m_display_acceleration_text_property;
  // Property to enable/disable pose with covariance visualization
  rviz_common::properties::BoolProperty m_display_pose_covariance_property;
  // Property to enable/disable yaw covariance visualization
  rviz_common::properties::BoolProperty m_display_yaw_covariance_property;
  // Property to enable/disable twist visualization
  rviz_common::properties::BoolProperty m_display_twist_property;
  // Property to enable/disable twist covariance visualization
  rviz_common::properties::BoolProperty m_display_twist_covariance_property;
  // Property to enable/disable yaw rate visualization
  rviz_common::properties::BoolProperty m_display_yaw_rate_property;
  // Property to enable/disable yaw rate covariance visualization
  rviz_common::properties::BoolProperty m_display_yaw_rate_covariance_property;
  // Property to enable/disable predicted paths visualization
  rviz_common::properties::BoolProperty m_display_predicted_paths_property;
  // Property to enable/disable predicted path confidence visualization
  rviz_common::properties::BoolProperty m_display_path_confidence_property;

  rviz_common::properties::BoolProperty m_display_existence_probability_property;

  // Property to decide line width of object shape
  rviz_common::properties::FloatProperty m_line_width_property;
  // Default topic name to be visualized
  std::string m_default_topic;

  std::vector<std_msgs::msg::ColorRGBA> predicted_path_colors;
};
}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__OBJECT_POLYGON_DISPLAY_BASE_HPP_
