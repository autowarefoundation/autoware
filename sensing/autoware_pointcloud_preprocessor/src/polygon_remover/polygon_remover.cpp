// Copyright 2022 The Autoware Contributors
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

#include "autoware/pointcloud_preprocessor/polygon_remover/polygon_remover.hpp"

namespace autoware::pointcloud_preprocessor
{
PolygonRemoverComponent::PolygonRemoverComponent(const rclcpp::NodeOptions & options)
: Filter("PolygonRemover", options)
{
  pub_marker_ptr_ = this->create_publisher<visualization_msgs::msg::Marker>("Removed_polygon", 10);

  this->declare_parameter<std::vector<double>>("polygon_vertices");
  this->get_parameter("polygon_vertices", param);
  this->declare_parameter<bool>("will_visualize");
  this->get_parameter("will_visualize", will_visualize_);
  polygon_vertices_ = param.as_double_array();
  if (polygon_vertices_.size() % 2 != 0) {
    throw std::length_error(
      "polygon_vertices has odd number of elements. "
      "It must have a list of x,y double pairs.");
  }

  auto make_point = [](float x, float y, float z) {
    geometry_msgs::msg::Point32 point_32;
    point_32.set__x(x);
    point_32.set__y(y);
    point_32.set__z(z);
    return point_32;
  };
  polygon_ = std::make_shared<geometry_msgs::msg::Polygon>();
  for (size_t i = 0UL; i < polygon_vertices_.size(); i += 2) {
    auto p_x = static_cast<float>(polygon_vertices_.at(i));
    auto p_y = static_cast<float>(polygon_vertices_.at(i + 1));
    polygon_->points.emplace_back(make_point(p_x, p_y, 0.0F));
  }
  this->update_polygon(polygon_);
}

void PolygonRemoverComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }

  if (!this->polygon_is_initialized_) {
    RCLCPP_INFO_STREAM(get_logger(), "Polygon is not initialized, publishing incoming cloud.");
    output = *input;
    return;
  }

  output = this->remove_updated_polygon_from_cloud(input);
  if (will_visualize_) {
    pub_marker_ptr_->publish(marker_);
  }
}

void PolygonRemoverComponent::update_polygon(
  const geometry_msgs::msg::Polygon::ConstSharedPtr & polygon_in)
{
  autoware::pointcloud_preprocessor::utils::to_cgal_polygon(*polygon_in, polygon_cgal_);
  if (will_visualize_) {
    marker_.ns = "";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.02;
    marker_.color.a = 1.0;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;

    auto make_point = [](float x, float y, float z) {
      geometry_msgs::msg::Point point;
      point.x = static_cast<double>(x);
      point.y = static_cast<double>(y);
      point.z = static_cast<double>(z);
      return point;
    };
    for (size_t index_cur = 0; index_cur < polygon_cgal_.size(); ++index_cur) {
      const auto & vertex = polygon_cgal_.at(index_cur);

      // Take the last segment into consideration to connect the loop
      size_t index_next = index_cur == polygon_cgal_.size() - 1 ? 0UL : index_cur + 1;
      const auto & vertex_next = polygon_cgal_.at(index_next);

      // Build upper ring
      auto vertex_up_cur =
        make_point(static_cast<float>(vertex.x()), static_cast<float>(vertex.y()), 5.0F);
      auto vertex_up_next =
        make_point(static_cast<float>(vertex_next.x()), static_cast<float>(vertex_next.y()), 5.0F);
      marker_.points.emplace_back(vertex_up_cur);
      marker_.points.emplace_back(vertex_up_next);

      // Build lower ring
      auto vertex_down_cur =
        make_point(static_cast<float>(vertex.x()), static_cast<float>(vertex.y()), -5.0F);
      auto vertex_down_next =
        make_point(static_cast<float>(vertex_next.x()), static_cast<float>(vertex_next.y()), -5.0F);
      marker_.points.emplace_back(vertex_down_cur);
      marker_.points.emplace_back(vertex_down_next);

      // Connect up and down vertices
      marker_.points.emplace_back(vertex_up_cur);
      marker_.points.emplace_back(vertex_down_cur);
    }
  }
  polygon_is_initialized_ = true;
}

sensor_msgs::msg::PointCloud2 PolygonRemoverComponent::remove_updated_polygon_from_cloud(
  const PointCloud2ConstPtr & cloud_in)
{
  if (will_visualize_) {
    marker_.header.frame_id = cloud_in->header.frame_id;
  }
  if (!polygon_is_initialized_) {
    throw std::runtime_error("Polygon is not initialized. Please use `update_polygon` first.");
  }

  PointCloud2 cloud_out;
  autoware::pointcloud_preprocessor::utils::remove_polygon_cgal_from_cloud(
    *cloud_in, polygon_cgal_, cloud_out);
  return cloud_out;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PolygonRemoverComponent)
