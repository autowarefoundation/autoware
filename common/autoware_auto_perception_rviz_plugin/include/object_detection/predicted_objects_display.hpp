// Copyright 2020 Tier IV, Inc.
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
#ifndef OBJECT_DETECTION__PREDICTED_OBJECTS_DISPLAY_HPP_
#define OBJECT_DETECTION__PREDICTED_OBJECTS_DISPLAY_HPP_

#include <object_detection/object_polygon_display_base.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <condition_variable>
#include <list>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
/// \brief Class defining rviz plugin to visualize PredictedObjects
class AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC PredictedObjectsDisplay
: public ObjectPolygonDisplayBase<autoware_auto_perception_msgs::msg::PredictedObjects>
{
  Q_OBJECT

public:
  using PredictedObjects = autoware_auto_perception_msgs::msg::PredictedObjects;

  PredictedObjectsDisplay();

private:
  void processMessage(PredictedObjects::ConstSharedPtr msg) override;

  boost::uuids::uuid to_boost_uuid(const unique_identifier_msgs::msg::UUID & uuid_msg)
  {
    const std::string uuid_str = uuid_to_string(uuid_msg);
    boost::uuids::string_generator gen;
    boost::uuids::uuid uuid = gen(uuid_str);
    return uuid;
  }

  void update_id_map(const PredictedObjects::ConstSharedPtr & msg)
  {
    std::vector<boost::uuids::uuid> new_uuids;
    std::vector<boost::uuids::uuid> tracked_uuids;
    new_uuids.reserve(msg->objects.size());
    tracked_uuids.reserve(msg->objects.size());
    for (const auto & object : msg->objects) {
      const auto uuid = to_boost_uuid(object.object_id);
      ((id_map.find(uuid) != id_map.end()) ? tracked_uuids : new_uuids).push_back(uuid);
    }

    auto itr = id_map.begin();
    while (itr != id_map.end()) {
      if (
        std::find(tracked_uuids.begin(), tracked_uuids.end(), itr->first) == tracked_uuids.end()) {
        unused_marker_ids.push_back(itr->second);
        itr = id_map.erase(itr);
      } else {
        ++itr;
      }
    }

    for (const auto & new_uuid : new_uuids) {
      if (unused_marker_ids.empty()) {
        id_map.emplace(new_uuid, marker_id);
        marker_id++;
      } else {
        id_map.emplace(new_uuid, unused_marker_ids.front());
        unused_marker_ids.pop_front();
      }
    }
  }

  int32_t uuid_to_marker_id(const unique_identifier_msgs::msg::UUID & uuid_msg)
  {
    auto uuid = to_boost_uuid(uuid_msg);
    return id_map.at(uuid);
  }

  std::vector<visualization_msgs::msg::Marker::SharedPtr> createMarkers(
    PredictedObjects::ConstSharedPtr msg);
  void workerThread();

  void update(float wall_dt, float ros_dt) override;

  std::unordered_map<boost::uuids::uuid, int32_t, boost::hash<boost::uuids::uuid>> id_map;
  // std::unordered_map<boost::uuids::uuid, int32_t> id_map;
  std::list<int32_t> unused_marker_ids;
  int32_t marker_id = 0;
  const int32_t PATH_ID_CONSTANT = 1e3;

  PredictedObjects::ConstSharedPtr msg;
  bool consumed{false};
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<visualization_msgs::msg::Marker::SharedPtr> markers;
};

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // OBJECT_DETECTION__PREDICTED_OBJECTS_DISPLAY_HPP_
