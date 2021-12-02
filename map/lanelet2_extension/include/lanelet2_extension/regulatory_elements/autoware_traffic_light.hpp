// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
//
// Authors: Ryohsuke Mitsudome

#ifndef LANELET2_EXTENSION__REGULATORY_ELEMENTS__AUTOWARE_TRAFFIC_LIGHT_HPP_
#define LANELET2_EXTENSION__REGULATORY_ELEMENTS__AUTOWARE_TRAFFIC_LIGHT_HPP_

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <vector>

namespace lanelet
{
namespace autoware
{
struct AutowareRoleNameString
{
  static constexpr const char LightBulbs[] = "light_bulbs";
};

class AutowareTrafficLight : public lanelet::TrafficLight
{
public:
  using Ptr = std::shared_ptr<AutowareTrafficLight>;
  static constexpr char RuleName[] = "traffic_light";

  //! Directly construct a stop line from its required rule parameters.
  //! Might modify the input data in oder to get correct tags.
  static Ptr make(
    Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficLights,
    const Optional<LineString3d> & stopLine = {}, const LineStrings3d & lightBulbs = {})
  {
    return Ptr{new AutowareTrafficLight(id, attributes, trafficLights, stopLine, lightBulbs)};
  }

  /**
   * @brief get the relevant traffic light bulbs
   * @return the traffic light bulbs
   *
   * There might be multiple traffic light bulbs but they are required to show
   * the same signal.
   */
  ConstLineStrings3d lightBulbs() const;

  /**
   * @brief add a new traffic light bulb
   * @param primitive the traffic light bulb to add
   *
   * Traffic light bulbs are represented as linestrings with each point
   * expressing position of each light bulb (lamp).
   */
  void addLightBulbs(const LineStringOrPolygon3d & primitive);

  /**
   * @brief remove a traffic light bulb
   * @param primitive the primitive
   * @return true if the traffic light bulb existed and was removed
   */
  bool removeLightBulbs(const LineStringOrPolygon3d & primitive);

private:
  // the following lines are required so that lanelet2 can create this object
  // when loading a map with this regulatory element
  friend class lanelet::RegisterRegulatoryElement<AutowareTrafficLight>;
  AutowareTrafficLight(
    Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficLights,
    const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs);
  explicit AutowareTrafficLight(const lanelet::RegulatoryElementDataPtr & data);
};
static lanelet::RegisterRegulatoryElement<AutowareTrafficLight> regAutowareTraffic;

// moved to lanelet2_extension/lib/autoware_traffic_light.cpp to avoid multiple
// definition errors
/*
#if __cplusplus < 201703L
constexpr char AutowareTrafficLight::RuleName[];      // instantiate string in
cpp file #endif
*/
}  // namespace autoware
}  // namespace lanelet

#endif  // LANELET2_EXTENSION__REGULATORY_ELEMENTS__AUTOWARE_TRAFFIC_LIGHT_HPP_
