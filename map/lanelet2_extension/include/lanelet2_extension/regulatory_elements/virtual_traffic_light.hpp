// Copyright 2021 Tier IV, Inc.
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

#ifndef LANELET2_EXTENSION__REGULATORY_ELEMENTS__VIRTUAL_TRAFFIC_LIGHT_HPP_
#define LANELET2_EXTENSION__REGULATORY_ELEMENTS__VIRTUAL_TRAFFIC_LIGHT_HPP_

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>

#include <memory>
#include <vector>

namespace lanelet
{
namespace autoware
{
class VirtualTrafficLight : public lanelet::RegulatoryElement
{
public:
  using Ptr = std::shared_ptr<VirtualTrafficLight>;
  using ConstPtr = std::shared_ptr<const VirtualTrafficLight>;
  static constexpr char RuleName[] = "virtual_traffic_light";

  static Ptr make(
    const Id id, const AttributeMap & attributes, const LineString3d & virtual_traffic_light)
  {
    return Ptr{new VirtualTrafficLight(id, attributes, virtual_traffic_light)};
  }

  ConstLineString3d getVirtualTrafficLight() const
  {
    return getParameters<ConstLineString3d>(RoleName::Refers).front();
  }

  Optional<ConstLineString3d> getStopLine() const
  {
    const auto vec = getParameters<ConstLineString3d>(RoleName::RefLine);
    if (vec.empty()) {
      return {};
    }
    return vec.front();
  }

  ConstLineString3d getStartLine() const
  {
    return getParameters<ConstLineString3d>("start_line").front();
  }

  ConstLineStrings3d getEndLines() const { return getParameters<ConstLineString3d>("end_line"); }

private:
  // the following lines are required so that lanelet2 can create this object
  // when loading a map with this regulatory element
  friend class lanelet::RegisterRegulatoryElement<VirtualTrafficLight>;

  VirtualTrafficLight(
    const Id id, const AttributeMap & attributes, const LineString3d & virtualTrafficLight);

  explicit VirtualTrafficLight(const lanelet::RegulatoryElementDataPtr & data);
};

static lanelet::RegisterRegulatoryElement<VirtualTrafficLight> regVirtualTrafficLight;

}  // namespace autoware
}  // namespace lanelet

#endif  // LANELET2_EXTENSION__REGULATORY_ELEMENTS__VIRTUAL_TRAFFIC_LIGHT_HPP_
