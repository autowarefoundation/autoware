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

#include "lanelet2_extension/regulatory_elements/virtual_traffic_light.hpp"

#include <boost/variant.hpp>

#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace lanelet
{
namespace autoware
{
namespace
{
RegulatoryElementDataPtr constructVirtualTrafficLightData(
  const Id id, const AttributeMap & attributes, const LineString3d & virtual_traffic_light)
{
  RuleParameterMap rpm;
  RuleParameters rule_parameters = {virtual_traffic_light};
  rpm.insert(std::make_pair(RoleNameString::Refers, rule_parameters));

  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = "virtual_traffic_light";
  return data;
}
}  // namespace

VirtualTrafficLight::VirtualTrafficLight(const RegulatoryElementDataPtr & data)
: RegulatoryElement(data)
{
  if (getParameters<ConstLineString3d>("start_line").size() != 1) {
    throw InvalidInputError("There must be exactly one start_line defined!");
  }
  if (getParameters<ConstLineString3d>("end_line").empty()) {
    throw InvalidInputError("No end_line defined!");
  }
}

VirtualTrafficLight::VirtualTrafficLight(
  const Id id, const AttributeMap & attributes, const LineString3d & virtual_traffic_light)
: VirtualTrafficLight(constructVirtualTrafficLightData(id, attributes, virtual_traffic_light))
{
}

#if __cplusplus < 201703L
constexpr char VirtualTrafficLight::RuleName[];  // instantiate string in cpp file
#endif

}  // namespace autoware
}  // namespace lanelet
