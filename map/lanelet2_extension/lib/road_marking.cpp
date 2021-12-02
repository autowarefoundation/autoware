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

#include "lanelet2_extension/regulatory_elements/road_marking.hpp"

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
RegulatoryElementDataPtr constructRoadMarkingData(
  Id id, const AttributeMap & attributes, const LineString3d & road_marking)
{
  RuleParameterMap rpm;
  RuleParameters rule_parameters = {road_marking};
  rpm.insert(std::make_pair(RoleNameString::Refers, rule_parameters));

  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = "road_marking";
  return data;
}
}  // namespace

RoadMarking::RoadMarking(const RegulatoryElementDataPtr & data) : RegulatoryElement(data)
{
  if (getParameters<ConstLineString3d>(RoleName::Refers).size() != 1) {
    throw InvalidInputError("There must be exactly one road marking defined!");
  }
}

RoadMarking::RoadMarking(Id id, const AttributeMap & attributes, const LineString3d & road_marking)
: RoadMarking(constructRoadMarkingData(id, attributes, road_marking))
{
}

ConstLineString3d RoadMarking::roadMarking() const
{
  return getParameters<ConstLineString3d>(RoleName::Refers).front();
}

LineString3d RoadMarking::roadMarking()
{
  return getParameters<LineString3d>(RoleName::Refers).front();
}

void RoadMarking::setRoadMarking(const LineString3d & road_marking)
{
  parameters()[RoleName::Refers] = {road_marking};
}

void RoadMarking::removeRoadMarking() { parameters()[RoleName::Refers] = {}; }

#if __cplusplus < 201703L
constexpr char RoadMarking::RuleName[];  // instantiate string in cpp file
#endif

}  // namespace autoware
}  // namespace lanelet
