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

#include "lanelet2_extension/regulatory_elements/detection_area.hpp"

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
template <typename T>
bool findAndErase(const T & primitive, RuleParameters * member)
{
  if (member == nullptr) {
    std::cerr << __FUNCTION__ << ": member is null pointer";
    return false;
  }
  auto it = std::find(member->begin(), member->end(), RuleParameter(primitive));
  if (it == member->end()) {
    return false;
  }
  member->erase(it);
  return true;
}

template <typename T>
RuleParameters toRuleParameters(const std::vector<T> & primitives)
{
  auto cast_func = [](const auto & elem) { return static_cast<RuleParameter>(elem); };
  return utils::transform(primitives, cast_func);
}

// template <>
// RuleParameters toRuleParameters(const std::vector<Polygon3d>& primitives)
// {
//   auto cast_func = [](const auto& elem) { return elem.asRuleParameter(); };
//   return utils::transform(primitives, cast_func);
// }

Polygons3d getPoly(const RuleParameterMap & paramsMap, RoleName role)
{
  auto params = paramsMap.find(role);
  if (params == paramsMap.end()) {
    return {};
  }

  Polygons3d result;
  for (auto & param : params->second) {
    auto p = boost::get<Polygon3d>(&param);
    if (p != nullptr) {
      result.push_back(*p);
    }
  }
  return result;
}

ConstPolygons3d getConstPoly(const RuleParameterMap & params, RoleName role)
{
  auto cast_func = [](auto & poly) { return static_cast<ConstPolygon3d>(poly); };
  return utils::transform(getPoly(params, role), cast_func);
}

RegulatoryElementDataPtr constructDetectionAreaData(
  Id id, const AttributeMap & attributes, const Polygons3d & detectionAreas,
  const LineString3d & stopLine)
{
  RuleParameterMap rpm = {{RoleNameString::Refers, toRuleParameters(detectionAreas)}};

  RuleParameters rule_parameters = {stopLine};
  rpm.insert(std::make_pair(RoleNameString::RefLine, rule_parameters));

  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = "detection_area";
  return data;
}
}  // namespace

DetectionArea::DetectionArea(const RegulatoryElementDataPtr & data) : RegulatoryElement(data)
{
  if (getConstPoly(data->parameters, RoleName::Refers).empty()) {
    throw InvalidInputError("No detection area defined!");
  }
  if (getParameters<ConstLineString3d>(RoleName::RefLine).size() != 1) {
    throw InvalidInputError("There must be exactly one stopline defined!");
  }
}

DetectionArea::DetectionArea(
  Id id, const AttributeMap & attributes, const Polygons3d & detectionAreas,
  const LineString3d & stopLine)
: DetectionArea(constructDetectionAreaData(id, attributes, detectionAreas, stopLine))
{
}

ConstPolygons3d DetectionArea::detectionAreas() const
{
  return getConstPoly(parameters(), RoleName::Refers);
}
Polygons3d DetectionArea::detectionAreas() { return getPoly(parameters(), RoleName::Refers); }

void DetectionArea::addDetectionArea(const Polygon3d & primitive)
{
  parameters()["detection_area"].emplace_back(primitive);
}

bool DetectionArea::removeDetectionArea(const Polygon3d & primitive)
{
  return findAndErase(primitive, &parameters().find("detection_area")->second);
}

ConstLineString3d DetectionArea::stopLine() const
{
  return getParameters<ConstLineString3d>(RoleName::RefLine).front();
}

LineString3d DetectionArea::stopLine()
{
  return getParameters<LineString3d>(RoleName::RefLine).front();
}

void DetectionArea::setStopLine(const LineString3d & stopLine)
{
  parameters()[RoleName::RefLine] = {stopLine};
}

void DetectionArea::removeStopLine() { parameters()[RoleName::RefLine] = {}; }

#if __cplusplus < 201703L
constexpr char DetectionArea::RuleName[];  // instantiate string in cpp file
#endif

}  // namespace autoware
}  // namespace lanelet
