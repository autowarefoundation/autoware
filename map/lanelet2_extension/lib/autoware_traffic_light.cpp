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

#include "lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"

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

template <>
RuleParameters toRuleParameters(const std::vector<LineStringOrPolygon3d> & primitives)
{
  auto cast_func = [](const auto & elem) { return elem.asRuleParameter(); };
  return utils::transform(primitives, cast_func);
}

LineStringsOrPolygons3d getLsOrPoly(const RuleParameterMap & paramsMap, RoleName role)
{
  auto params = paramsMap.find(role);
  if (params == paramsMap.end()) {
    return {};
  }
  LineStringsOrPolygons3d result;
  for (auto & param : params->second) {
    auto l = boost::get<LineString3d>(&param);
    if (l != nullptr) {
      result.push_back(*l);
    }
    auto p = boost::get<Polygon3d>(&param);
    if (p != nullptr) {
      result.push_back(*p);
    }
  }
  return result;
}

[[maybe_unused]] ConstLineStringsOrPolygons3d getConstLsOrPoly(
  const RuleParameterMap & params, RoleName role)
{
  auto cast_func = [](auto & lsOrPoly) {
    return static_cast<ConstLineStringOrPolygon3d>(lsOrPoly);
  };
  return utils::transform(getLsOrPoly(params, role), cast_func);
}

[[maybe_unused]] RegulatoryElementDataPtr constructAutowareTrafficLightData(
  Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficLights,
  const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs)
{
  RuleParameterMap rpm = {{RoleNameString::Refers, toRuleParameters(trafficLights)}};

  if (!!stopLine) {
    RuleParameters rule_parameters = {*stopLine};
    rpm.insert(std::make_pair(RoleNameString::RefLine, rule_parameters));
  }
  if (!lightBulbs.empty()) {
    rpm.insert(std::make_pair(AutowareRoleNameString::LightBulbs, toRuleParameters(lightBulbs)));
  }

  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = AttributeValueString::TrafficLight;
  return data;
}
}  // namespace

constexpr const char AutowareRoleNameString::LightBulbs[];

AutowareTrafficLight::AutowareTrafficLight(const RegulatoryElementDataPtr & data)
: TrafficLight(data)
{
}

AutowareTrafficLight::AutowareTrafficLight(
  Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficLights,
  const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs)
: TrafficLight(id, attributes, trafficLights, stopLine)
{
  for (const auto & lightBulb : lightBulbs) {
    addLightBulbs(lightBulb);
  }
}

ConstLineStrings3d AutowareTrafficLight::lightBulbs() const
{
  return getParameters<ConstLineString3d>(AutowareRoleNameString::LightBulbs);
}

void AutowareTrafficLight::addLightBulbs(const LineStringOrPolygon3d & primitive)
{
  parameters()[AutowareRoleNameString::LightBulbs].emplace_back(primitive.asRuleParameter());
}

bool AutowareTrafficLight::removeLightBulbs(const LineStringOrPolygon3d & primitive)
{
  return findAndErase(
    primitive.asRuleParameter(), &parameters().find(AutowareRoleNameString::LightBulbs)->second);
}

#if __cplusplus < 201703L
constexpr char AutowareTrafficLight::RuleName[];  // instantiate string in cpp file
#endif

}  // namespace autoware
}  // namespace lanelet
