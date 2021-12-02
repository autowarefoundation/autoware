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

#include "lanelet2_extension/regulatory_elements/no_stopping_area.hpp"

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
Optional<T> tryGetFront(const std::vector<T> & vec)
{
  if (vec.empty()) {
    return {};
  }
  return vec.front();
}

template <typename T>
RuleParameters toRuleParameters(const std::vector<T> & primitives)
{
  auto cast_func = [](const auto & elem) { return static_cast<RuleParameter>(elem); };
  return utils::transform(primitives, cast_func);
}

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

RegulatoryElementDataPtr constructNoStoppingAreaData(
  Id id, const AttributeMap & attributes, const Polygons3d & noStoppingAreas,
  const Optional<LineString3d> & stopLine = {})
{
  RuleParameterMap rpm = {{RoleNameString::Refers, toRuleParameters(noStoppingAreas)}};
  if (!!stopLine) {
    rpm.insert({RoleNameString::RefLine, {*stopLine}});
  }

  auto data = std::make_shared<RegulatoryElementData>(id, std::move(rpm), attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = "no_stopping_area";
  return data;
}
}  // namespace

NoStoppingArea::NoStoppingArea(const RegulatoryElementDataPtr & data) : RegulatoryElement(data)
{
  if (getConstPoly(data->parameters, RoleName::Refers).empty()) {
    throw InvalidInputError("no stopping area defined!");
  }
  if (getParameters<ConstLineString3d>(RoleName::RefLine).size() > 1) {
    throw InvalidInputError("There can not exist more than one stop line!");
  }
}

NoStoppingArea::NoStoppingArea(
  Id id, const AttributeMap & attributes, const Polygons3d & no_stopping_areas,
  const Optional<LineString3d> & stopLine)
: NoStoppingArea(constructNoStoppingAreaData(id, attributes, no_stopping_areas, stopLine))
{
}

ConstPolygons3d NoStoppingArea::noStoppingAreas() const
{
  return getConstPoly(parameters(), RoleName::Refers);
}
Polygons3d NoStoppingArea::noStoppingAreas() { return getPoly(parameters(), RoleName::Refers); }

void NoStoppingArea::addNoStoppingArea(const Polygon3d & primitive)
{
  parameters()["no_stopping_area"].emplace_back(primitive);
}

bool NoStoppingArea::removeNoStoppingArea(const Polygon3d & primitive)
{
  return findAndErase(primitive, &parameters().find("no_stopping_area")->second);
}

Optional<ConstLineString3d> NoStoppingArea::stopLine() const
{
  return tryGetFront(getParameters<ConstLineString3d>(RoleName::RefLine));
}

Optional<LineString3d> NoStoppingArea::stopLine()
{
  return tryGetFront(getParameters<LineString3d>(RoleName::RefLine));
}

void NoStoppingArea::setStopLine(const LineString3d & stopLine)
{
  parameters()[RoleName::RefLine] = {stopLine};
}

void NoStoppingArea::removeStopLine() { parameters()[RoleName::RefLine] = {}; }

#if __cplusplus < 201703L
constexpr char NoStoppingArea::RuleName[];  // instantiate string in cpp file
#endif

}  // namespace autoware
}  // namespace lanelet
