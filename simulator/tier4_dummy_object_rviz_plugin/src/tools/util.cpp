// Copyright 2022 TIER IV, Inc.
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

#include "util.hpp"

#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>

#include <OgreCamera.h>
#include <OgreRay.h>
#include <OgreViewport.h>

std::optional<Ogre::Vector3> get_point_from_mouse(rviz_common::ViewportMouseEvent & event)
{
  using rviz_rendering::RenderWindowOgreAdapter;
  const auto viewport = RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  const auto w = viewport->getActualWidth();
  const auto h = viewport->getActualHeight();
  const auto x = static_cast<Ogre::Real>(event.x) / static_cast<Ogre::Real>(w);
  const auto y = static_cast<Ogre::Real>(event.y) / static_cast<Ogre::Real>(h);

  const auto plane = Ogre::Plane(Ogre::Vector3::UNIT_Z, 0.0);
  const auto ray = viewport->getCamera()->getCameraToViewportRay(x, y);
  const auto intersect = ray.intersects(plane);
  return intersect.first ? std::optional(ray.getPoint(intersect.second)) : std::nullopt;
}
